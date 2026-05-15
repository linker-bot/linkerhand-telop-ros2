#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LinkerEG 遥操作手套串口通讯核心模块

协议特点:
- 波特率: 115200, 8-n-1
- 帧格式: [0xAA] [CMD] [LEN] [DATA...] [CHECKSUM] [0x55]
- 校验和: 补码累加和 checksum = ~(cmd + len + sum(data)) + 1
- 左右手共用同一串口
"""

import array
import struct
import time
import re
import os
import subprocess
import serial
import serial.tools.list_ports
from enum import Enum
from threading import Thread, Event
from typing import Optional, Tuple, List, Callable

# 帧常量
FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55
MAX_DATA_SIZE = 128
BUFFER_SIZE = 512

# 命令类型
class CmdType:
    READ_CONTROL_MODE = 0x0B # 读取控制方式
    RECEIVER_CONTROL = 0x0D  # 接收器控制灵巧手
    SDK_CONTROL = 0x0E       # SDK控制灵巧手
    ENABLE_RAW_DATA = 0x0F   # 启用传感器原始数据推送
    DISABLE_RAW_DATA = 0x10  # 禁用传感器原始数据推送
    ENABLE_MAPPED_DATA = 0x11  # 启用映射数据推送
    DISABLE_MAPPED_DATA = 0x12 # 禁用映射数据推送
    READ_VERSION = 0x14      # 读取版本号
    RAW_DATA_PUSH = 0x20     # 传感器原始数据推送 (50Hz)
    MAPPED_DATA_PUSH = 0x21  # 映射数据推送 (50Hz)

# 协议类型映射
PROTOCOL_MAP = {
    0: {'name': 'L20', 'joints': 16},
    1: {'name': 'L10', 'joints': 10},
    2: {'name': 'L21', 'joints': 16},
    3: {'name': 'L6',  'joints': 6},
    4: {'name': 'O7',  'joints': 7},
    5: {'name': 'G20', 'joints': 16},
}

# 结果码
class ResultCode:
    SUCCESS = 0x00
    FAILED = 0x01
    UNKNOWN_CMD = 0xFD
    FRAME_MODE_DISABLED = 0xFE
    CHECKSUM_ERROR = 0xFF


class FrameParseState(Enum):
    """帧解析状态机"""
    HEADER = 0
    CMD = 1
    LENGTH = 2
    DATA = 3
    CHECKSUM = 4
    TAIL = 5


class FrameParser:
    """LinkerEG 帧解析器"""
    
    def __init__(self):
        self.state = FrameParseState.HEADER
        self.frame_buf = array.array('B', [0] * (4 + MAX_DATA_SIZE + 2))
        self.expected_len = 0
        self.current_pos = 0
        self.cmd = 0
        self.data_len = 0
    
    def reset(self):
        """重置解析器状态"""
        self.state = FrameParseState.HEADER
        self.current_pos = 0
        self.cmd = 0
        self.data_len = 0
    
    @staticmethod
    def calculate_checksum(cmd: int, data_len: int, data: bytes) -> int:
        """
        计算校验和 (补码累加和)
        checksum = ~(cmd + len + sum(data)) + 1
        """
        total = cmd + data_len
        for b in data:
            total += b
        checksum = (~total + 1) & 0xFF
        return checksum
    
    def process_byte(self, byte: int) -> bool:
        """
        处理单个字节，返回是否接收到完整帧
        """
        byte = byte & 0xFF
        
        if self.state == FrameParseState.HEADER:
            if byte == FRAME_HEADER:
                self.frame_buf[0] = byte
                self.current_pos = 1
                self.state = FrameParseState.CMD
        
        elif self.state == FrameParseState.CMD:
            self.cmd = byte
            self.frame_buf[1] = byte
            self.current_pos = 2
            self.state = FrameParseState.LENGTH
        
        elif self.state == FrameParseState.LENGTH:
            self.data_len = byte
            self.frame_buf[2] = byte
            self.current_pos = 3
            if byte <= MAX_DATA_SIZE:
                if byte == 0:
                    self.state = FrameParseState.CHECKSUM
                else:
                    self.state = FrameParseState.DATA
            else:
                self.reset()
        
        elif self.state == FrameParseState.DATA:
            self.frame_buf[self.current_pos] = byte
            self.current_pos += 1
            if self.current_pos >= 3 + self.data_len:
                self.state = FrameParseState.CHECKSUM
        
        elif self.state == FrameParseState.CHECKSUM:
            self.frame_buf[self.current_pos] = byte
            # 验证校验和
            data = bytes(self.frame_buf[3:3 + self.data_len])
            expected_checksum = self.calculate_checksum(self.cmd, self.data_len, data)
            if byte == expected_checksum:
                self.current_pos += 1
                self.state = FrameParseState.TAIL
            else:
                self.reset()
        
        elif self.state == FrameParseState.TAIL:
            if byte == FRAME_TAIL:
                self.frame_buf[self.current_pos] = byte
                return True
            else:
                self.reset()
        
        return False
    
    def get_frame_data(self) -> Tuple[int, bytes]:
        """获取解析后的帧数据 (cmd, data)"""
        data = bytes(self.frame_buf[3:3 + self.data_len])
        return self.cmd, data


class LinkerEGSerial:
    """LinkerEG 串口通讯类"""
    
    # 控制模式
    MODE_SDK = 'sdk'           # SDK控制模式 (linkereg)
    MODE_RECEIVER = 'receiver' # 接收器控制模式 (linkereg1)
    
    def __init__(self, port: str = None, baudrate: int = 115200, password: str = '12345678', isdebug: bool = False, mode: str = 'sdk'):
        """
        初始化 LinkerEG 串口通讯
        
        Args:
            port: 串口路径，如果为 None 则自动扫描
            baudrate: 波特率 (默认 115200)
            password: sudo 密码，用于修复串口权限
            isdebug: 是否打印调试信息
            mode: 控制模式 'sdk'=SDK控制模式, 'receiver'=接收器控制模式
        """
        self.port = port
        self.baudrate = baudrate
        self.mode = mode


        print(f"[LinkerEG] 初始化 LinkerEGSerial (mode={self.mode})...", flush=True)
        self.serial_port: Optional[serial.Serial] = None
        self.parser = FrameParser()
        self.running = Event()
        self.thread: Optional[Thread] = None
        
        # 版本信息
        self.version: Optional[str] = None
        self.connected = False
        self.initialized = False
        
        # 数据存储 (左右手) - 控制数据
        self.right_hand_data: List[int] = []
        self.left_hand_data: List[int] = []
        self.right_hand_protocol: int = -1
        self.left_hand_protocol: int = -1
        
        # 回调函数 (control_data, protocol)
        self.on_right_hand_data: Optional[Callable[[List[int], int], None]] = None
        self.on_left_hand_data: Optional[Callable[[List[int], int], None]] = None
        
        # 传感器原始数据存储 (左右手) - 15个int32关节值
        self.right_hand_raw_data: List[int] = []
        self.left_hand_raw_data: List[int] = []
        
        # 原始数据回调函数 (raw_data: List[int])
        # raw_data 为15个int32值: 大拇指横摆、大拇指弯曲、大拇指指尖、食指横摆、食指弯曲、食指指尖...
        self.on_right_hand_raw_data: Optional[Callable[[List[int]], None]] = None
        self.on_left_hand_raw_data: Optional[Callable[[List[int]], None]] = None
        
        # 原始数据推送状态
        self.raw_data_enabled = False
        
        # sudo密码 (用于权限修复，从外部传入)
        self._sudo_password: str = password
        self.isdebug = isdebug
        
        # 串口扫描相关
        self.checked_ports: set = set()
        self.exclude_ports: set = set()


        # 灵巧手控制状态
        self.hand_control_mode: Optional[str] = None  # 'sdk' 或 'receiver'
    
    def _is_usb_device(self, port_name: str) -> bool:
        """
        判断是否为USB串口设备
        支持: /dev/ttyUSB*, /dev/ttyACM* 等USB转串口设备
        """
        import re
        usb_patterns = [
            r'/dev/ttyUSB\d+',
            r'/dev/ttyACM\d+',
            r'/dev/ttyXRUSB\d+',
            r'/dev/ttyOBC\d+',
        ]
        
        for pattern in usb_patterns:
            if re.match(pattern, port_name):
                return True
        
        try:
            ports = serial.tools.list_ports.comports()
            for port_info in ports:
                if port_info.device == port_name:
                    description = (port_info.description or "").lower()
                    if any(keyword in description for keyword in ['usb', 'serial', 'com']):
                        return True
                    if port_info.hwid and 'USB' in port_info.hwid.upper():
                        return True
        except:
            pass
        
        return False
    
    def scan_serial_ports(self) -> List[str]:
        """扫描所有可用的USB串口"""
        ports = serial.tools.list_ports.comports()
        available_ports = []
        
        for port in ports:
            port_device = port.device
            if not self._is_usb_device(port_device):
                continue
            if port_device in self.exclude_ports:
                if self.isdebug:
                    print(f"[LinkerEG] 跳过排除的串口: {port_device}", flush=True)
                continue
            if port_device not in self.checked_ports:
                available_ports.append(port_device)
        
        return available_ports
    
    def _quick_test_port(self, port_name: str, timeout: float = 3.0) -> Tuple[bool, Optional[int]]:
        """
        快速测试串口是否为 LinkerEG 设备
        返回: (success, error_code)
        error_code: None=成功, -1=不存在, -2=权限问题, -3=设备忙, -99=其他错误
        """
        try:
            with serial.Serial(
                port_name,
                baudrate=self.baudrate,
                timeout=0.2,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            ) as ser:
                # 清空缓冲区
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                time.sleep(0.2)
                
                # 发送 frame_enable (尝试三次，间隔更长)
                for _ in range(3):
                    ser.write(b'frame_enable\n')
                    time.sleep(0.2)
                
                # 清空可能的回复
                if ser.in_waiting > 0:
                    ser.read(ser.in_waiting)
                
                # 发送 SDK 控制命令 (0x0E) - 这个命令响应更可靠
                cmd = CmdType.SDK_CONTROL
                data = b''
                checksum = FrameParser.calculate_checksum(cmd, 0, data)
                frame = bytes([FRAME_HEADER, cmd, 0, checksum, FRAME_TAIL])
                if self.isdebug:
                    print(f"[LinkerEG] 发送SDK控制命令: {frame.hex()}", flush=True)
                ser.write(frame)
                time.sleep(0.1)
                
                # 再发送读取版本命令
                cmd = CmdType.READ_VERSION
                checksum = FrameParser.calculate_checksum(cmd, 0, data)
                frame = bytes([FRAME_HEADER, cmd, 0, checksum, FRAME_TAIL])
                if self.isdebug:
                    print(f"[LinkerEG] 发送版本查询: {frame.hex()}", flush=True)
                ser.write(frame)
                
                # 等待响应
                start_time = time.time()
                parser = FrameParser()
                
                while (time.time() - start_time) < timeout:
                    if ser.in_waiting > 0:
                        chunk = ser.read(ser.in_waiting)
                        if self.isdebug:
                            print(f"[LinkerEG] 收到数据: {chunk.hex()}", flush=True)
                        for byte in chunk:
                            if parser.process_byte(byte):
                                cmd_resp, data_resp = parser.get_frame_data()
                                if self.isdebug:
                                    print(f"[LinkerEG] 解析到帧: cmd=0x{cmd_resp:02X}, data={data_resp.hex()}", flush=True)
                                # 收到任何有效响应都说明是 LinkerEG 设备
                                if cmd_resp in (CmdType.READ_VERSION, CmdType.SDK_CONTROL, CmdType.RECEIVER_CONTROL, 
                                                CmdType.MAPPED_DATA_PUSH, CmdType.RAW_DATA_PUSH, 
                                                CmdType.ENABLE_MAPPED_DATA, CmdType.DISABLE_MAPPED_DATA):
                                    if self.isdebug:
                                        print(f"[LinkerEG] 串口 {port_name} 检测到 LinkerEG 设备 (cmd=0x{cmd_resp:02X})", flush=True)
                                    return True, None
                                parser.reset()
                    time.sleep(0.01)
                
                if self.isdebug:
                    print(f"[LinkerEG] 串口 {port_name} 无响应", flush=True)
                return False, None
                
        except serial.SerialException as e:
            error_msg = str(e)
            if "No such file or directory" in error_msg or "[Errno 2]" in error_msg:
                return False, -1
            elif "Permission denied" in error_msg or "[Errno 13]" in error_msg:
                return False, -2
            elif "Device or resource busy" in error_msg:
                return False, -3
            else:
                if self.isdebug:
                    print(f"[LinkerEG] 串口测试失败: {e}", flush=True)
                return False, -99
        except Exception as e:
            if self.isdebug:
                print(f"[LinkerEG] 串口测试异常: {e}", flush=True)
            return False, -99
    
    def find_valid_port(self) -> Tuple[Optional[str], Optional[int]]:
        """
        自动扫描并查找有效的 LinkerEG 串口
        返回: (port_name, error_code)
        """
        print("[LinkerEG] 开始扫描串口...", flush=True)
        
        available_ports = self.scan_serial_ports()
        if not available_ports:
            print("[LinkerEG] 未发现可用的 USB 串口设备", flush=True)
            return None, -1
        
        print(f"[LinkerEG] 发现 {len(available_ports)} 个串口: {available_ports}", flush=True)
        
        for port in available_ports:
            print(f"[LinkerEG] 正在检测 {port}...", flush=True)
            success, error_code = self._quick_test_port(port)
            
            if error_code == -2:  # 权限问题
                print(f"[LinkerEG] 检测到权限问题，尝试修复 {port}...", flush=True)
                if self._fix_serial_permission(port):
                    # 修复后重试
                    success, error_code = self._quick_test_port(port)
            
            self.checked_ports.add(port)
            
            if success:
                print(f"[LinkerEG] ✓ 找到有效串口: {port}", flush=True)
                return port, None
        
        print("[LinkerEG] 未找到 LinkerEG 设备", flush=True)
        return None, -1
    
    
    def _fix_serial_permission(self, port_name: str) -> bool:
        """尝试修复串口权限（使用预设密码）"""
        try:
            if not os.path.exists(port_name):
                print(f"[LinkerEG] 串口设备不存在: {port_name}", flush=True)
                return False
            
            password = self._sudo_password
            if not password:
                print("[LinkerEG] 未设置 sudo 密码，无法修复权限", flush=True)
                return False
            
            # 使用echo传递密码执行chmod
            command = f'echo "{password}" | sudo -S chmod 666 {port_name}'
            result = subprocess.run(
                command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                print(f"[LinkerEG] ✓ 成功修复 {port_name} 权限", flush=True)
                return True
            else:
                print(f"[LinkerEG] ✗ 权限修复失败: {result.stderr.strip()}", flush=True)
                return False
                
        except subprocess.TimeoutExpired:
            print(f"[LinkerEG] 修复权限超时", flush=True)
        except Exception as e:
            print(f"[LinkerEG] 修复权限时出错: {e}", flush=True)
        
        return False
    
    def open(self, auto_scan: bool = True) -> bool:
        """
        打开串口（带权限检测和自动修复）
        
        Args:
            auto_scan: 如果 port 为 None，是否自动扫描
        """
        # 如果没有指定串口，自动扫描
        if self.port is None and auto_scan:
            port, error_code = self.find_valid_port()
            if port is None:
                print("[LinkerEG] 自动扫描未找到有效串口", flush=True)
                return False
            self.port = port
        
        if self.port is None:
            print("[LinkerEG] 未指定串口", flush=True)
            return False
        
        max_retry = 2  # 最多尝试2次 (第一次失败后尝试修复权限再试一次)
        
        for attempt in range(max_retry):
            try:
                self.serial_port = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=0.01,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                self.connected = True
                print(f"[LinkerEG] 串口 {self.port} 打开成功 (波特率: {self.baudrate})", flush=True)
                return True
                
            except serial.SerialException as e:
                error_msg = str(e)
                
                if "No such file or directory" in error_msg or "[Errno 2]" in error_msg:
                    print(f"[LinkerEG] 串口设备不存在: {self.port}", flush=True)
                    print("[LinkerEG] 请检查串口名称或设备是否连接", flush=True)
                    return False
                    
                elif "Permission denied" in error_msg or "[Errno 13]" in error_msg:
                    print(f"[LinkerEG] 权限被拒绝: {self.port}", flush=True)
                    
                    if attempt == 0:  # 第一次失败，尝试修复权限
                        print("[LinkerEG] 检测到权限问题，尝试修复权限...", flush=True)
                        if self._fix_serial_permission(self.port):
                            print("[LinkerEG] 权限修复成功，重新尝试打开串口...", flush=True)
                            continue  # 重试
                        else:
                            print("[LinkerEG] 权限修复失败", flush=True)
                            return False
                    else:
                        print("[LinkerEG] 权限修复后仍无法打开串口", flush=True)
                        return False
                        
                elif "Device or resource busy" in error_msg:
                    print(f"[LinkerEG] 设备忙: {self.port} - 串口可能已被其他程序占用", flush=True)
                    return False
                    
                else:
                    print(f"[LinkerEG] 串口打开失败: {e}", flush=True)
                    return False
        
        return False
    
    def close(self):
        """关闭串口"""
        self.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connected = False
        print("[LinkerEG] 串口已关闭", flush=True)
    
    def _build_frame(self, cmd: int, data: bytes = b'') -> bytes:
        """构建发送帧"""
        data_len = len(data)
        checksum = FrameParser.calculate_checksum(cmd, data_len, data)
        frame = bytes([FRAME_HEADER, cmd, data_len]) + data + bytes([checksum, FRAME_TAIL])
        return frame
    
    def _send_frame(self, cmd: int, data: bytes = b'') -> bool:
        """发送帧"""
        if not self.serial_port or not self.serial_port.is_open:
            return False
        frame = self._build_frame(cmd, data)
        try:
            self.serial_port.write(frame)
            return True
        except Exception as e:
            print(f"[LinkerEG] 发送失败: {e}", flush=True)
            return False
    
    def enable_frame_mode(self) -> bool:
        """启用数据帧模式 (发送 'frame_enable\n' 多次)"""
        if not self.serial_port or not self.serial_port.is_open:
            return False
        try:
            # 清空缓冲区
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            # 发送 ASCII 字符串 "frame_enable\n" 多次确保可靠
            for _ in range(3):
                self.serial_port.write(b'frame_enable\n')
                time.sleep(0.1)
            print("[LinkerEG] 已发送 frame_enable 命令", flush=True)
            return True
        except Exception as e:
            print(f"[LinkerEG] 发送 frame_enable 失败: {e}", flush=True)
            return False
    
    def sdk_control(self) -> bool:
        """SDK控制灵巧手 (0x0E) - linkereg2模式"""
        return self._send_frame(CmdType.SDK_CONTROL)
    
    def receiver_control(self) -> bool:
        """接收器控制灵巧手 (0x0D) - linkereg1模式"""
        return self._send_frame(CmdType.RECEIVER_CONTROL)
    
    def enable_mapped_data(self) -> bool:
        """启用映射数据推送 (0x11)"""
        return self._send_frame(CmdType.ENABLE_MAPPED_DATA)
    
    def disable_mapped_data(self) -> bool:
        """禁用映射数据推送 (0x12)"""
        return self._send_frame(CmdType.DISABLE_MAPPED_DATA)
    
    def enable_raw_data(self) -> bool:
        """
        启用传感器原始数据推送 (0x0F)
        发送: AA 0F 00 F1 55
        响应: AA 0F 01 00 [校验和] 55
        启用后系统以50Hz频率推送原始传感器数据 (命令类型0x20)
        """
        return self._send_frame(CmdType.ENABLE_RAW_DATA)
    
    def disable_raw_data(self) -> bool:
        """
        禁用传感器原始数据推送 (0x10)
        发送: AA 10 00 F0 55
        响应: AA 10 01 00 [校验和] 55
        """
        return self._send_frame(CmdType.DISABLE_RAW_DATA)
    
    def read_control_mode(self) -> bool:
        """
        读取控制方式 (0x0B)
        发送: AA 0B 00 F5 55
        响应: AA 0B 02 00 [0x00:SDK控制, 0x01:接收器控制] [校验和] 55
        """
        return self._send_frame(CmdType.READ_CONTROL_MODE)
    
    def read_version(self) -> bool:
        """读取版本号 (0x14)"""
        return self._send_frame(CmdType.READ_VERSION)
    
    def initialize(self) -> bool:
        """
        完整初始化流程:
        - SDK控制模式: 发送 0x0E 命令
        - 接收器控制模式: 发送 0x0D 命令 (需要连接灵巧手)
        """
        mode_name = "SDK控制模式" if self.mode == self.MODE_SDK else "接收器控制模式"
        print(f"[LinkerEG] 开始初始化 ({mode_name})...", flush=True)
        
        # 1. 启用数据帧模式
        if not self.enable_frame_mode():
            print("[LinkerEG] 启用数据帧模式失败", flush=True)
            return False
        time.sleep(0.2)
        
        # 2. 根据模式发送控制命令 (多次发送确保生效)
        while 1:
            if self.mode == self.MODE_RECEIVER and self.hand_control_mode != self.MODE_RECEIVER:
                self.receiver_control()
            elif self.mode == self.MODE_SDK and self.hand_control_mode != self.MODE_SDK:
                self.sdk_control()
            else:
                print(f"[LinkerEG] 当前控制方式: {self.hand_control_mode} (与期望一致)", flush=True)
                break
            time.sleep(0.2)
            self.read_control_mode()

        


        # 3. 读取版本号
        if not self.read_version():
            print("[LinkerEG] 读取版本号失败", flush=True)
            return False
        time.sleep(0.3)
        
        # 4. 读取当前控制方式
        if not self.read_control_mode():
            print("[LinkerEG] 读取控制方式失败", flush=True)
            return False
        time.sleep(0.3)

        # 5. 启用映射数据推送
        if not self.enable_mapped_data():
            print("[LinkerEG] 启用映射数据推送失败", flush=True)
            return False
        
        self.initialized = True
        
        print(f"[LinkerEG] 初始化完成 ({self.hand_control_mode})", flush=True)
        return True
    
    def start(self):
        """启动接收线程"""
        if self.thread is not None and self.thread.is_alive():
            return
        self.running.set()
        self.thread = Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        print("[LinkerEG] 接收线程已启动", flush=True)
    
    def stop(self):
        """停止接收线程"""
        self.running.clear()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        print("[LinkerEG] 接收线程已停止", flush=True)
    
    def _reorder_joints(self, control_data: List[int], protocol: int) -> List[int]:
        """
        重排关节顺序，使输出与 haocun/linkerforce 保持一致
        
        Args:
            control_data: 原始控制数据
            protocol: 协议类型
        
        Returns:
            重排后的控制数据
        """
        if protocol == 2:  # L21 (16关节 -> 25关节输出)
            # 原序 (16关节): [大拇指横摆, 食指横摆, 中指横摆, 无名指横摆, 小指横摆,
            #                大拇指弯曲, 食指弯曲, 中指弯曲, 无名指弯曲, 小指弯曲,
            #                大拇指横滚, 大拇指中部, 大拇指指尖, 食指指尖, 中指指尖, 无名指指尖, 小指指尖]

            if len(control_data) >= 16:
                return [
                    control_data[5],   # 大拇指弯曲
                    control_data[6],   # 食指弯曲
                    control_data[7],   # 中指弯曲
                    control_data[8],   # 无名指弯曲
                    control_data[9],   # 小拇指弯曲
                    control_data[0],   # 大拇指横摆
                    control_data[1],   # 食指横摆
                    control_data[2],   # 中指横摆
                    control_data[3],   # 无名指横摆
                    control_data[4],   # 小拇指横摆
                    control_data[15],  # 大拇指横滚
                    0,                 # 预留
                    0,                 # 预留
                    0,                 # 预留
                    0,                 # 预留
                    0,  # 大拇指中部
                    0,                 # 预留
                    0,                 # 预留
                    0,                 # 预留
                    0,                 # 预留
                    control_data[10],  # 大拇指指尖
                    control_data[11],  # 食指指尖
                    control_data[12],  # 中指指尖
                    control_data[13],  # 无名指指尖
                    control_data[14],  # 小指指尖

                ]
        
        elif protocol == 3:  # L6/O6 (6关节)
            # 原序: [大拇指横摆, 大拇指弯曲, 食指, 中指, 无名指, 小指]
            # 新序: [大拇指弯曲, 大拇指横摆, 食指, 中指, 无名指, 小指]
            if len(control_data) >= 6:
                return [
                    control_data[1],  # 大拇指弯曲
                    control_data[0],  # 大拇指横摆
                    control_data[2],  # 食指弯曲
                    control_data[3],  # 中指弯曲
                    control_data[4],  # 无名指弯曲
                    control_data[5],  # 小指弯曲
                ]
        
        elif protocol == 4:  # O7 (7关节)
            # 原序: [大拇指横摆, 大拇指弯曲, 食指, 中指, 无名指, 小指, 大拇指旋转]
            # 新序: [大拇指弯曲, 大拇指横摆, 食指, 中指, 无名指, 小指, 大拇指旋转]
            if len(control_data) >= 7:
                return [
                    control_data[1],  # 大拇指弯曲
                    control_data[0],  # 大拇指横摆
                    control_data[2],  # 食指弯曲
                    control_data[3],  # 中指弯曲
                    control_data[4],  # 无名指弯曲
                    control_data[5],  # 小指弯曲
                    control_data[6],  # 大拇指旋转
                ]
        
        elif protocol == 1:  # L10 (10关节)
            # 新序: [大拇指弯曲, 大拇指横摆, 食指弯曲, 中指弯曲, 无名指弯曲, 小指弯曲, 食指横摆, 无名指横摆, 小指横摆, 大拇指旋转]
            # 新序: [大拇指弯曲, 大拇指横摆, 食指弯曲, 中指弯曲, 无名指弯曲, 小指弯曲, 食指横摆, 无名指横摆, 小指横摆, 大拇指旋转]
            if len(control_data) >= 10:
                return [
                    control_data[4],  # 大拇指弯曲
                    control_data[0],  # 大拇指横摆
                    control_data[5],  # 食指弯曲
                    control_data[6],  # 中指弯曲
                    control_data[7],  # 无名指弯曲
                    control_data[8],  # 小指弯曲
                    control_data[1],  # 食指横摆
                    control_data[2],  # 无名指横摆
                    control_data[3],  # 小指横摆
                    control_data[9],  # 大拇指旋转
                ]
        
        elif protocol == 0 or protocol == 5:  # L20 G20(16关节 -> 20关节输出)
            # 原序 (16关节): [大拇指横摆, 食指横摆, 中指横摆, 无名指横摆, 小指横摆,
            #                大拇指弯曲, 食指弯曲, 中指弯曲, 无名指弯曲, 小指弯曲,
            #                大拇指指尖, 食指指尖, 中指指尖, 无名指指尖, 小指指尖, 大拇指横滚]
            # 新序 (20关节): [拇指弯曲, 食指弯曲, 中指弯曲, 无名指弯曲, 小指弯曲,
            #                拇指横摆, 食指横摆, 中指横摆, 无名指横摆, 小指横摆,
            #                拇指横滚, 预留, 预留, 预留, 预留,
            #                拇指指尖, 食指指尖, 中指指尖, 无名指指尖, 小指指尖]
            if len(control_data) >= 16:
                return [
                    control_data[5],   # 拇指弯曲
                    control_data[6],   # 食指弯曲
                    control_data[7],   # 中指弯曲
                    control_data[8],   # 无名指弯曲
                    control_data[9],   # 小指弯曲
                    control_data[0],   # 拇指横摆
                    control_data[1],   # 食指横摆
                    control_data[2],   # 中指横摆
                    control_data[3],   # 无名指横摆
                    control_data[4],   # 小指横摆
                    control_data[15],  # 拇指横滚
                    0,                 # 预留
                    0,                 # 预留
                    0,                 # 预留
                    0,                 # 预留
                    control_data[10],  # 拇指指尖
                    control_data[11],  # 食指指尖
                    control_data[12],  # 中指指尖
                    control_data[13],  # 无名指指尖
                    control_data[14],  # 小指指尖
                ]
        
        # 其他协议直接透传
        return control_data
    
    def _receive_loop(self):
        """接收数据循环"""
        while self.running.is_set():
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    for byte in data:
                        if self.parser.process_byte(byte):
                            self._handle_frame()
                            self.parser.reset()
                time.sleep(0.001)
            except Exception as e:
                print(f"[LinkerEG] 接收错误: {e}", flush=True)
                time.sleep(0.01)
    
    def _handle_frame(self):
        """处理接收到的完整帧"""
        cmd, data = self.parser.get_frame_data()
        
        if cmd == CmdType.READ_CONTROL_MODE:
            # 响应格式: AA 0B 02 00 [0x00:SDK控制, 0x01:接收器控制] [校验和] 55
            if len(data) >= 2 and data[0] == ResultCode.SUCCESS:
                control_mode = data[1]
                self.hand_control_mode = "sdk" if control_mode == 0x00 else "receiver"  
        elif cmd == CmdType.READ_VERSION:
            # 响应格式: [结果码] [版本数据...]
            if len(data) >= 5 and data[0] == ResultCode.SUCCESS:
                # 版本号格式: 4字节小端序
                version_bytes = data[1:5]
                major = version_bytes[0]
                minor = version_bytes[1]
                patch = (version_bytes[2] << 8) | version_bytes[3]
                self.version = f"{major}.{minor}.{patch}"
                print(f"[LinkerEG] 版本号: {self.version}", flush=True)
        
        elif cmd == CmdType.MAPPED_DATA_PUSH:
            # 映射数据帧 (linkereg2 SDK控制模式): [手侧] [协议] [状态] [N字节控制数据]
            # SDK模式下只有控制数据
            if len(data) >= 3:
                hand_side = data[0]  # 0=右手, 1=左手
                protocol = data[1]   # 协议类型
                # data[2] 是状态字节，忽略
                
                if protocol in PROTOCOL_MAP:
                    num_joints = PROTOCOL_MAP[protocol]['joints']
                    # SDK模式: 只有控制数据，从第3字节开始
                    if len(data) >= 3 + num_joints:
                        control_data = list(data[3:3 + num_joints])
                        
                        # 重排关节顺序，与 haocun/linkerforce 保持一致
                        control_data = self._reorder_joints(control_data, protocol)
                        
                        if hand_side == 0:  # 右手
                            self.right_hand_data = control_data
                            self.right_hand_protocol = protocol
                            if self.on_right_hand_data:
                                self.on_right_hand_data(control_data, protocol)
                        else:  # 左手
                            self.left_hand_data = control_data
                            self.left_hand_protocol = protocol
                            if self.on_left_hand_data:
                                self.on_left_hand_data(control_data, protocol)
        
        elif cmd == CmdType.SDK_CONTROL:
            if len(data) >= 1 and data[0] == ResultCode.SUCCESS:
                print("[LinkerEG] SDK控制模式已启用", flush=True)
        
        elif cmd == CmdType.RECEIVER_CONTROL:
            if len(data) >= 1 and data[0] == ResultCode.SUCCESS:
                print("[LinkerEG] 接收器控制模式已启用", flush=True)
        
        elif cmd == CmdType.ENABLE_MAPPED_DATA:
            if len(data) >= 1 and data[0] == ResultCode.SUCCESS:
                print("[LinkerEG] 映射数据推送已启用", flush=True)
        
        elif cmd == CmdType.DISABLE_MAPPED_DATA:
            if len(data) >= 1 and data[0] == ResultCode.SUCCESS:
                print("[LinkerEG] 映射数据推送已禁用", flush=True)
        
        elif cmd == CmdType.ENABLE_RAW_DATA:
            if len(data) >= 1 and data[0] == ResultCode.SUCCESS:
                self.raw_data_enabled = True
                print("[LinkerEG] 传感器原始数据推送已启用", flush=True)
        
        elif cmd == CmdType.DISABLE_RAW_DATA:
            if len(data) >= 1 and data[0] == ResultCode.SUCCESS:
                self.raw_data_enabled = False
                print("[LinkerEG] 传感器原始数据推送已禁用", flush=True)
        
        elif cmd == CmdType.RAW_DATA_PUSH:
            # 传感器原始数据帧 (0x20): AA 20 3D [1字节手侧] [60字节传感器数据] [校验和] 55
            # 数据长度: 0x3D = 61字节 (1字节手侧 + 60字节传感器数据)
            # 传感器数据: 15个int32_t值 (每个4字节，小端序)
            # 顺序: 大拇指横摆、大拇指弯曲、大拇指指尖、食指横摆、食指弯曲、食指指尖、
            #       中指横摆、中指弯曲、中指指尖、无名指横摆、无名指弯曲、无名指指尖、
            #       小指横摆、小指弯曲、小指指尖
            if len(data) >= 61:  # 1字节手侧 + 60字节传感器数据
                hand_side = data[0]  # 0=右手, 1=左手
                sensor_bytes = data[1:61]  # 60字节传感器数据
                
                # 解析15个int32_t值 (小端序)
                raw_values = []
                for i in range(15):
                    offset = i * 4
                    value = struct.unpack('<i', bytes(sensor_bytes[offset:offset+4]))[0]
                    raw_values.append(value)
                
                if hand_side == 0:  # 右手
                    self.right_hand_raw_data = raw_values
                    if self.on_right_hand_raw_data:
                        self.on_right_hand_raw_data(raw_values)
                else:  # 左手
                    self.left_hand_raw_data = raw_values
                    if self.on_left_hand_raw_data:
                        self.on_left_hand_raw_data(raw_values)
