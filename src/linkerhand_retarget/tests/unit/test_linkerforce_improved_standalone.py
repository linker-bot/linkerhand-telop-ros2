#!/usr/bin/env python3
"""
LinkerForce 改进版验证测试
独立运行，不依赖pytest
"""
import sys
import os
import threading
import time
import struct
import pytest

# 添加路径 - 从当前文件位置向上查找
current_dir = os.path.dirname(os.path.abspath(__file__))
linkerhand_retarget_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, linkerhand_retarget_dir)

# 直接从文件导入
import importlib.util
linkerforce_improved_path = os.path.join(
    linkerhand_retarget_dir,
    "linkerhand_retarget/linkerhand/linkerforce_improved.py",
)
if not os.path.exists(linkerforce_improved_path):
    pytest.skip("linkerforce_improved.py is not part of the current package", allow_module_level=True)
spec = importlib.util.spec_from_file_location(
    "linkerforce_improved",
    linkerforce_improved_path,
)
linkerforce_improved = importlib.util.module_from_spec(spec)
spec.loader.exec_module(linkerforce_improved)

ForceSerialReader = linkerforce_improved.ForceSerialReader
SerialConfig = linkerforce_improved.SerialConfig
DeviceInfo = linkerforce_improved.DeviceInfo
FrameParser = linkerforce_improved.FrameParser
FrameParseState = linkerforce_improved.FrameParseState

# 导入常量
spec2 = importlib.util.spec_from_file_location(
    "constants",
    os.path.join(linkerhand_retarget_dir, "linkerhand_retarget/linkerhand/constants.py")
)
constants = importlib.util.module_from_spec(spec2)
spec2.loader.exec_module(constants)

HandType = constants.HandType
import numpy as np


class TestRunner:
    def __init__(self):
        self.tests_passed = 0
        self.tests_failed = 0
        self.errors = []
    
    def test(self, name, func):
        try:
            func()
            print(f"✓ {name}")
            self.tests_passed += 1
        except Exception as e:
            print(f"✗ {name}: {e}")
            self.tests_failed += 1
            self.errors.append((name, str(e)))
    
    def report(self):
        print(f"\n{'='*60}")
        print(f"测试结果: {self.tests_passed} 通过, {self.tests_failed} 失败")
        if self.errors:
            print("\n失败详情:")
            for name, error in self.errors:
                print(f"  - {name}: {error}")
        print(f"{'='*60}\n")


def test_serial_config():
    runner = TestRunner()
    
    def test_default():
        config = SerialConfig()
        assert config.baudrates == [2000000, 1000000, 921600, 460800]
        assert config.timeout == 0.001
        assert config.auto_reconnect == True
    
    def test_custom():
        config = SerialConfig(
            baudrates=[921600],
            auto_reconnect=False,
            max_reconnect_attempts=10
        )
        assert config.baudrates == [921600]
        assert config.auto_reconnect == False
        assert config.max_reconnect_attempts == 10
    
    runner.test("默认配置", test_default)
    runner.test("自定义配置", test_custom)
    return runner


def test_device_info():
    runner = TestRunner()
    
    def test_creation():
        info = DeviceInfo(
            port="/dev/ttyUSB0",
            baudrate=2000000,
            handtype="Right",
            version="1.0.0"
        )
        assert info.port == "/dev/ttyUSB0"
        assert info.baudrate == 2000000
        assert info.handtype == "Right"
        assert info.version == "1.0.0"
    
    def test_optional():
        info = DeviceInfo(port="/dev/ttyUSB0", baudrate=2000000)
        assert info.handtype is None
        assert info.version is None
    
    runner.test("设备信息创建", test_creation)
    runner.test("设备信息可选字段", test_optional)
    return runner


def test_frame_parser():
    runner = TestRunner()
    
    def test_initial():
        parser = FrameParser()
        assert parser.state == FrameParseState.HEADER
        assert parser.current_pos == 0
        assert parser.checksum == 0
    
    def test_reset():
        parser = FrameParser()
        parser.state = FrameParseState.DATA
        parser.current_pos = 10
        parser.checksum = 100
        parser.reset()
        assert parser.state == FrameParseState.HEADER
        assert parser.current_pos == 0
        assert parser.checksum == 0
    
    def test_parse_valid():
        parser = FrameParser()
        # 构造帧: 0x5D, 0x01, 0x00, checksum
        frame_data = bytes([0x5D, 0x01, 0x00, 0x5E])
        result = False
        for byte in frame_data:
            if parser.process_byte(byte):
                result = True
        assert result == True
        assert parser.frame_buf[0] == 0x5D
        assert parser.frame_buf[1] == 0x01
    
    runner.test("帧解析器初始状态", test_initial)
    runner.test("帧解析器重置", test_reset)
    runner.test("帧解析器解析有效帧", test_parse_valid)
    return runner


def test_force_reader():
    runner = TestRunner()
    
    def test_initial():
        config = SerialConfig(auto_reconnect=False)
        reader = ForceSerialReader(hand_type=HandType.right, config=config)
        assert reader.poslist == [0.0] * 21
        assert reader.forcelist == [0.0] * 5
        assert reader.handtype is None
        assert reader.version is None
        assert reader.is_connected == False
    
    def test_thread_safe():
        config = SerialConfig(auto_reconnect=False)
        reader = ForceSerialReader(hand_type=HandType.right, config=config)
        
        errors = []
        
        def write_data():
            for i in range(100):
                with reader._lock:
                    reader._poslist = [float(i)] * 21
                time.sleep(0.001)
        
        def read_data():
            for _ in range(100):
                try:
                    data = reader.poslist
                    assert len(data) == 21
                except Exception as e:
                    errors.append(e)
                time.sleep(0.001)
        
        writer = threading.Thread(target=write_data)
        readers = [threading.Thread(target=read_data) for _ in range(3)]
        
        writer.start()
        for r in readers:
            r.start()
        
        writer.join()
        for r in readers:
            r.join()
        
        assert len(errors) == 0
    
    def test_version_frame():
        config = SerialConfig(auto_reconnect=False)
        reader = ForceSerialReader(hand_type=HandType.right, config=config)
        
        # 模拟版本帧
        value = 10001
        status_code = 1
        frame_data = struct.pack('<I', value) + bytes([status_code])
        
        reader._handle_version_frame(frame_data)
        
        assert reader.version == "1.0.1"
        assert reader.handtype == "Right"
    
    def test_position_frame():
        config = SerialConfig(auto_reconnect=False)
        reader = ForceSerialReader(hand_type=HandType.right, config=config)
        
        # 模拟位置帧
        angles_deg = [30.0, 45.0, 60.0]
        frame_data = b''.join(struct.pack('<f', angle) for angle in angles_deg)
        
        reader._handle_position_frame(frame_data)
        
        poslist = reader.poslist
        assert len(poslist) == 3
        for i, angle in enumerate(angles_deg):
            expected = np.deg2rad(angle)
            assert abs(poslist[i] - expected) < 0.0001
    
    def test_force_frame():
        config = SerialConfig(auto_reconnect=False)
        reader = ForceSerialReader(hand_type=HandType.right, config=config)
        
        # 模拟力数据帧
        forces = [100, 200, 300]
        frame_data = b''.join(struct.pack('>h', force) for force in forces)
        
        reader._handle_force_frame(frame_data)
        
        realforcelist = reader.realforcelist
        assert len(realforcelist) == 3
        assert realforcelist == forces
    
    def test_pack_data():
        config = SerialConfig(auto_reconnect=False)
        reader = ForceSerialReader(hand_type=HandType.right, config=config)
        
        pack_01 = reader._pack_01_data()
        assert pack_01[0] == 0x5D
        assert pack_01[1] == 0x01
        assert pack_01[2] == 0x00
        
        pack_03 = reader._pack_03_data()
        assert pack_03[0] == 0x5D
        assert pack_03[1] == 0x03
    
    runner.test("ForceReader初始状态", test_initial)
    runner.test("ForceReader线程安全", test_thread_safe)
    runner.test("ForceReader版本帧处理", test_version_frame)
    runner.test("ForceReader位置帧处理", test_position_frame)
    runner.test("ForceReader力数据帧处理", test_force_frame)
    runner.test("ForceReader数据打包", test_pack_data)
    return runner


def test_concurrent_access():
    """并发访问压力测试"""
    runner = TestRunner()
    
    def test_concurrent():
        config = SerialConfig(auto_reconnect=False)
        reader = ForceSerialReader(hand_type=HandType.right, config=config)
        
        read_count = [0]
        write_count = [0]
        
        def writer():
            for i in range(100):
                with reader._lock:
                    reader._poslist = [float(i)] * 21
                write_count[0] += 1
                time.sleep(0.0001)
        
        def reader_thread():
            for _ in range(100):
                data = reader.poslist
                assert len(data) == 21
                read_count[0] += 1
                time.sleep(0.0001)
        
        threads = [
            threading.Thread(target=writer),
            threading.Thread(target=reader_thread),
            threading.Thread(target=reader_thread)
        ]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        assert write_count[0] == 100
        assert read_count[0] == 200
    
    runner.test("并发访问压力测试", test_concurrent)
    return runner


def main():
    print("=" * 60)
    print("LinkerForce 改进版验证测试")
    print("=" * 60)
    print()
    
    # 运行所有测试
    results = []
    
    print("【配置类测试】")
    results.append(test_serial_config())
    print()
    
    print("【设备信息测试】")
    results.append(test_device_info())
    print()
    
    print("【帧解析器测试】")
    results.append(test_frame_parser())
    print()
    
    print("【ForceReader测试】")
    results.append(test_force_reader())
    print()
    
    print("【并发访问测试】")
    results.append(test_concurrent_access())
    print()
    
    # 汇总结果
    print("=" * 60)
    print("总体测试报告")
    print("=" * 60)
    
    total_passed = sum(r.tests_passed for r in results)
    total_failed = sum(r.tests_failed for r in results)
    
    print(f"总通过: {total_passed}")
    print(f"总失败: {total_failed}")
    print(f"成功率: {total_passed / (total_passed + total_failed) * 100:.1f}%")
    
    if total_failed == 0:
        print("\n✓ 所有测试通过！")
        return 0
    else:
        print("\n✗ 存在失败的测试")
        return 1


if __name__ == '__main__':
    sys.exit(main())
