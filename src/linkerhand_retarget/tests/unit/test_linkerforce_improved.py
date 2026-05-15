"""
LinkerForce 改进版单元测试
"""
import unittest
import threading
import time
import queue
from unittest.mock import Mock, MagicMock, patch
import pytest
import serial

pytest.importorskip(
    "linkerhand.linkerforce_improved",
    reason="linkerforce_improved.py is not part of the current package",
)
from linkerhand.linkerforce_improved import ForceSerialReader, SerialConfig, DeviceInfo, FrameParser, FrameParseState
from linkerhand.constants import HandType


class TestSerialConfig(unittest.TestCase):
    """测试配置类"""
    
    def test_default_config(self):
        """测试默认配置"""
        config = SerialConfig()
        self.assertEqual(config.baudrates, [2000000, 1000000, 921600, 460800])
        self.assertEqual(config.timeout, 0.001)
        self.assertTrue(config.auto_reconnect)
        self.assertEqual(config.max_reconnect_attempts, 5)
    
    def test_custom_config(self):
        """测试自定义配置"""
        config = SerialConfig(
            baudrates=[921600],
            auto_reconnect=False,
            max_reconnect_attempts=10
        )
        self.assertEqual(config.baudrates, [921600])
        self.assertFalse(config.auto_reconnect)
        self.assertEqual(config.max_reconnect_attempts, 10)


class TestDeviceInfo(unittest.TestCase):
    """测试设备信息类"""
    
    def test_device_info_creation(self):
        """测试设备信息创建"""
        info = DeviceInfo(
            port="/dev/ttyUSB0",
            baudrate=2000000,
            handtype="Right",
            version="1.0.0"
        )
        self.assertEqual(info.port, "/dev/ttyUSB0")
        self.assertEqual(info.baudrate, 2000000)
        self.assertEqual(info.handtype, "Right")
        self.assertEqual(info.version, "1.0.0")
    
    def test_device_info_optional_fields(self):
        """测试可选字段"""
        info = DeviceInfo(port="/dev/ttyUSB0", baudrate=2000000)
        self.assertIsNone(info.handtype)
        self.assertIsNone(info.version)


class TestFrameParser(unittest.TestCase):
    """测试帧解析器"""
    
    def setUp(self):
        self.parser = FrameParser()
    
    def test_initial_state(self):
        """测试初始状态"""
        self.assertEqual(self.parser.state, FrameParseState.HEADER)
        self.assertEqual(self.parser.current_pos, 0)
        self.assertEqual(self.parser.checksum, 0)
    
    def test_reset(self):
        """测试重置"""
        self.parser.state = FrameParseState.DATA
        self.parser.current_pos = 10
        self.parser.checksum = 100
        
        self.parser.reset()
        
        self.assertEqual(self.parser.state, FrameParseState.HEADER)
        self.assertEqual(self.parser.current_pos, 0)
        self.assertEqual(self.parser.checksum, 0)
    
    def test_parse_valid_frame(self):
        """测试解析有效帧"""
        # 构造一个有效帧: 0x5D, 0x01, 0x00, checksum
        frame_data = bytes([0x5D, 0x01, 0x00, 0x5E])  # checksum = 0x5D + 0x01 + 0x00 = 0x5E
        
        result = False
        for byte in frame_data:
            if self.parser.process_byte(byte):
                result = True
        
        self.assertTrue(result)
        self.assertEqual(self.parser.frame_buf[0], 0x5D)
        self.assertEqual(self.parser.frame_buf[1], 0x01)
        self.assertEqual(self.parser.frame_buf[2], 0x00)


class TestForceSerialReader(unittest.TestCase):
    """测试主类"""
    
    def setUp(self):
        self.config = SerialConfig(
            baudrates=[2000000],
            auto_reconnect=False
        )
        self.reader = ForceSerialReader(
            hand_type=HandType.right,
            config=self.config,
            debug=False
        )
    
    def tearDown(self):
        if self.reader.is_connected:
            self.reader.stop()
    
    def test_initial_state(self):
        """测试初始状态"""
        self.assertEqual(self.reader.poslist, [0.0] * 21)
        self.assertEqual(self.reader.forcelist, [0.0] * 5)
        self.assertIsNone(self.reader.handtype)
        self.assertIsNone(self.reader.version)
        self.assertFalse(self.reader.is_connected)
    
    def test_thread_safe_access(self):
        """测试线程安全访问"""
        results = []
        errors = []
        
        def write_data():
            for i in range(100):
                with self.reader._lock:
                    self.reader._poslist = [float(i)] * 21
                time.sleep(0.001)
        
        def read_data():
            for _ in range(100):
                try:
                    data = self.reader.poslist
                    results.append(len(data))
                except Exception as e:
                    errors.append(e)
                time.sleep(0.001)
        
        writer = threading.Thread(target=write_data)
        readers = [threading.Thread(target=read_data) for _ in range(5)]
        
        writer.start()
        for r in readers:
            r.start()
        
        writer.join()
        for r in readers:
            r.join()
        
        self.assertEqual(len(errors), 0)
        self.assertEqual(len(results), 500)
        for result in results:
            self.assertEqual(result, 21)
    
    def test_callback_registration(self):
        """测试回调注册"""
        disconnect_called = []
        reconnect_called = []
        
        self.reader.set_disconnect_callback(lambda: disconnect_called.append(True))
        self.reader.set_reconnect_callback(lambda: reconnect_called.append(True))
        
        self.assertIsNotNone(self.reader._on_disconnect)
        self.assertIsNotNone(self.reader._on_reconnect)
    
    def test_context_manager(self):
        """测试上下文管理器"""
        with patch.object(self.reader, 'start') as mock_start, \
             patch.object(self.reader, 'stop') as mock_stop:
            
            with self.reader:
                mock_start.assert_called_once()
            
            mock_stop.assert_called_once()
    
    def test_pack_data(self):
        """测试数据打包"""
        pack_01 = self.reader._pack_01_data()
        self.assertEqual(pack_01[0], 0x5D)  # header
        self.assertEqual(pack_01[1], 0x01)  # cmd
        self.assertEqual(pack_01[2], 0x00)  # length
        
        pack_03 = self.reader._pack_03_data()
        self.assertEqual(pack_03[0], 0x5D)
        self.assertEqual(pack_03[1], 0x03)
        
        float_data = [1.0, 2.0, 3.0]
        pack_A7 = self.reader._pack_A7_data(float_data)
        self.assertEqual(pack_A7[0], 0x5D)
        self.assertEqual(pack_A7[1], 0xA7)
    
    def test_handle_version_frame(self):
        """测试版本帧处理"""
        # 模拟版本帧数据: value=10001, status_code=1
        import struct
        value = 10001  # version 1.0.1
        status_code = 1  # right hand
        frame_data = struct.pack('<I', value) + bytes([status_code])
        
        self.reader._handle_version_frame(frame_data)
        
        self.assertEqual(self.reader.version, "1.0.1")
        self.assertEqual(self.reader.handtype, "Right")
    
    def test_handle_position_frame(self):
        """测试位置帧处理"""
        import struct
        import numpy as np
        
        # 3个角度值 (度数)
        angles_deg = [30.0, 45.0, 60.0]
        frame_data = b''.join(struct.pack('<f', angle) for angle in angles_deg)
        
        self.reader._handle_position_frame(frame_data)
        
        poslist = self.reader.poslist
        self.assertEqual(len(poslist), 3)
        # 检查转换为弧度
        for i, angle in enumerate(angles_deg):
            expected = np.deg2rad(angle)
            self.assertAlmostEqual(poslist[i], expected, places=5)
    
    def test_handle_force_frame(self):
        """测试力数据帧处理"""
        import struct
        
        # 3个力值
        forces = [100, 200, 300]
        frame_data = b''.join(struct.pack('>h', force) for force in forces)
        
        self.reader._handle_force_frame(frame_data)
        
        realforcelist = self.reader.realforcelist
        self.assertEqual(len(realforcelist), 3)
        self.assertEqual(realforcelist, forces)
    
    def test_disconnect_handler(self):
        """测试断开连接处理"""
        disconnect_called = []
        self.reader.set_disconnect_callback(lambda: disconnect_called.append(True))
        
        self.reader._connected = True
        self.reader._handle_disconnect()
        
        self.assertFalse(self.reader._connected)
        self.assertEqual(len(disconnect_called), 1)
    
    def test_reconnect_disabled(self):
        """测试重连禁用"""
        config = SerialConfig(auto_reconnect=False)
        reader = ForceSerialReader(HandType.right, config=config)
        
        reader._device_info = DeviceInfo(port="/dev/ttyUSB0", baudrate=2000000)
        
        # 不应该重连
        reader._attempt_reconnect()
        
        self.assertFalse(reader._connected)


class TestForceSerialReaderIntegration(unittest.TestCase):
    """集成测试"""
    
    def setUp(self):
        self.config = SerialConfig(
            baudrates=[2000000],
            auto_reconnect=False
        )
    
    @patch('serial.Serial')
    def test_scan_and_connect(self, mock_serial):
        """测试扫描和连接流程"""
        # 模拟串口设备
        mock_port = Mock()
        mock_port.device = "/dev/ttyUSB0"
        mock_port.description = "USB Serial"
        mock_port.hwid = "USB VID:PID"
        
        with patch('serial.tools.list_ports.comports', return_value=[mock_port]):
            reader = ForceSerialReader(
                hand_type=HandType.right,
                config=self.config,
                debug=True
            )
            
            # 检查是否识别为USB设备
            is_usb = reader._is_usb_device("/dev/ttyUSB0")
            self.assertTrue(is_usb)
    
    def test_concurrent_read_write(self):
        """测试并发读写"""
        reader = ForceSerialReader(HandType.right, config=self.config)
        
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
                self.assertEqual(len(data), 21)
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
        
        self.assertEqual(write_count[0], 100)
        self.assertEqual(read_count[0], 200)


class TestForceSerialReaderMockSerial(unittest.TestCase):
    """模拟串口测试"""
    
    def setUp(self):
        self.config = SerialConfig(
            baudrates=[2000000],
            auto_reconnect=False
        )
    
    @patch('serial.Serial')
    def test_open_serial_success(self, mock_serial_class):
        """测试成功打开串口"""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = True
        mock_serial_class.return_value = mock_serial_instance
        
        reader = ForceSerialReader(HandType.right, config=self.config)
        result = reader.open_serial("/dev/ttyUSB0", 2000000)
        
        self.assertTrue(result)
        mock_serial_class.assert_called_once()
    
    @patch('serial.Serial')
    def test_open_serial_failure(self, mock_serial_class):
        """测试打开串口失败"""
        mock_serial_class.side_effect = serial.SerialException("Permission denied")
        
        reader = ForceSerialReader(HandType.right, config=self.config)
        result = reader.open_serial("/dev/ttyUSB0", 2000000)
        
        self.assertFalse(result)
    
    @patch('serial.Serial')
    def test_close_serial(self, mock_serial_class):
        """测试关闭串口"""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = True
        mock_serial_class.return_value = mock_serial_instance
        
        reader = ForceSerialReader(HandType.right, config=self.config)
        reader.open_serial("/dev/ttyUSB0", 2000000)
        reader.close_serial()
        
        mock_serial_instance.close.assert_called_once()


if __name__ == '__main__':
    unittest.main(verbosity=2)
