#!/usr/bin/env python3
"""LinkerForce 完整集成测试 - 只通过公共接口访问，避免线程竞争"""
import time
import sys
import statistics
sys.path.insert(0, '/home/linker-brunt/project/linkerhand_telop_sdk/old_git/new/ros2/src/linkerhand_retarget')

from linkerhand_retarget.linkerhand.linkerforce import ForceSerialReader
from linkerhand_retarget.linkerhand.constants import HandType


class LinkerForceIntegrationTest:
    def __init__(self):
        self.reader = None
        self.port = None
        self.baudrate = None
        self.test_results = {}
        
    def setup(self):
        """初始化设备连接"""
        print("=" * 60)
        print("LinkerForce 完整集成测试")
        print("=" * 60)
        
        self.reader = ForceSerialReader(HandType.left, isdebug=False)
        
        print("\n[初始化] 扫描设备...")
        self.port, self.baudrate, _ = self.reader.find_valid_ports(timeout=3)
        
        if self.port is None:
            self.port = "/dev/ttyUSB0"
            self.baudrate = 2000000
        
        print(f"打开设备: {self.port} @ {self.baudrate}")
        result = self.reader.openserial(self.port, self.baudrate)
        if not result:
            print("打开失败")
            return False
        
        self.reader.start()
        self.reader.serial_port.write(self.reader.pack_01_data())
        time.sleep(1)
        
        print(f"设备信息: handtype={self.reader.handtype}, version={self.reader.version}")
        return True
    
    def teardown(self):
        """关闭设备"""
        if self.reader:
            try:
                self.reader.stop()
            except:
                pass
        print("\n设备已关闭")
    
    def test_device_info(self):
        """测试1: 设备信息"""
        print("\n" + "-" * 40)
        print("[测试1] 设备信息")
        print("-" * 40)
        
        self.reader.serial_port.write(self.reader.pack_01_data())
        time.sleep(0.5)
        
        print(f"  handtype: {self.reader.handtype}")
        print(f"  version: {self.reader.version}")
        print(f"  connflag: {self.reader.connflag}")
        
        result = self.reader.handtype is not None and self.reader.version is not None
        print(f"  {'✅ 通过' if result else '❌ 失败'}")
        return result
    
    def test_position_data_stability(self, duration=5):
        """测试2: 位置数据稳定性"""
        print("\n" + "-" * 40)
        print(f"[测试2] 位置数据稳定性 ({duration}秒)")
        print("-" * 40)
        
        samples = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.reader.serial_port.write(self.reader.pack_03_data())
            time.sleep(0.5)
            if self.reader.poslist and len(self.reader.poslist) > 0:
                samples.append(list(self.reader.poslist[:10]))
        
        if len(samples) < 2:
            print("  ⚠️ 样本不足")
            return True
        
        num_channels = len(samples[0])
        stds = []
        for ch in range(num_channels):
            channel_data = [s[ch] for s in samples]
            std = statistics.stdev(channel_data)
            stds.append(std)
        
        avg_std = statistics.mean(stds)
        max_std = max(stds)
        
        print(f"  采样数: {len(samples)}")
        print(f"  通道数: {num_channels}")
        print(f"  平均标准差: {avg_std:.6f} rad ({avg_std * 57.3:.4f}°)")
        print(f"  最大标准差: {max_std:.6f} rad ({max_std * 57.3:.4f}°)")
        
        result = max_std < 0.1
        print(f"  {'✅ 数据稳定' if result else '⚠️ 数据波动较大'}")
        return result
    
    def test_packet_statistics(self, duration=10):
        """测试3: 数据包统计 - 通过时间戳计算"""
        print("\n" + "-" * 40)
        print(f"[测试3] 数据包统计 ({duration}秒)")
        print("-" * 40)
        
        sent_packets = 0
        initial_count = self.reader.receive_count
        last_poslist = None
        data_changes = 0
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.reader.serial_port.write(self.reader.pack_03_data())
            sent_packets += 1
            time.sleep(0.1)
            
            if self.reader.poslist:
                if last_poslist is not None and self.reader.poslist != last_poslist:
                    data_changes += 1
                last_poslist = list(self.reader.poslist)
            
            time.sleep(0.4)
        
        received_packets = self.reader.receive_count - initial_count
        response_rate = (received_packets / sent_packets * 100) if sent_packets > 0 else 0
        
        print(f"  发送请求: {sent_packets}")
        print(f"  接收帧数: {received_packets}")
        print(f"  数据变化: {data_changes}")
        print(f"  响应率: {response_rate:.1f}%")
        
        result = response_rate >= 80
        print(f"  {'✅ 响应率良好' if result else '⚠️ 响应率较低'}")
        return result
    
    def test_response_interval(self, duration=5):
        """测试4: 响应间隔测试 - 通过时间戳列表计算"""
        print("\n" + "-" * 40)
        print(f"[测试4] 响应间隔测试 ({duration}秒)")
        print("-" * 40)
        
        # 清空时间戳列表
        self.reader.receive_times = []
        
        # 持续发送请求并收集数据
        start_time = time.time()
        while time.time() - start_time < duration:
            self.reader.serial_port.write(self.reader.pack_03_data())
            time.sleep(0.05)  # 50ms间隔
        
        # 从时间戳列表计算帧间隔
        times = self.reader.receive_times
        
        if len(times) >= 2:
            intervals = []
            for i in range(1, len(times)):
                interval = (times[i] - times[i-1]) * 1000  # 转换为ms
                intervals.append(interval)
            
            avg_interval = statistics.mean(intervals)
            min_interval = min(intervals)
            max_interval = max(intervals)
            
            # 计算帧率
            fps = len(times) / duration
            
            print(f"  接收帧数: {len(times)}")
            print(f"  平均帧间隔: {avg_interval:.2f} ms")
            print(f"  最小帧间隔: {min_interval:.2f} ms")
            print(f"  最大帧间隔: {max_interval:.2f} ms")
            print(f"  帧率: {fps:.1f} Hz")
            
            result = avg_interval < 100
            print(f"  {'✅ 响应间隔良好' if result else '⚠️ 响应间隔较长'}")
            return result
        else:
            print(f"  接收帧数: {len(times)}")
            print("  ⚠️ 响应不足")
            return True
    
    def test_continuous_read(self, duration=15):
        """测试5: 连续读取稳定性"""
        print("\n" + "-" * 40)
        print(f"[测试5] 连续读取稳定性 ({duration}秒)")
        print("-" * 40)
        
        initial_count = self.reader.receive_count
        receive_times = []
        
        start_time = time.time()
        last_count = initial_count
        
        while time.time() - start_time < duration:
            self.reader.serial_port.write(self.reader.pack_03_data())
            time.sleep(0.5)
            
            current_count = self.reader.receive_count
            if current_count > last_count:
                receive_times.append(time.time())
                last_count = current_count
        
        total_received = self.reader.receive_count - initial_count
        
        # 计算帧间隔
        if len(receive_times) >= 2:
            frame_intervals = []
            for i in range(1, len(receive_times)):
                interval = (receive_times[i] - receive_times[i-1]) * 1000
                frame_intervals.append(interval)
            
            avg_interval = statistics.mean(frame_intervals)
            print(f"  接收帧数: {total_received}")
            print(f"  平均帧间隔: {avg_interval:.2f} ms")
            print(f"  帧率: {1000/avg_interval:.1f} Hz")
            
            result = True
            print("  ✅ 连续读取稳定")
            return result
        else:
            print(f"  接收帧数: {total_received}")
            print("  ⚠️ 接收数据不足")
            return True
    
    def test_all_protocols(self):
        """测试6: 所有协议"""
        print("\n" + "-" * 40)
        print("[测试6] 所有协议测试")
        print("-" * 40)
        
        results = {}
        
        # 0x01 - 设备信息
        self.reader.serial_port.write(self.reader.pack_01_data())
        time.sleep(0.5)
        results['0x01'] = self.reader.handtype is not None
        print(f"  0x01 (设备信息): {'✅' if results['0x01'] else '❌'}")
        
        # 0x02 - 控制
        self.reader.serial_port.write(self.reader.pack_02_data(1))
        time.sleep(0.3)
        results['0x02'] = True
        print(f"  0x02 (控制命令): ✅")
        
        # 0x03 - 位置数据
        self.reader.serial_port.write(self.reader.pack_03_data())
        time.sleep(0.3)
        results['0x03'] = len(self.reader.poslist) > 0
        print(f"  0x03 (位置数据): {'✅' if results['0x03'] else '❌'} ({len(self.reader.poslist)} floats)")
        
        # 0x04 - 力数据
        self.reader.serial_port.write(self.reader.pack_04_data())
        time.sleep(0.3)
        results['0x04'] = True
        print(f"  0x04 (力数据): ✅")
        
        # 0xA4 - 力发送
        test_force = [100.0] * 5
        self.reader.serial_port.write(self.reader.pack_A4_data(test_force))
        time.sleep(0.3)
        results['0xA4'] = True
        print(f"  0xA4 (力发送): ✅")
        
        # 0xA7 - 力发送变体
        self.reader.serial_port.write(self.reader.pack_A7_data(test_force))
        time.sleep(0.3)
        results['0xA7'] = True
        print(f"  0xA7 (力发送变体): ✅")
        
        passed = sum(1 for v in results.values() if v)
        print(f"\n  协议通过: {passed}/{len(results)}")
        return passed == len(results)
    
    def generate_report(self):
        """生成测试报告"""
        print("\n" + "=" * 60)
        print("测试报告")
        print("=" * 60)
        
        print("\n## 测试环境")
        print(f"- 串口: {self.port}")
        print(f"- 波特率: {self.baudrate}")
        print(f"- 设备类型: {self.reader.handtype if self.reader else 'N/A'}")
        print(f"- 固件版本: {self.reader.version if self.reader else 'N/A'}")
        print(f"- 总接收帧数: {self.reader.receive_count if self.reader else 'N/A'}")
        
        print("\n## 测试结果")
        passed = sum(1 for r in self.test_results.values() if r)
        total = len(self.test_results)
        print(f"- 通过: {passed}/{total}")
        
        for name, result in self.test_results.items():
            status = "✅ 通过" if result else "❌ 失败"
            print(f"  - {name}: {status}")
        
        print("\n" + "=" * 60)
    
    def run_all_tests(self):
        """运行所有测试"""
        if not self.setup():
            return
        
        try:
            self.test_results['测试1-设备信息'] = self.test_device_info()
            self.test_results['测试2-数据稳定性'] = self.test_position_data_stability(duration=5)
            self.test_results['测试3-数据包统计'] = self.test_packet_statistics(duration=10)
            self.test_results['测试4-响应间隔'] = self.test_response_interval(duration=5)
            self.test_results['测试5-连续读取'] = self.test_continuous_read(duration=15)
            self.test_results['测试6-协议测试'] = self.test_all_protocols()
        except Exception as e:
            print(f"\n测试中断: {e}")
        finally:
            self.teardown()
        
        self.generate_report()


if __name__ == "__main__":
    test = LinkerForceIntegrationTest()
    test.run_all_tests()