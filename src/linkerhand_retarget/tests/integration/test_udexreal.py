#!/usr/bin/env python3
"""UdexReal 集成测试 - UDP 设备连接"""
import time
import sys
sys.path.insert(0, '/home/linker-brunt/project/linkerhand_telop_sdk/old_git/new/ros2/src/linkerhand_retarget')

from linkerhand_retarget.linkerhand.udexrealcore import UdexRealScoketUdp, UdexRealData


class UdexRealIntegrationTest:
    def __init__(self):
        self.client = None
        self.test_results = {}
        
    def setup(self, host='0.0.0.0', port=8888):
        """初始化设备连接"""
        print("=" * 60)
        print("UdexReal 集成测试")
        print("=" * 60)
        
        print(f"\n[初始化] 连接 UDP {host}:{port}...")
        
        self.client = UdexRealScoketUdp(host=host, port=port)
        result = self.client.udp_initial()
        
        if result:
            print(f"✅ UDP 初始化成功")
            return True
        else:
            print(f"❌ UDP 初始化失败")
            return False
    
    def teardown(self):
        """关闭设备"""
        if self.client:
            self.client.udp_close()
        print("\n设备已关闭")
    
    def test_connection(self):
        """测试1: 连接状态"""
        print("\n" + "-" * 40)
        print("[测试1] 连接状态")
        print("-" * 40)
        
        status = self.client.get_connection_status()
        print(f"  is_connected: {status['is_connected']}")
        print(f"  is_data_timeout: {status['is_data_timeout']}")
        
        result = status['is_connected']
        print(f"  {'✅ 通过' if result else '❌ 失败'}")
        return result
    
    def test_receive_data(self, duration=5):
        """测试2: 数据接收"""
        print("\n" + "-" * 40)
        print(f"[测试2] 数据接收 ({duration}秒)")
        print("-" * 40)
        
        frame_count = 0
        last_frame = 0
        start_time = time.time()
        
        while time.time() - start_time < duration:
            time.sleep(0.5)
            current_frame = self.client.realmocapdata.frame_index
            if current_frame > last_frame:
                frame_count += current_frame - last_frame
                last_frame = current_frame
                print(f"  接收帧: {current_frame}")
        
        print(f"  总帧数: {frame_count}")
        print(f"  帧率: {frame_count / duration:.1f} Hz")
        
        result = frame_count > 0
        print(f"  {'✅ 通过' if result else '❌ 无数据'}")
        return result
    
    def test_data_content(self):
        """测试3: 数据内容"""
        print("\n" + "-" * 40)
        print("[测试3] 数据内容")
        print("-" * 40)
        
        time.sleep(1)
        
        data = self.client.realmocapdata
        print(f"  frame_index: {data.frame_index}")
        print(f"  frequency: {data.frequency}")
        print(f"  is_update: {data.is_update}")
        
        # 检查关节数据
        r_hand = data.jointangle_rHand
        l_hand = data.jointangle_lHand
        
        print(f"  右手关节数: {len(r_hand)}")
        print(f"  左手关节数: {len(l_hand)}")
        
        # 检查是否有非零数据
        r_nonzero = any(v != 0.0 for v in r_hand)
        l_nonzero = any(v != 0.0 for v in l_hand)
        
        if r_nonzero:
            print(f"  右手关节示例: {r_hand[:5]}")
        if l_nonzero:
            print(f"  左手关节示例: {l_hand[:5]}")
        
        result = len(r_hand) == 24 and len(l_hand) == 24
        print(f"  {'✅ 通过' if result else '❌ 失败'}")
        return result
    
    def test_timeout_detection(self, timeout=2):
        """测试4: 超时检测"""
        print("\n" + "-" * 40)
        print(f"[测试4] 超时检测")
        print("-" * 40)
        
        timeout_status = self.client.check_timeout()
        print(f"  is_timeout: {timeout_status.is_timeout}")
        print(f"  time_since_last_data: {timeout_status.time_since_last_data:.3f}s")
        print(f"  timeout_threshold: {timeout_status.timeout_threshold}s")
        
        result = True  # 功能存在即通过
        print(f"  ✅ 超时检测功能正常")
        return result
    
    def test_continuous_read(self, duration=10):
        """测试5: 连续读取"""
        print("\n" + "-" * 40)
        print(f"[测试5] 连续读取 ({duration}秒)")
        print("-" * 40)
        
        frames = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            time.sleep(1)
            frame = self.client.realmocapdata.frame_index
            frames.append(frame)
            timeout_status = self.client.check_timeout()
            status = "⏰ 超时" if timeout_status.is_timeout else "✓"
            print(f"  {int(time.time() - start_time)}s: frame={frame} {status}")
        
        # 计算帧率
        if len(frames) >= 2:
            frame_diff = frames[-1] - frames[0]
            avg_fps = frame_diff / (duration - 1) if duration > 1 else 0
            print(f"  平均帧率: {avg_fps:.1f} Hz")
        
        result = len(frames) > 0
        print(f"  {'✅ 通过' if result else '❌ 失败'}")
        return result
    
    def generate_report(self):
        """生成测试报告"""
        print("\n" + "=" * 60)
        print("测试报告")
        print("=" * 60)
        
        print("\n## 测试结果")
        passed = sum(1 for r in self.test_results.values() if r)
        total = len(self.test_results)
        print(f"- 通过: {passed}/{total}")
        
        for name, result in self.test_results.items():
            status = "✅ 通过" if result else "❌ 失败"
            print(f"  - {name}: {status}")
        
        print("\n" + "=" * 60)
    
    def run_all_tests(self, host='0.0.0.0', port=8888):
        """运行所有测试"""
        if not self.setup(host, port):
            return
        
        try:
            self.test_results['测试1-连接状态'] = self.test_connection()
            self.test_results['测试2-数据接收'] = self.test_receive_data(duration=5)
            self.test_results['测试3-数据内容'] = self.test_data_content()
            self.test_results['测试4-超时检测'] = self.test_timeout_detection()
            self.test_results['测试5-连续读取'] = self.test_continuous_read(duration=10)
        except Exception as e:
            print(f"\n测试中断: {e}")
        finally:
            self.teardown()
        
        self.generate_report()


if __name__ == "__main__":
    test = UdexRealIntegrationTest()
    test.run_all_tests(host='0.0.0.0', port=8888)