#!/usr/bin/env python3
"""LinkerMCG 集成测试 - UDP 客户端"""
import time
import sys
sys.path.insert(0, '/home/linker-brunt/project/linkerhand_telop_sdk/old_git/new/ros2/src/linkerhand_retarget')

from linkerhand_retarget.linkerhand.linkermcgcore import HaoCunScoketUdp, HaoCunData


class LinkerMCGIntegrationTest:
    def __init__(self):
        self.client = None
        self.test_results = {}
        
    def setup(self, host='192.168.1.23', port=8888):
        print("=" * 60)
        print("LinkerMCG 集成测试")
        print("=" * 60)
        
        print(f"\n[初始化] 连接 UDP {host}:{port}...")
        
        self.client = HaoCunScoketUdp(host=host, port=port)
        result = self.client.udp_initial()
        
        if result:
            print(f"✅ UDP 初始化成功")
            return True
        else:
            print(f"❌ UDP 初始化失败")
            return False
    
    def teardown(self):
        if self.client:
            self.client.udp_close()
        print("\n设备已关闭")
    
    def test_connection(self):
        print("\n" + "-" * 40)
        print("[测试1] 连接状态")
        print("-" * 40)
        
        is_connect = self.client.udp_is_connect()
        print(f"  is_connected: {is_connect}")
        
        print(f"  {'✅ 通过' if is_connect else '❌ 失败'}")
        return is_connect
    
    def test_receive_data(self, duration=5):
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
        print("\n" + "-" * 40)
        print("[测试3] 数据内容")
        print("-" * 40)
        
        time.sleep(1)
        
        data = self.client.realmocapdata
        print(f"  frame_index: {data.frame_index}")
        print(f"  is_update: {data.is_update}")
        
        r_hand = data.jointangle_rHand
        l_hand = data.jointangle_lHand
        
        print(f"  右手关节数: {len(r_hand)}")
        print(f"  左手关节数: {len(l_hand)}")
        
        r_nonzero = any(v != 0.0 for v in r_hand)
        l_nonzero = any(v != 0.0 for v in l_hand)
        
        if r_nonzero:
            print(f"  右手关节示例: {[f'{v:.2f}' for v in r_hand[:5]]}")
        if l_nonzero:
            print(f"  左手关节示例: {[f'{v:.2f}' for v in l_hand[:5]]}")
        
        result = len(r_hand) == 25 and len(l_hand) == 25
        print(f"  {'✅ 通过' if result else '❌ 失败'}")
        return result
    
    def test_continuous_read(self, duration=10):
        print("\n" + "-" * 40)
        print(f"[测试4] 连续读取 ({duration}秒)")
        print("-" * 40)
        
        frames = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            time.sleep(1)
            frame = self.client.realmocapdata.frame_index
            frames.append(frame)
            print(f"  {int(time.time() - start_time)}s: frame={frame}")
        
        if len(frames) >= 2:
            frame_diff = frames[-1] - frames[0]
            avg_fps = frame_diff / (duration - 1) if duration > 1 else 0
            print(f"  平均帧率: {avg_fps:.1f} Hz")
        
        result = len(frames) > 0
        print(f"  {'✅ 通过' if result else '❌ 失败'}")
        return result
    
    def generate_report(self):
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
    
    def run_all_tests(self, host='192.168.1.23', port=8888):
        if not self.setup(host, port):
            return
        
        try:
            self.test_results['测试1-连接状态'] = self.test_connection()
            self.test_results['测试2-数据接收'] = self.test_receive_data(duration=5)
            self.test_results['测试3-数据内容'] = self.test_data_content()
            self.test_results['测试4-连续读取'] = self.test_continuous_read(duration=10)
        except Exception as e:
            print(f"\n测试中断: {e}")
        finally:
            self.teardown()
        
        self.generate_report()


if __name__ == "__main__":
    test = LinkerMCGIntegrationTest()
    test.run_all_tests(host='192.168.11.88', port=9000)