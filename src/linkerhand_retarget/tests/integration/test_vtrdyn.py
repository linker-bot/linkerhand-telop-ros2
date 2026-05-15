#!/usr/bin/env python3
"""VtrDyn 集成测试 - UDP 设备"""
import time
import sys
sys.path.insert(0, '/home/linker-brunt/project/linkerhand_telop_sdk/old_git/new/ros2/src/linkerhand_retarget')

from linkerhand_retarget.linkerhand.vtrdyncore import VtrdynSocketUdp, MocapData


class VtrDynIntegrationTest:
    def __init__(self):
        self.client = None
        self.test_results = {}
        
    def setup(self, local_port=7000, remote_ip='192.168.11.88', remote_port=7000):
        print("=" * 60)
        print("VtrDyn 集成测试")
        print("=" * 60)
        
        print(f"\n[初始化] 本地端口 {local_port}, 远程 {remote_ip}:{remote_port}...")
        
        self.client = VtrdynSocketUdp(debug=True)
        result = self.client.udp_initial(local_port)
        
        if not result:
            print(f"❌ UDP 初始化失败")
            return False
        
        print(f"✅ UDP 初始化成功")
        
        dst_addr = (remote_ip, remote_port)
        conn_result = self.client.udp_send_request_connect(dst_addr)
        
        if conn_result:
            print(f"✅ 连接成功")
        else:
            print(f"⚠️ 发送连接请求，等待数据...")
        
        return True
    
    def teardown(self):
        if self.client:
            self.client.udp_close(('192.168.11.88', 7000))
        print("\n设备已关闭")
    
    def test_connection(self):
        print("\n" + "-" * 40)
        print("[测试1] 连接状态")
        print("-" * 40)
        
        is_connect = self.client.udp_is_onnect()
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
            current_frame = self.client.mocap_data_realtime.frame_index
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
        
        data = self.client.mocap_data_realtime
        print(f"  frame_index: {data.frame_index}")
        print(f"  frequency: {data.frequency}")
        print(f"  is_update: {data.is_update}")
        
        print(f"  身体节点数: {len(data.position_body)}")
        print(f"  右手节点数: {len(data.position_rHand)}")
        print(f"  左手节点数: {len(data.position_lHand)}")
        
        body_pos = data.position_body[0] if data.position_body else [0,0,0]
        print(f"  身体位置示例: {[f'{v:.3f}' for v in body_pos]}")
        
        result = len(data.position_body) == 23
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
            frame = self.client.mocap_data_realtime.frame_index
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
    
    def run_all_tests(self, local_port=7000, remote_ip='192.168.11.88', remote_port=7000):
        if not self.setup(local_port, remote_ip, remote_port):
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
    test = VtrDynIntegrationTest()
    test.run_all_tests(local_port=7000, remote_ip='192.168.11.88', remote_port=7000)