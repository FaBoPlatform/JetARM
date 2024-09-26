import threading
from dynamixel_sdk import *  # Dynamixel SDKのインポート
import serial

# 定数
ADDR_XL_TORQUE_ENABLE = 64
ADDR_XL_PRESENT_LOAD = 126  # Replace with the correct address for load
ADDR_XL_PRESENT_SPEED = 128  # Replace with the correct address for speed
ADDR_XL_PRESENT_POSITION = 132  # Replace with the correct address for position
TORQUE_ENABLE = 1
COMM_SUCCESS = 0

port_lock = threading.Lock()

class DynamixelController:
    def __init__(self, device_name, baudrate=1000000):
        self.baudrate = baudrate
        self.device_name = device_name
        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(2.0)  # Ensure the correct protocol version is passed

    def setup_port(self):
        with port_lock:
            try:
                if not self.portHandler.openPort():
                    print(f"--------------------")
                    print("接続失敗")
                    print(f"--------------------")
                    print(f"Device名: {self.device_name}")
                    print(f"Baudrate: {self.baudrate}")
                    print(f"--------------------")
                    return False
                if not self.portHandler.setBaudRate(self.baudrate):
                    print(f"--------------------")
                    print("ポート設定エラー")
                    print(f"--------------------")
                    print(f"Device名: {self.device_name}")
                    print(f"Baudrate: {self.baudrate}")
                    print(f"--------------------")
                    return False
            except (FileNotFoundError, serial.SerialException) as e:
                print(f"--------------------")
                print("接続失敗")
                print(f"--------------------")
                print(f"Device名: {self.device_name}")
                print(f"Baudrate: {self.baudrate}")
                print(f"エラー内容: {e}")
                print(f"--------------------")
                return False

            print(f"--------------------")
            print("接続成功")
            print(f"--------------------")
            print(f"Device名: {self.device_name}")
            print(f"Baudrate: {self.baudrate}")
            print(f"--------------------")
            return True

    def enable_torque(self, ids):
        with port_lock:
            for dxl_id in ids:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE)
                
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"ID {dxl_id} のトルクを有効にすることに失敗しました: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    print(f"ID {dxl_id} のトルク有効化中にエラーが発生しました: {self.packetHandler.getRxPacketError(dxl_error)}")
                else:
                    print(f"ID: {dxl_id} のトルクオプションを有効にしました。")

    def sync_write_goal_position(self, ids, goal_positions, address, data_length=4):
        with port_lock:
            group_sync_write = GroupSyncWrite(self.portHandler, self.packetHandler, address, data_length)
            for i, dxl_id in enumerate(ids):
                goal_position = goal_positions[i]
                data = [
                    DXL_LOBYTE(DXL_LOWORD(goal_position)),
                    DXL_HIBYTE(DXL_LOWORD(goal_position)),
                    DXL_LOBYTE(DXL_HIWORD(goal_position)),
                    DXL_HIBYTE(DXL_HIWORD(goal_position))
                ]
                add_param_result = group_sync_write.addParam(dxl_id, data)
                if not add_param_result:
                    print(f"ID {dxl_id} のパラメータ追加に失敗しました。")
                    return False
            
            # サーボにパケットを送信
            dxl_comm_result = group_sync_write.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(f"ゴールポジションの同期書き込みに失敗しました: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return False
            else:
                print("全てのサーボに対してゴールポジションの書き込みが成功しました。")
            group_sync_write.clearParam()
            return True

    def sync_read_data(self, ids):
        with port_lock:
            group_sync_read = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_XL_PRESENT_LOAD, 10)
            
            for dxl_id in ids:
                if not group_sync_read.addParam(dxl_id):
                    print(f"ID {dxl_id} のパラメータ読み込み追加に失敗しました。")
                    return False

            # Read the data
            dxl_comm_result = group_sync_read.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(f"データの同期読み込みに失敗しました: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return False

            results = {}
            for dxl_id in ids:
                load = group_sync_read.getData(dxl_id, ADDR_XL_PRESENT_LOAD, 2)
                speed = group_sync_read.getData(dxl_id, ADDR_XL_PRESENT_SPEED, 4)
                position = group_sync_read.getData(dxl_id, ADDR_XL_PRESENT_POSITION, 4)
                results[dxl_id] = {'load': load, 'speed': speed, 'position': position}
                print(f"ID: {dxl_id}, Load: {load}, Speed: {speed}, Position: {position}")

            group_sync_read.clearParam()
            return results
