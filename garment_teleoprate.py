#!/usr/bin/env python
# -*- coding:utf-8 -*-

# For IMU
import serial
import struct
import platform
# import serial.tools.list_ports
import math

# 查找 ttyUSB* 设备
# def find_ttyUSB():
#     print('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
#     posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
#     print('当前电脑所连接的 {} 串口设备共 {} 个: {}'.format('USB', len(posts), posts))

# For ROS
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

# 校验
def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


# 16 进制转 ieee 浮点数
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


# 处理串口数据
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag, imu_msg
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return 
    if key < 11:  # 根据数据长度位的判断, 来获取对应长度数据
        return 
    else:
        data_buff = list(buff.values())  # 获取字典所有 value

        if buff[1] == 0x51 and pub_flag[0]:
            if checkSum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
            else:
                print('0x51 校验失败')
            pub_flag[0] = False

        elif buff[1] == 0x52 and pub_flag[1]:
            if checkSum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]

            else:
                print('0x52 校验失败')
            pub_flag[1] = False

        elif buff[1] == 0x53 and pub_flag[2]:
            if checkSum(data_buff[0:10], data_buff[10]):
                angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
            else:
                print('0x53 校验失败')
            pub_flag[2] = False

        else:
            buff = {}
            key = 0

        buff = {}
        key = 0
        if pub_flag[0] == True or pub_flag[1] == True or pub_flag[2] == True:
            return 
        pub_flag[0] = pub_flag[1] = pub_flag[2] = True
        
        q = quaternion_from_euler(angle_degree[0] * math.pi / 180, angle_degree[1] * math.pi / 180, angle_degree[2] * math.pi / 180)
        
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation_covariance[0] = -1
        imu_msg.angular_velocity.x = angularVelocity[0]
        imu_msg.angular_velocity.y = angularVelocity[1]
        imu_msg.angular_velocity.z = angularVelocity[2]
        imu_msg.linear_acceleration.x = acceleration[0]
        imu_msg.linear_acceleration.y = acceleration[1]
        imu_msg.linear_acceleration.z = acceleration[2]
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        return 

# IMU Publisher Node
class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        ## Serial Initialize
        port = "/dev/ttyUSB0"
        baudrate = 921600
        try:
            self.hf_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
            if self.hf_imu.isOpen():
                print("\033[32m Serial Port Open Scucess...\033[0m")
            else:
                self.hf_imu.open()
                print("\033[32m Serial Port Open Scucess...\033[0m")
        except Exception as e:
            print(e)
            print("\033[31m Serial Port Open Failed\033[0m")
            exit(0)
        
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            buff_count = self.hf_imu.inWaiting()
        except Exception as e:
            print("exception:" + str(e))
            print("imu 失去连接，接触不良，或断线")
            exit(0)
        if buff_count > 0:
            buff_data = self.hf_imu.read(buff_count)
            for i in range(0, buff_count):
                handleSerialData(buff_data[i])
        
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(imu_msg)


key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
pub_flag = [True, True, True]
imu_msg = Imu()


if __name__ == "__main__":
    python_version = platform.python_version()[0]
    
    rclpy.init()
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown() 
