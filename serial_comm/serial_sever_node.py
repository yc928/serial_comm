import rclpy
import numpy as np
import time

from rclpy.node import Node
from serial_comm.serial_sever import SerialSever

from std_msgs.msg import Bool
from tku_msgs.msg import HeadPackage
from tku_msgs.msg import SensorSet
from tku_msgs.msg import SensorPackage
from tku_msgs.msg import Interface
from tku_msgs.msg import Parameter

from tku_msgs.srv import WalkingGaitParameter


class SerialPacket(Node):

    def __init__(self):
        super().__init__('serial_sever_node')

        self.head_sub = self.create_subscription(
                HeadPackage,
                '/package/HeadMotor',
                self.head_callback,
                10)
        self.head_sub

        self.imu_sub = self.create_subscription(
                SensorSet,
                '/sensorset',
                self.imu_callback,
                10)
        self.imu_sub

        self.generate_sub = self.create_subscription(
                Interface,
                '/SendBodyAuto_Topic',
                self.generate_callback,
                10)
        self.generate_sub

        self.change_walk_data_sub = self.create_subscription(
                Interface,
                '/ChangeContinuousValue_Topic',
                self.change_walk_data_callback,
                10)
        self.change_walk_data_sub

        self.save_walk_params_sub = self.create_subscription(
                Parameter,
                '/web/parameter_Topic',
                self.save_params_callback,
                10)
        self.save_walk_params_sub
        
        self.imu_data_pub = self.create_publisher(SensorPackage, '/package/sensorpackage', 1)

        self.load_walk_params_srv = self.create_service(WalkingGaitParameter, '/web/LoadWalkingGaitParameter', self.load_walk_params_callback) 

        self.serial_server = SerialSever()
        
        
    def imu_pub(self, data):
        sensor_package = SensorPackage()
        for i in range(len(data)):
            sensor_package.imudata.append(data[i])
            #print(data)
        self.imu_data_pub.publish(sensor_package)
        time.sleep(0.05)
    
    def head_callback(self, head_info):
        self.serial_server.tx_head_packet(head_info)

    def imu_callback(self, imu_info):
        self.serial_server.tx_imu_packet(imu_info)

    def generate_callback(self, walk_info):
        self.serial_server.tx_generate_walk_packet(walk_info)

    def change_walk_data_callback(self, walk_info):
        self.serial_server.tx_change_walk_data(walk_info)

    def save_params_callback(self, params_info):
        self.serial_server.tku_packet.save_params(params_info)

    def load_walk_params_callback(self, mode, response):
        response.x_swing_range, response.y_swing_range, response.z_swing_range, response.period_t, response.period_t2, response.sample_time, response.osc_lockrange, response.base_default_z, response.y_swing_shift, response.x_swing_com, response.base_lift_z = load_params(mode)


def main(args=None):
    rclpy.init(args=args)

    serial_server_node = SerialPacket()

    while rclpy.ok():
        #serial_sever_node.serial_sever.rx_head_packet()
        #print('main')
        serial_server_node.serial_server.tx_get_imu_packet()
        imu_data = serial_server_node.serial_server.rx_imu_packet()
        serial_server_node.imu_pub(imu_data)
        #print("spin")
        #rclpy.spin_once(serial_server_node)
    rclpy.spin(serial_sever_node)

    serial_server_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
