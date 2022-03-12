import threading
import rclpy
import numpy as np
import time
import os


from rclpy.node import Node
from serial_comm.serial_sever import SerialSever
from serial_comm.action_processor import *

from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from tku_msgs.msg import HeadPackage
from tku_msgs.msg import SensorSet
from tku_msgs.msg import SensorPackage
from tku_msgs.msg import Interface
from tku_msgs.msg import Parameter
from tku_msgs.msg import Walking
from tku_msgs.msg import SaveMotion
from tku_msgs.msg import MotorAction
from tku_msgs.srv import ReadMotion
from tku_msgs.srv import ExecuteSector


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
        
        self.save_motion_sub = self.create_subscription(
                SaveMotion,
                '/package/InterfaceSaveMotion',
                self.save_motion_callback,
                10)
        self.save_motion_sub

        self.save_motion_packet_sub = self.create_subscription(
                MotorAction,
                '/package/InterfaceSend2Sector',
                self.save_motion_packet_callback,
                10)
        self.save_motion_packet_sub

        self.read_motion_srv = self.create_service(ReadMotion, '/package/InterfaceReadSaveMotion', self.read_motion_callback)
        self.excute_sector_srv = self.create_service(ExecuteSector, '/package/ExecuteSector', self.excute_sector_callback)

        self.motion_table = {"cnt": 0, "action_list": [], "motor_list": []}

        #self.continuous_back_sub = self.create_subscription(
        #        Bool, 
        #        '/walkinggait/Continuousback',
        #        self.continuousback_callback,
        #        10)
        #self.continuous_back_sub
        
        self.imu_data_pub = self.create_publisher(SensorPackage, '/package/sensorpackage', 1)
        self.dio_data_pub = self.create_publisher(UInt8, '/package/FPGAack', 1)

        self.load_walk_params_srv = self.create_service(WalkingGaitParameter, '/web/LoadWalkingGaitParameter', self.load_walk_params_callback) 

        self.serial_server = SerialSever()
        
        #self.continuousback_flag = False
        self.walk_params_filename = {1: 'continuous_params', 2: 'lc_up_params', 3: 'lc_down_params'}
        
        
    def imu_pub(self, data):
        if data:
            sensor_package = SensorPackage()
            for i in range(len(data)):
                sensor_package.imudata.append(data[i])
                #print(data)
            self.imu_data_pub.publish(sensor_package)
            time.sleep(0.05)
            
            
    def excute_sector_callback(self, request, response):

        file_name = str(request.sector) + ".ini"
        save_path = os.path.join(os.getcwd(), file_name)
        print(save_path)


        motion_info = read_action_packet(save_path)
        
        self.serial_server.tx_motion_packet(motion_info["action_mode"], motion_info["motor_packet"], motion_info["delay"])

        response.done = True

        return response

    def save_motion_packet_callback(self, save_data):

        print('save_motion_packet_callback')
        file_name = save_data.sectorname + ".ini"
        save_path = os.path.join(os.getcwd(), file_name)

        action_angle_list = [list(motor_info.motor_angle) for motor_info in save_data.action_list]
        action_speed_list = [list(motor_info.motor_speed) for motor_info in save_data.action_list]

        save_action_packet(save_path, save_data.action_mode, action_angle_list, 
                           action_speed_list, list(save_data.delay_list))
        
    def read_motion_callback(self, request, response):

        file_path = os.path.join(os.getcwd(), request.name)

        motion_info = read_motion_table(file_path)

        response.vectorcnt = motion_info["cnt"]
        response.motionstate = motion_info["state"]
        response.id = motion_info["ID"]
        response.motionlist = motion_info["motion_list"]
        response.relativedata = motion_info["relative"]
        response.absolutedata = motion_info["absolute"]

        return response

    def save_motion_callback(self, save_data):

        SAVE_MOTION = 0

        
        if save_data.saveflag:

            #save_path = os.path.expanduser('~/some/directory/file.txt')
            save_path = os.path.join(os.getcwd(), save_data.name)

            save_motion_table(save_path, self.motion_table)

            self.motion_table = {"cnt": 0, "action_list": [], "motor_list": []}


        else:

            self.motion_table['cnt'] += 1

            if save_data.motionstate == SAVE_MOTION:

                _data = {"State": save_data.motionstate, "ID": save_data.id, 
                     "data": save_data.motionlist}

                self.motion_table['action_list'].append(_data)

            else:

                _data = {"State": save_data.motionstate, "ID": save_data.id, 
                     "data": save_data.motordata}

                self.motion_table['motor_list'].append(_data)


    def dio_pub(self, data):
        if data is not None:
            dio_package = UInt8()
            dio_package.data = data
            print(dio_package.data)
            self.dio_data_pub.publish(dio_package)      

    def head_callback(self, head_info):
        self.serial_server.tx_head_packet(head_info)

    def imu_callback(self, imu_info):
        self.serial_server.tx_imu_packet(imu_info)

    def generate_callback(self, walk_info):
        self.serial_server.tx_generate_walk_packet(walk_info, 0x02)

    def change_walk_data_callback(self, walk_info):
        self.serial_server.tx_change_walk_data(walk_info, 0x04)
        
    #def continuousback_callback(self, msg):
    #    self.continuousback_flag = msg.data

    def save_params_callback(self, params_info):
        #print("in save")
        self.serial_server.tku_packet.save_params(params_info)

    def load_walk_params_callback(self, request, response):
        param = self.serial_server.tku_packet.load_params(self.walk_params_filename[request.mode])
        #print(param)
        #for k, v in param.items():
        #    print(k, v, type(v))
        if request.mode == 1:
            response.period_t = param["period_t"]
            response.base_default_z = param["base_default_z"]
            response.osc_lockrange = param["osc_lockrange"]
            return response
        elif request.mode == 2 or request.mode == 3:
            response.x_swing_range = param["x_swing_range"] 
            response.y_swing_range = param["y_swing_range"] 
            response.z_swing_range = param["z_swing_range"] 
            response.period_t = param["period_t"] 
            response.period_t2 = param["period_t2"] 
            response.sample_time = param["sample_time"] 
            response.osc_lockrange = param["osc_lockrange"] 
            response.base_default_z = param["base_default_z"] 
            response.y_swing_shift = param["y_swing_shift"] 
            response.x_swing_com = param["x_swing_com"] 
            response.base_lift_z = param["base_lift_z"]
            return response


def spin_thread(node):
    rclpy.spin(node)
    
def main(args=None):
    rclpy.init(args=args)

    serial_server_node = SerialPacket()
    
    _spin_thread = threading.Thread(target=spin_thread, args=(serial_server_node,))
    _spin_thread.start()
    while rclpy.ok():
        #serial_sever_node.serial_sever.rx_head_packet()
        #print('main')
        serial_server_node.serial_server.tx_get_imu_packet()
        imu_data = serial_server_node.serial_server.rx_imu_packet()
        serial_server_node.imu_pub(imu_data)
        
        dio_data = serial_server_node.serial_server.rx_dio_packet()
        if dio_data is not None:
            serial_server_node.dio_pub(dio_data)

        #print("spin")
        #rclpy.spin_once(serial_server_node)
    #rclpy.spin(serial_sever_node)

    serial_sever_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
