import numpy as np
import serial
import time

from serial_comm.head_package import HeadPacket
from serial_comm.tku_packet import TKUPacket


class SerialSever():
    def __init__(self):
        self.head_motor = HeadPacket()
        self.tku_packet = TKUPacket()
        self.ini_serial()

    def ini_serial(self):
        #self.head_serial = serial.Serial(port='/dev/ttyUSB1', baudrate=115200, bytesize=8, timeout=.1)
        self.walk_serial = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, bytesize=8, timeout=.1)
        self.head_serial = serial.Serial(port='/dev/ttyS1', baudrate=115200, bytesize=8, timeout=.1)
        self.imu_serial = serial.Serial(port='/dev/ttyS0', baudrate=115200, bytesize=8, timeout=.1)
        pass
        
    def tx_head_packet(self, head_info):
        self.head_serial.write(self.head_motor.torque.tobytes())
        time.sleep(0.005)
        self.head_motor.update_head_packet(head_info)
        self.head_serial.write(self.head_motor.packet.tobytes())
        time.sleep(0.005)
        #print(self.head_motor.torque.tobytes())
        #print(self.head_motor.packet.tobytes())

    def tx_imu_packet(self, imu_info):
        self.tku_packet.update_imu_packet(imu_info)
        self.imu_serial.write(self.tku_packet.imu_packet.tobytes())
        time.sleep(0.05)
    
    def tx_get_imu_packet(self):
        self.imu_serial.write(self.tku_packet.get_imu_val_packet.tobytes())
        time.sleep(0.05)
    
    def tx_generate_walk_packet(self, walk_info, walk_cmd):
        #print("walk_info.walking_mode:", walk_info.walking_mode)
        params_packet = self.tku_packet.update_params_packet(walk_info.walking_mode)
        self.walk_serial.write(params_packet.tobytes())
        time.sleep(0.05)
        self.tku_packet.update_walk_packet(walk_info, walk_cmd)
        self.walk_serial.write(self.tku_packet.walk_packet.tobytes())
        time.sleep(0.05)
        #print(params_packet.tobytes())
        #print(self.tku_packet.walk_packet.tobytes())

    def tx_change_walk_data(self, walk_info, walk_cmd):
        self.tku_packet.update_walk_packet(walk_info, walk_cmd)
        self.walk_serial.write(self.tku_packet.walk_packet.tobytes())
        time.sleep(0.05)

#   def rx_generate_walk_packet(self):
#       while True:
#           if self.walk_serial.inWaiting():
#               value = self.walk_serial.read(50)
#               print('---------------')
#               print(value)
#               time.sleep(0.05)

    def rx_imu_packet(self):
        imu_data = [0, 0, 0]
        if self.imu_serial.inWaiting():
            packet = self.imu_serial.read(28)
            if len(packet) == 28 and packet[0] == 0x53 and packet[1] == 0x54 \
                and packet[2] == 0xF7 and packet[-1] == 0x45:
               
                for i in range(3):
                    idx = i*2
                    imu_data[i] = ((packet[idx+3] << 8) | (packet[idx+4]))
                    
                    if imu_data[i] & 0x8000:
                        imu_data[i] = (~(imu_data[i] & 0x7FFF) + 1)
                        
                    imu_data[i] = imu_data[i] / 100.0
                    
                return imu_data
                
    def rx_dio_packet(self):
        dio_data = None
        if self.walk_serial.inWaiting():
            packet = self.walk_serial.read(4)
            #print('ack')
            #print(packet)
            if packet[0] == 0x53 and packet[1] == 0x55:
                dio_data = packet[2]
            return dio_data
                

    def tx_motion_packet(self, action_mode, motion_info_list, delay_list):

        for motion_info, delay in zip(motion_info_list, delay_list):
            
            motion_packet = self.tku_packet.packet_motion_packet(action_mode, motion_info)

            #print("send motion packet", len(motion_packet))
            #print(motion_packet)
            self.walk_serial.write(motion_packet.tobytes())
            time.sleep(0.05) # delay for tx packet

            if delay:
                time.sleep(delay * 0.001) # delay for motion act

#    def rx_head_packet(self):
#        #while True:
#        if self.head_serial.inWaiting():
#            value = self.head_serial.readline()
#            print(value)
#            time.sleep(0.05)


#if __name__ == '__main__':
#    from collections import namedtuple
#    bb = SerialSever()
#    params_information = namedtuple('params_information', 'x_swing, y_swing, z_swing, period_t, period_t2, sample_time, lock_range, base_default_z, x_swing_com, y_swing_shift, base_lift_z, walking_mode')
#    walk_information = namedtuple('walk_informaiton', 'x, y, z, theta, walking_cmd, sensor_mode')
#    l1 = params_information(1, 1, 1, 300, 600, 30, 0.25, 4, 5, 2.2, 1, 2)
#    l2 = walk_information(5000, 0, 0, 2, 1, 1)
#    bb.tx_generate_walk_packet(l1, l2)
#    bb.rx_generate_walk_packet()


#    imu_information = namedtuple('imu_information', 'imu_reset, gain_set, force_state, desire_set, roll, pitch, yaw')
#    i1 = imu_information(0, 0, 0, 0, 0, 0, 0)
#    print(i1)
#    bb.tx_imu_packet(i1)
#    bb.rx_imu_packet()

#
#
#    from collections import namedtuple
#
#    aa = SerialSever()
#
#    #time.sleep(5)
#    head_motor_info = namedtuple('head_motor_info', 'id, speed, position')
#    h1 = head_motor_info(0, 1024, 1024)
#    h2 = head_motor_info(1, 1024, 1024)
#    print(h1)
#    print(h2)
#
#    aa.tx_head_packet([h1, h2])
#    #aa.rx_head_packet()
#    pass

        
