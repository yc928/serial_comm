import numpy as np
import serial
import time

from serial_sever.head_package import HeadPacket
from serial_sever.tku_packet import TKUPacket


class SerialSever():
    def __init__(self):
        self.head_motor = HeadPacket()
        self.tku_packet = TKUPacket()
        self.ini_serial()

    def ini_serial(self):
        #self.head_serial = serial.Serial(port='/dev/ttyUSB1', baudrate=115200, bytesize=8, timeout=.1)
        self.walk_serial = None
        self.head_serial = None
        self.imu_serial = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, bytesize=8, timeout=.1)

    def tx_head_packet(self, head_info):
        self.head_serial.write(self.head_motor.torque.tobytes())
        time.sleep(0.05)
        self.head_motor.update_head_packet(head_info)
        self.head_serial.write(self.head_motor.packet.tobytes())
        time.sleep(0.05)
        print(self.head_motor.torque.tobytes())
        print(self.head_motor.packet.tobytes())

    def tx_imu_packet(self, imu_info):
        self.tku_packet.update_imu_packet(imu_info)
        self.imu_serial.write(self.tku_packet.imu_packet.tobytes())
        time.sleep(0.05)
    
    def tx_get_imu_packet(self):
        self.imu_serial.write(self.tku_packet.get_imu_val_packet.tobytes())
        time.sleep(0.05)
    
    def tx_generate_walk_packet(self, walk_info):
        params_packet = self.tku_packet.update_params_packet(walk_info.walking_mode)
        self.walk_serial.write(params_packet.tobytes())
        time.sleep(0.05)
        self.tku_packet.update_walk_packet(walk_info)
        self.walk_serial.write(self.tku_packet.walk_packet.tobytes())
        time.sleep(0.05)
        print(params_packet.tobytes())
        print(self.tku_packet.walk_packet.tobytes())

    def tx_change_walk_data(self, walk_info):
        self.tku_packet.update_walk_packet(walk_info)
        self.walk_serial.write(self.tku_packet.walk_packet.tobytes())
        time.sleep(0.05)

#   def rx_generate_walk_packet(self):
#       while True:
#           if self.walk_serial.inWaiting():
#               value = self.walk_serial.read(50)
#               print('---------------')
#               print(value)
#               time.sleep(0.05)

#    def rx_imu_packet(self):
#        #data_ready = False
#        #packet, tmp_packet = [], []
#        #if self.imu_serial.inWaiting():
#        #    tmp_packet = self.imu_serial.read(14)
#        #    for i in range(len(tmp_packet)):
#        #        if tmp_packet[i] == 0x53 && tmp_packet[i+1] == 0x54 && tmp_packet[i+2] == 0xF7:
#        #            packet_start = i
#        #            data_ready = True
#        #    if data_ready:
#        #        for i in range(14):
#        #            packet.append(tmp_packet[packet_start + i])
#        #while True:
#        if self.imu_serial.inWaiting():
#            value = self.imu_serial.read(14)
#            print('-----')
#            print(value)
#            time.sleep(0.05)


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

        
