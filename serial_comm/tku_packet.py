import numpy as np
import json

class TKUPacket():
    def __init__(self):
        self.imu_packet = np.array([0x53, 0x54, 0xF6, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x45], dtype=np.uint8)

        self.get_imu_val_packet = np.array([0x53, 0x54, 0xF6, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x01, 0x00, 0x45], dtype=np.uint8)

        self.walk_packet = np.array([0x53, 0x54, 0xF5, 0x01, 0x00, 0x03,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45], dtype=np.uint8)

        self.lc_packet = np.array([0x53, 0x54, 0xF5, 0x00, 0x00, 0x06, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45], dtype=np.uint8)

        self.lipm_packet = np.array([0x53, 0x54, 0xF5, 0x00, 0x00, 0x06, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45], dtype=np.uint8)
        self.walk_params = {'continuous': self.load_params('continuous_params'), 'lc_up': self.load_params('lc_up_params'), 'lc_down': self.load_params('lc_down_params')} 

    def update_imu_packet(self, imu_info):
        self.imu_packet[-3] = (imu_info.gain_set << 3) | (imu_info.force_state << 2) | (imu_info.imu_reset <<1) | (imu_info.desire_set)
        self.desire(imu_info.roll, imu_info.pitch, imu_info.yaw)
    
    def desire(self, roll, pitch, yaw):
        def get_hi_lo_byte(value):
            if value < 0:
                value = ~value + 1
                hi_byte = ((value >> 8) & 0xFF) | 0x80
                lo_byte = value & 0xFF
            else:
                hi_byte = ((value >> 8) & 0xFF)
                lo_byte = value & 0xFF

            return hi_byte, lo_byte

        self.imu_packet[3], self.imu_packet[4] = get_hi_lo_byte(roll)
        self.imu_packet[5], self.imu_packet[6] = get_hi_lo_byte(pitch)
        self.imu_packet[7], self.imu_packet[8] = get_hi_lo_byte(yaw)

    def update_walk_packet(self, walk_info, walk_cmd):
        def get_hi_lo_byte(value):
            if value < 0:
                hi_byte = (-value >> 8) & 0xFF | 0x80
                lo_byte = (-value) & 0xFF
            else:
                hi_byte = (value >> 8) & 0xFF 
                lo_byte = value & 0xFF
            return hi_byte, lo_byte

        self.walk_packet[6], self.walk_packet[7] = get_hi_lo_byte(walk_info.x)
        self.walk_packet[8], self.walk_packet[9] = get_hi_lo_byte(walk_info.y)
        self.walk_packet[10], self.walk_packet[11] = get_hi_lo_byte(walk_info.z)
        self.walk_packet[12], self.walk_packet[13] = get_hi_lo_byte(walk_info.theta)
        self.walk_packet[14] = walk_cmd
        self.walk_packet[15] = walk_info.sensor_mode

    def update_params_packet(self, walking_mode):
        def get_multiple_hi_lo_byte(value):
            if value < 0:
                hi_byte = (((int(value * -100)) >> 8) & 0xFF) | 0x80
                lo_byte = (int(value * -100)) & 0xFF
            else: 
                hi_byte = ((int(value * 100)) >> 8) & 0xFF
                lo_byte = (int(value * 100)) & 0xFF
            return hi_byte, lo_byte

        def get_hi_lo_byte(value):

            hi_byte = (value >> 8) & 0xFF
            lo_byte = value & 0xFF

            return hi_byte, lo_byte

        print("walking_mode:", walking_mode)
        if walking_mode == 1: #0x00: single, 0x01:continue, 0x02:LC
            self.lipm_packet[12], self.lipm_packet[13] = get_hi_lo_byte(self.walk_params['continuous']['period_t'])
            _, self.lipm_packet[17] = get_multiple_hi_lo_byte(self.walk_params['continuous']['osc_lockrange'])        
            self.lipm_packet[18], self.lipm_packet[19] = get_multiple_hi_lo_byte(self.walk_params['continuous']['base_default_z'])
            _, self.lipm_packet[26] = get_hi_lo_byte(self.walk_params['continuous']['walking_mode'])
            return self.lipm_packet

        elif walking_mode == 2 or walking_mode == 3:    
            walk_mode = 'lc_up'
            if walking_mode == 3:
                walk_mode = 'lc_down'

            self.lc_packet[6], self.lc_packet[7] = get_multiple_hi_lo_byte(self.walk_params[walk_mode]['x_swing_range'])
            self.lc_packet[8], self.lc_packet[9] = get_multiple_hi_lo_byte(self.walk_params[walk_mode]['y_swing_range'])
            self.lc_packet[10], self.lc_packet[11] = get_multiple_hi_lo_byte(self.walk_params[walk_mode]['z_swing_range'])

            self.lc_packet[12], self.lc_packet[13] = get_hi_lo_byte(self.walk_params[walk_mode]['period_t'])
            self.lc_packet[14], self.lc_packet[15] = get_hi_lo_byte(self.walk_params[walk_mode]['period_t2'])

            _, self.lc_packet[16] = get_hi_lo_byte(self.walk_params[walk_mode]['sample_time'])

            _, self.lc_packet[17] = get_multiple_hi_lo_byte(self.walk_params[walk_mode]['osc_lockrange'])

            self.lc_packet[18], self.lc_packet[19] = get_multiple_hi_lo_byte(self.walk_params[walk_mode]['base_default_z'])
            self.lc_packet[20], self.lc_packet[21] = get_multiple_hi_lo_byte(self.walk_params[walk_mode]['x_swing_com'])
            self.lc_packet[22], self.lc_packet[23] = get_multiple_hi_lo_byte(self.walk_params[walk_mode]['y_swing_shift'])
            self.lc_packet[24], self.lc_packet[25] = get_multiple_hi_lo_byte(self.walk_params[walk_mode]['base_lift_z'])

            _, self.lc_packet[26] = get_hi_lo_byte(self.walk_params[walk_mode]['walking_mode'])
            return self.lc_packet


    def save_params(self, params_info):
        if params_info.mode == 1:
            continuous_params = {
                        'period_t': params_info.period_t,
                        'osc_lockrange': params_info.osc_lockrange,
                        'base_default_z': params_info.base_default_z,
                        'walking_mode': params_info.mode
                    }
            self.walk_params['continuous'] = continuous_params
            with open('./src/humanoid-ros2/serial_comm/continuous_params.json', 'w') as f:
                json.dump(continuous_params, f)
        elif params_info.mode == 2:
            lc_up_params = {
                        'x_swing_range': params_info.x_swing_range,
                        'y_swing_range': params_info.y_swing_range,
                        'z_swing_range': params_info.z_swing_range,
                        'period_t': params_info.period_t,
                        'period_t2': params_info.period_t2,
                        'sample_time': params_info.sample_time,
                        'osc_lockrange': params_info.osc_lockrange,
                        'base_default_z': params_info.base_default_z,
                        'x_swing_com': params_info.x_swing_com,
                        'y_swing_shift': params_info.y_swing_shift,
                        'base_lift_z': params_info.base_lift_z,
                        'walking_mode': params_info.mode
                    }
            self.walk_params['lc_up'] = lc_up_params
            with open('./src/humanoid-ros2/serial_comm/lc_up_params.json', 'w') as f:
                json.dump(lc_up_params, f)
        elif params_info.mode == 3:
            lc_down_params = {
                        'x_swing_range': params_info.x_swing_range,
                        'y_swing_range': params_info.y_swing_range,
                        'z_swing_range': params_info.z_swing_range,
                        'period_t': params_info.period_t,
                        'period_t2': params_info.period_t2,
                        'sample_time': params_info.sample_time,
                        'osc_lockrange': params_info.osc_lockrange,
                        'base_default_z': params_info.base_default_z,
                        'x_swing_com': params_info.x_swing_com,
                        'y_swing_shift': params_info.y_swing_shift,
                        'base_lift_z': params_info.base_lift_z,
                        'walking_mode': params_info.mode
                    }
            self.walk_params['lc_down'] = lc_down_params
            with open('./src/humanoid-ros2/serial_comm/lc_down_params.json', 'w') as f:
                json.dump(lc_down_params, f)

    def load_params(self, params_name):
            with open('./src/humanoid-ros2/serial_comm/' + params_name + '.json', 'r') as f:
                params = json.load(f)

            return params


    def packet_motion_packet(self, action_mode, motion_info):

        motion_packet = [0x53, 0x54, action_mode]
        motion_packet += motion_info

        motion_packet = np.array(motion_packet,  dtype=np.uint8)

        return motion_packet
