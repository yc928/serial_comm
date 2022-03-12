import re

def save_action_packet(save_path, action_mode, action_angle_list, action_speed_list, action_delay_list):
    
    def get_lo_hi_byte(value):
        if value < 0:
            value = ~value + 1
            hi_byte = ((value >> 8) & 0xFF) | 0x80
            lo_byte = value & 0xFF
        else:
            hi_byte = ((value >> 8) & 0xFF)
            lo_byte = value & 0xFF

        return lo_byte, hi_byte

    def motor_packet_format(motor_angles, motor_speed):

        motor_packet = []

        for angle, speed in zip(motor_angles, motor_speed):

            motor_packet += list(get_lo_hi_byte(speed))
            motor_packet += list(get_lo_hi_byte(angle))

        _save_format = list(map(str, motor_packet))
        _save_format = " |  ".join(_save_format)

        _save_format = "Motor_Packet = " + _save_format

        return _save_format


    save_data = []
    save_data.append("Mode = {}".format(action_mode))

    for  motor_angles, motor_speed, delay in \
            zip(action_angle_list, action_speed_list, action_delay_list):

        motor_packet = motor_packet_format(motor_angles, motor_speed)

        save_data.append(motor_packet)
        save_data.append("Delay = {}".format(delay))

    with open(save_path, 'w') as _file:
        _file.write('\n'.join(save_data))


def read_action_packet(save_path):

    motion_info = {"action_mode": 0, "motor_packet": [], "delay": []}

    with open(save_path, 'r') as _file:

        read_info = iter(_file.readlines())

        motion_cnt = next(read_info)

        motion_info['action_mode'] = int(re.findall(r"Mode = (\d+)", motion_cnt)[0])
        
        while True:
            try:
                # motor_packet
                motor_info = next(read_info)
                motor_info = re.findall(r" (\d+)", motor_info)
                motor_info = list(map(int, motor_info))
                motion_info["motor_packet"].append(motor_info)

                # delay
                _delay = next(read_info)
                _delay = int(re.findall(r"Delay = (\d+)", _delay)[0])
                motion_info["delay"].append(_delay)

            except StopIteration:
                break

    return motion_info


def save_motion_table(save_path, motion_table):

    def action_format(action_list):

        motion_action = "A{} = {}"
        motion_delay = "D{} = {}"

        _save_array = [motion_delay.format(int(idx / 2)+1, _data) if idx%2  else
                       motion_action.format(int(idx / 2)+1, _data) 
                               for idx, _data in enumerate(action_list)]

        _save_format = " | ".join(_save_array)

        return _save_format


    def motor_format(motor_list):

        motor_info = "M{} = {}"

        _save_array = [motor_info.format(int(idx+1), _data)
                           for idx, _data in enumerate(motor_list)]

        _save_format = " | ".join(_save_array)

        return _save_format


    str_cnt = "VectorCnt = {}".format(motion_table['cnt'])

    save_data = [str_cnt]

    for action_data in motion_table['action_list']:

        save_data.append("State = {}".format(action_data['State']))
        save_data.append("ID = {}".format(action_data['ID']))
        save_data.append(action_format(action_data['data']))

    for motor_data in motion_table['motor_list']:

        save_data.append("State = {}".format(motor_data['State']))
        save_data.append("ID = {}".format(motor_data['ID']))
        save_data.append(motor_format(motor_data['data']))


    with open(save_path, 'w') as _file:
        _file.write('\n'.join(save_data))


def read_motion_table(save_path):

    motion_info = {"cnt": 0, "state": [], "ID": [], "motion_list": [], "relative": [], "absolute": []}

    state_array = {0: motion_info["motion_list"], 1: motion_info["relative"], 2: motion_info["relative"], 3: motion_info["absolute"], 4: motion_info["absolute"] }

    with open(save_path, 'r') as _file:

        read_info = iter(_file.readlines())

        motion_cnt = next(read_info)

        motion_info['cnt'] = int(re.findall(r"VectorCnt = (\d+)", motion_cnt)[0])
        
        while True:
            try:
                # state
                state = next(read_info)
                state = int(re.findall(r"State = (\d+)", state)[0])
                motion_info["state"].append(state)

                # id
                _id = next(read_info)
                _id = int(re.findall(r"ID = (\d+)", _id)[0])
                motion_info["ID"].append(_id)

                # motion or motor info
                data_info = next(read_info)
                data_info = re.findall(r" = (\d+)", data_info)
                data_info = list(map(int, data_info))

                _data_array = state_array[state]
                _data_array += data_info

            except StopIteration:
                break

    return motion_info


