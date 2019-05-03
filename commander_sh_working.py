from __future__ import print_function
from time import sleep
from pymavlink import mavutil
import threading
from datetime import datetime
import math
import serial
from socket import *


mission_cnt = 0


class CommandThread(threading.Thread):
    def __init__(self, drone, command):
        threading.Thread.__init__(self)
        self.drone = drone
        self.command = command

    def run(self):
        if self.command == 'arming':
            arming(self.drone)
        elif self.command == 'disarming':
            disarming(self.drone)
        elif self.command == 'takeoff':
            takeoff(self.drone)
        elif self.command == 'land':
            land(self.drone)
        elif self.command == 'start':
            start(self.drone)
        elif self.command == 'next':
            next_mission(self.drone)
        elif self.command == 'continue':
            continue_mission(self.drone)
        elif self.command == 'pause':
            pause_mission(self.drone)
        elif self.command == 'upload':
            upload_mission(self.drone)
        elif self.command == 'test':
            test_function(self.drone)
        elif self.command == 'test2':
            test_function2(self.drone)
        elif self.command == 'thread':
            test_thread(self.drone)
        elif self.command == 'kill':
            kill(self.drone)
        elif self.command == 'reboot':
            reboot(self.drone)
        elif self.command == 'gps_status':
            gps_status(self.drone)
        else:
            print('arming/ disarming/ takeoff/ land/ start/ next/ continue/ pause/ upload/ kill')


def commander(drones_set):
    while True:
        command = raw_input('commander? (upload, start, pause, continue, next, land, kill): ')
        command_to_drone(command, drones_set)


def command_to_drone(command, drones_set):
    try:
        split_command = command.split()
        target_command = split_command[0]
        drone_id_command = split_command[1]
        print(target_command)
        print(drone_id_command)
    except:
        print('You must need to input two argument!! command / drone_id(all or sys_id)')
        return
    if target_command == 'upload':
        for drone in drones_set:
            command_thread = CommandThread(drone, target_command)
            command_thread.start()
            command_thread.join()
    elif drone_id_command == 'all':
        command_thread_list = list()
        for drone in drones_set:
            command_thread = CommandThread(drone, target_command)
            command_thread_list.append(command_thread)
        for command_thread in command_thread_list:
            command_thread.start()
            sleep(0.001)
        for command_thread in command_thread_list:
            command_thread.join()
        print('Start ' + target_command)
    else:
        if target_command == 'thread':
            for drone in drones_set:
                print('target : ' + str(drone[0].target_system))
                if drone_id_command == str(drone[0].target_system):
                    command_thread = CommandThread(drone, target_command)
                    command_thread.start()
                    sleep(0.001)
        else:
            for drone in drones_set:
                print('target : ' + str(drone[0].target_system))
                if drone_id_command == str(drone[0].target_system):
                    command_thread = CommandThread(drone, target_command)
                    command_thread.start()
                    sleep(0.001)
                    command_thread.join()


def test_function(drone):
    this_drone = drone[0]
    print(this_drone)
    hawk_drone = var_drones_set[0][0]
    pigeon_home_msg = None
    pigeon_home_lat = None
    pigeon_home_lon = None
    while pigeon_home_msg is None:
        this_drone.mav.command_long_send(
            0,  # target_system
            0,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,  # command
            0,  # confirmation
            0,  # param1 (0 to indicate disarm)
            0,  # param2 (all other params meaningless)
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0)  # param7
        sleep(0.01)
        try:
            pigeon_home_msg = this_drone.recv_match(type='HOME_POSITION', blocking=True, timeout=0.01)
            #print(pigeon_home_msg)
            pigeon_home_lat = pigeon_home_msg.latitude
            pigeon_home_lon = pigeon_home_msg.longitude
        except:
            #print("doesn't receive message")
            pass
        sleep(0.01)
    # print("home lat : " + str(pigeon_home_lat))
    # print("home lon : " + str(pigeon_home_lon))


    while True:
        pigeon_now_position_msg = None
        pigeon_now_lat = None
        pigeon_now_lon = None
        while pigeon_now_position_msg is None:
            try:
                pigeon_now_position_msg = this_drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.01)
                pigeon_now_lat=pigeon_now_position_msg.lat
                pigeon_now_lon=pigeon_now_position_msg.lon
            except:
                #print("doesn't receive message")
                pass
            sleep(0.001)
        # print("pigeon lat: " + str(pigeon_now_lat))
        # print("pigeon lon: " + str(pigeon_now_lon))
        hawk_now_position_msg = None
        hawk_now_lat = None
        hawk_now_lon = None
        while hawk_now_position_msg is None:
            try:
                hawk_now_position_msg = hawk_drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.01)
                hawk_now_lat=hawk_now_position_msg.lat
                hawk_now_lon=hawk_now_position_msg.lon
            except:
                #print("doesn't receive message")
                pass
            sleep(0.001)    
        # print("hawk lat: " + str(hawk_now_lat))
        # print("hawk lon: " + str(hawk_now_lon))    
        #dist = 10
        dist = math.sqrt(
                pow(pigeon_now_lat - hawk_now_lat, 2) +
                pow(pigeon_now_lon - hawk_now_lon, 2)
                ) / 100
        # relative_vector = [
        #     (pigeon_now_lat - hawk_now_lat) / dist,
        #     (pigeon_now_lon - hawk_now_lon) / dist
        # ]
        #print("dist : " + str(dist))
        if (dist < 10 ):
            if(pigeon_now_lon  < hawk_now_lon):
                moving_distance = -7 * 100
            else:
                moving_distance =  7 * 100
            for iter_range in range(1, 5):
                this_drone.mav.command_long_send(
                    this_drone.target_system,  # target_system
                    this_drone.target_component,
                    mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # command
                    0,  # confirmation
                    1,  # param1 (0 to indicate disarm)
                    1,  # param2 (all other params meaningless)
                    0,  # param3
                    0,  # param4
                    pigeon_home_lat,  # param5
                    pigeon_home_lon + moving_distance,  # param6
                    float('nan')
                    ) 
                sleep(0.001)
        else:
            for iter_range in range(1, 5):
                this_drone.mav.command_long_send(
                    this_drone.target_system,  # target_system
                    this_drone.target_component,
                    mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # command
                    0,  # confirmation
                    1,  # param1 (0 to indicate disarm)
                    1,  # param2 (all other params meaningless)
                    0,  # param3
                    0,  # param4
                    pigeon_home_lat,  # param5
                    pigeon_home_lon,  # param6
                    float('nan'))  # param7
                sleep(0.001)
                

def test_function2(drone):
#this functions is original of reposition
    this_drone = drone[0]
    position_pigeon = None
    this_drone_lat = None
    this_drone_lon = None
    while position_pigeon is None:
        try:
            position_pigeon = this_drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.01)
            this_drone_lat = position_pigeon.lat
            this_drone_lon = position_pigeon.lon
        except:
            print("doesn't receive message")
            pass
        sleep(0.001)
    
    this_drone_lat = position_pigeon.lat
    this_drone_lon = position_pigeon.lon
    
    for iter_range in range(1, 5):
        this_drone.mav.command_long_send(
            this_drone.target_system,  # target_system
            this_drone.target_component,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # command
            0,  # confirmation
            1,  # param1 (0 to indicate disarm)
            1,  # param2 (all other params meaningless)
            0,  # param3
            0,  # param4
            position_pigeon.lat - 400,  # param5
            position_pigeon.lon ,  # param6
            float('nan'))  # param7
        sleep(0.001)



def test_thread(drone):
    while True:
        command_thread = CommandThread(drone, 'test')
        command_thread.start()
        command_thread.join()


def arming(drone):
    this_drone = drone[0]
    this_sequence = drone[1]
    sleep_time = float(this_sequence) * 5
    sleep(sleep_time)
    for iter_range in range(1, 5):
        this_drone.mav.command_long_send(
            this_drone.target_system,  # target_system
            this_drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
            0,                                             # confirmation
            1,                                             # param1 (0 to indicate disarm)
            0,                                             # param2 (all other params meaningless)
            0,                                             # param3
            0,                                             # param4
            0,                                             # param5
            0,                                             # param6
            0)                                             # param7
        sleep(0.001)


def disarming(drone):
    this_drone = drone[0]
    for iter_range in range(1, 5):
        this_drone.mav.command_long_send(
            this_drone.target_system,  # target_system
            this_drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
            0,                                             # confirmation
            0,                                             # param1 (0 to indicate disarm)
            0,                                             # param2 (all other params meaningless)
            0,                                             # param3
            0,                                             # param4
            0,                                             # param5
            0,                                             # param6
            0)                                             # param7
        sleep(0.001)

def takeoff(drone):
    this_drone = drone[0]
    for iter_range in range(1, 5):
        this_drone.flightmode = "TAKEOFF"
        sleep(0.001)
        this_drone.mav.command_long_send(
            this_drone.target_system,  # target_system
            this_drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
            0,                                   # confirmation
            -1,                                  # param1
            0,                                   # param2 (all other params meaningless)
            0,                                   # param3
            0,                                   # param4
            float('nan'), # param5
            float('nan'), # param6
            5)   # param7
        sleep(0.001)
        this_drone.mav.command_long_send(
            this_drone.target_system,  # target_system
            this_drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
            0,                                             # confirmation
            1,                                             # param1 (0 to indicate disarm)
            0,                                             # param2 (all other params meaningless)
            0,                                             # param3
            0,                                             # param4
            0,                                             # param5
            0,                                             # param6
            0)                                             # param7
        sleep(0.001)

def land(drone):
    this_drone = drone[0]
    for iter_range in range(1, 5):
        this_drone.mav.set_mode_send(this_drone.target_system, 157, 100925440)
        sleep(0.001)


def start(drone):
    this_drone = drone[0]
    for iter_range in range(1, 5):
        '''
        #old command
        this_drone.mav.set_mode_send(
                    this_drone.target_system,
                    29,
                    67371008)
        '''
        this_drone.set_mode_auto()
        sleep(0.001)


def next_mission(drone):
    this_drone = drone[0]
    next_seq = this_drone.waypoint_current()+1
    for iter_range in range(1, 5):
        this_drone.mav.mission_set_current_send(
            this_drone.target_system,  # target_system
            this_drone.target_component,
            next_seq)  # seq
        sleep(0.01)


def continue_mission(drone):
    this_drone = drone[0]
    for iter_range in range(1, 5):
        this_drone.mav.set_mode_send(
                    this_drone.target_system,
                    29,
                    67371008)
        sleep(0.001)


def pause_mission(drone):
    this_drone = drone[0]
    for iter_range in range(1, 5):
        this_drone.mav.set_mode_send(
                    this_drone.target_system,
                    29,
                    50593792)
        sleep(0.001)


def kill(drone):
    this_drone = drone[0]
    this_drone.mav.command_long_send(
        this_drone.target_system,  # target_system
        this_drone.target_component,
        mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,  # command
        0,                                             # confirmation
        1,                                             # param1 (0 to indicate disarm)
        0,                                             # param2 (all other params meaningless)
        0,                                             # param3
        0,                                             # param4
        0,                                             # param5
        0,                                             # param6
        0)                                             # param7
    print('***************** warning ' + str(this_drone.target_system) + ' is killed, be careful. ********************')


def reboot(drone):
    this_drone = drone[0]
    print("reboot")
    for iter_range in range(1, 5):
        this_drone.mav.command_long_send(
            this_drone.target_system,  # target_system
            this_drone.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command
            0,                                             # confirmation
            1,                                             # param1 (0 to indicate disarm)
            0,                                             # param2 (all other params meaningless)
            0,                                             # param3
            0,                                             # param4
            0,                                             # param5
            0,                                             # param6
            0)                                             # param7
    sleep(0.001)


def gps_status(drone):
    this_drone = drone[0]
    try:
        gps_msg = this_drone.recv_match(type='GPS_RAW_INT', blocking=True)
        fix_state = gps_msg.fix_type
        altitude  = gps_msg.alt
    except:
        print("error")
        return

    print("fix_type : " + str(fix_state))
    print("altitude : " + str(altitude))


def upload_mission(drone):
    this_drone = drone[0]
    this_drone.mav.mission_clear_all_send(
        this_drone.target_system,
        this_drone.target_component,
        0
    )
    file_name = 'mission' + str(this_drone.target_system) + '.plan'
    print(file_name)
    with open(file_name, 'r') as file:
        file_lines = file.read().split('\n')
    # read_sequence
    total_seq = int(file_lines[0])
    this_drone.waypoint_count_send(total_seq)
    print(str(this_drone.target_system) + '` upload start')
    for now_seq in range(total_seq):
        line = file_lines[now_seq+2]
        param = line.split()
        while(mission_cnt is int(param[0])):
            #print(mission_cnt)
            #print(int(param[0]))
            this_drone.mav.mission_item_send(
                this_drone.target_system,
                0,
                int(param[0]),      # seq
                int(param[1]),      # frame
                int(param[2]),      # mav_cmd
                int(param[3]),      # current
                int(param[4]),      # autocontinue
                float(param[5]),    # param1 Minimum pitch (if airspeed sensor present), desired pitch without sensor
                float(param[6]),    # param2 (all other params meaningless)
                float(param[7]),    # param3
                float(param[8]),    # param4
                float(param[9]),    # param5 latitude
                float(param[10]),   # param6 longitude
                float(param[11]),   # param7 altitude
                int(param[12])      # mission_type
            )
            sleep(0.02)
    print(str(this_drone.target_system) + ' upload eeennd')


def upload_rtcm(drone, rtcm_data):
    this_drone = drone[0]
    full_rtcm_msg = rtcm_data[0]
    msg_max_length = 180
    #print("len :" + str(len(full_rtcm_msg)))
    if(len(full_rtcm_msg) < msg_max_length):
        while len(full_rtcm_msg) < 180:
            full_rtcm_msg.append(0)
        rtcm_msg_flag = (this_drone.mav.seq & 0x1F) << 3 
        rtcm_msg_len = len(full_rtcm_msg)
        this_drone.mav.gps_rtcm_data_send(
            rtcm_msg_flag,  #flags
            rtcm_msg_len,   #len
            full_rtcm_msg    #data_t[180]
        )
        #print("send success")
    else:
        fragment_id = 0
        start = 0
        length = 0 
        while(start < len(full_rtcm_msg)):
            length = min(len(full_rtcm_msg) - start, msg_max_length)
            rtcm_msg_flag = 1
            fragment_id += 1
            rtcm_msg_flag |= (fragment_id << 1)
            rtcm_msg_flag |= ((this_drone.MAV.seq & 0x1F) << 3)
            rtcm_msg_len = length
            this_drone.mav.gps_rtcm_data_send(
            rtcm_msg_flag,  #flags
            rtcm_msg_len,   #len
            #full_rtcm_msg[start:]    #data_t[180]
            )
            start += length
        





# read a message
def read_message(drone):
    #file_name = 'GPS_LOG_id_' + str(drone.target_system) + '_' + str(datetime.now()) + '.txt'
    while True:
        # msg = drone.recv_match(
        #     type=['HEARTBEAT'],
        #     blocking=True,
        #     timeout=0.001)
        drone.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                0,0,0)
        '''
        msg = drone.recv_match(
            type=['MISSION_COUNT', 'MISSION_ITEM_INT', 'MISSION_REQUEST_INT', 'MISSION_REQUEST',
                  'MISSION_ACK', 'GLOBAL_POSITION_INT', 'STATUSTEXT'], blocking=True, timeout=0.5)
        '''
        try:
            if msg.name is 'MISSION_ACK' and msg.type is 0:
                print(str(drone.target_system) + '` upload done')
                global mission_cnt
                mission_cnt = 0
            elif msg.name is 'MISSION_REQUEST':
                #print(msg)
                mission_cnt = msg.seq
            if msg.get_type() is 'HOME_POSITION':
                print(msg)
            else:
                #print(msg.name)
                pass
            # GPS_LOG
            #if msg.name is 'GLOBAL_POSITION_INT':            
                #with open(file_name, 'a') as log_file:
                    #log_data = str(msg.lat) + ' ' + str(msg.lon) + ' ' + str(msg.relative_alt) + '\n'
                    #log_file.write(log_data)
            
        except:
            pass


def getSingleRtcmMsg():
    dropMsg, rtcmFullmsg, rtcmDataFrame = [], [], []
    msgByteCount, rtcmId, rtcmLength = 0, 0, 0
    while True:
        nowByte = gpsSerial.read(1)
        nowByte = int(nowByte.encode('hex'),16)
        #print(intByte)
        if msgByteCount == 0:
            if nowByte == 211: #0xD3
                if len(dropMsg) != 0:
                    #print("[UNKNOWN] : ")
                    #print(dropMsg)
                    dropMsg = []
            else:
                dropMsg.append(nowByte)
                continue
        elif msgByteCount == 1:
            rtcmLength = (nowByte & 3) << 8
        elif msgByteCount == 2:
            rtcmLength += nowByte
        elif msgByteCount <= 2 + rtcmLength:
            rtcmDataFrame.append(nowByte)

            
        #elif (msgByteCount > 2 + rtcmLength) and (msgByteCount <= (2 + rtcmLength) + 2):
            #crc two bytes
        elif msgByteCount == (2 + rtcmLength) + 3:
            rtcmFullmsg.append(nowByte) #crc last byte
            rtcmId = rtcmFullmsg[3]
            rtcmId = rtcmId << 4
            rtcmId = rtcmId | ((rtcmFullmsg[4] >> 4) or 15)
            return [rtcmFullmsg, rtcmDataFrame, rtcmId]
        msgByteCount += 1
        rtcmFullmsg.append(nowByte)


def read_data():
    #file_name = raw_input('input file: ')
    file_name = 'port.txt'
    with open(file_name, 'r') as file:
        input_data = file.read().split('\n')
    input_data.pop()  #remove last data
    return input_data


def make_connection(input_data):
    failed_data = list()
    for lines in input_data:
        input_datum = lines.split()
        port_num = input_datum[0]
        start_sequence = input_datum[1]
        # drones_set is list of drone-tuple(vehicle, port, sequence)
        drone = mavutil.mavlink_connection('udp:0.0.0.0:' + str(port_num), force_connected=True)
        sleep(0.005)
        ack = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        print(ack)
        if ack is None:
            print('fail : ' + str(int(port_num)-22000))
            failed_data.append(str(port_num) + ' ' + str(start_sequence))
        else:
            print('Heartbeat from drone (System %u)' % drone.target_system)
            var_drones_set.append((drone, start_sequence))
            threading.Thread(target=read_message, args=(drone,)).start()
    if len(failed_data) is not 0:
        check_retry = raw_input('retry the failure things? (y/n) ')
        print(check_retry)
        if check_retry == 'y':
            make_connection(failed_data)
        elif check_retry == 'n':
            return


def coummunicator(drones_set):
    while True:
        csock = socket(AF_INET, SOCK STREAM)
        HOST = '127.0.0.1'
        PORT = 4000
        BUFSIZE = 7
        csock.connect((HOST,PORT))
        print("connect is success")
        
        command = csock.recv(BUFSIZE, MSG_WAITALL)
        data = command.decode("UTF-8")
        print("type : {}, data len : {}, data : {}, contents : {}".format(type(command),len(command), command, data))
        to_server = int(12345)
        right_method = to_server.to_bytes(4, byteorder = 'little')
        print("Send Data : {}, bytes len : {}, bytes : {}".format(to_server, len(right_method),right_method)
        sent = csock.send(right_method)
        sleep(5)





if __name__ == '__main__':
    #mission_cnt = 0
    #global mission_cnt
    var_drones_set = list()
    input_data = read_data()
    rtk_use = raw_input('rtk use? (y/n)')
    if rtk_use == 'y':
        #com_port = "COM" + str(raw_input("input : COM"))
        com_port = "COM21"
        gpsSerial = serial.Serial(com_port, 115200, timeout=None)
    make_connection(input_data)
    commander_thread = threading.Thread(target=commander, args=[var_drones_set])
    commander_thread.start()
    rtcmExplain = {1005 : "Stationary RTK reference station ARP", 1074 : "GPS MSM4", 1077 : "GPS MSM7", 1084 : "GLONASS MSM4", 1087 : "GLONASS MSM7",  1094 : "Galileo MSM4",  1097 : "Galileo MSM7",  1124 : "BeiDou MSM4",  1127 : "BeiDou MSM7", 1230 : "GLONASS code-phase biases", 4072 : "Reference station PVT (u-blox proprietary RTCM Message)"}
    while True and rtk_use == 'y':
        rtcmMsg = getSingleRtcmMsg()
        rtcmex = rtcmExplain[rtcmMsg[2]] if rtcmMsg[2] in rtcmExplain else str(rtcmMsg[2])
        if rtcmMsg[2] == 1074 or rtcmMsg[2] == 1084 or rtcmMsg[2] == 1077 or rtcmMsg[2] == 1087 or rtcmMsg[2] == 1005:
            #print("[RTCM] (", rtcmex , ") : ", rtcmMsg[0])
            for drone in var_drones_set:
                #print(rtcmex)
                upload_rtcm(drone, rtcmMsg)
        else:
            pass
            #print("unknown data")

        #print("[RTCM] (", rtcmex , ") : ", rtcmMsg[0])
