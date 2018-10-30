from __future__ import print_function
from time import *
from pymavlink import mavutil
import threading
from datetime import datetime
import math

mission_cnt = 0

class CommandThread(threading.Thread):
    def __init__(self, drone, command):
        threading.Thread.__init__(self)
        self.drone = drone
        self.command = command

    def run(self):
        if self.command == "arming":
            arming(self.drone)
        elif self.command == "disarming":
            disarming(self.drone)
        elif self.command == "takeoff":
            takeoff(self.drone)
        elif self.command == "land":
            land(self.drone)
        elif self.command == "start":
            start(self.drone)
        elif self.command == "next":
            next_mission(self.drone)
        elif self.command == "continue":
            continue_mission(self.drone)
        elif self.command == "pause":
            pause_mission(self.drone)
        elif self.command == "upload":
            upload_mission(self.drone)
        elif self.command == "kill":
            kill(self.drone)
        else:
            print("arming/ disarming/ takeoff/ land/ start/ next/ continue/ pause/ upload/ kill")


def commander(drones_set):
    while True:
        command = raw_input("commander? (upload, start, pause, continue, next, land, kill): ")
        command_to_drone(command, drones_set)


def command_to_drone(command, drones_set):
    try:
        split_command = command.split()
        target_command = split_command[0]
        drone_id_command = split_command[1]
        print(target_command)
        print(drone_id_command)
    except:
        print("You must need to input two argument!! command / drone_id(all or sys_id)")
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
        for command_thread in command_thread_list:
            command_thread.join()
        print("Start " + target_command)
    else:
        for drone in drones_set:
            if drone_id_command == str(drone[0].target_system):
                print("target : " + str(drone[0].target_system))
                command_thread = CommandThread(drone, target_command)
                command_thread.start()
                sleep(0.001)
                command_thread.join()
            else:
                return


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


def takeoff(drone):
    this_drone = drone[0]
    this_drone.mav.set_mode_send(
                this_drone.target_system,
                29,
                50593792)
    sleep(0.01)
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
    sleep(0.01)
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


def land(drone):
    this_drone = drone[0]
    this_drone.mav.set_mode_send(this_drone.target_system, 157, 100925440)


def start(drone):
    this_drone = drone[0]
    for iter_range in range(1, 5):
        '''
        #old command
        this_drone.mav.set_mode_send(
                    this_drone.target_system,
                    29,
                    67371008)'''
        this_drone.set_mode_auto()
        sleep(0.01)


def next_mission(drone):
    this_drone = drone[0]
    next_seq = this_drone.waypoint_current()+1
    for iter_range in range(1, 5):
        this_drone.mav.mission_set_current_send(
            this_drone.target_system,  # target_system
            this_drone.target_component,
            next_seq)  # seq


def continue_mission(drone):
    this_drone = drone[0]
    for iter_range in range(1, 5):
        this_drone.mav.set_mode_send(
                    this_drone.target_system,
                    29,
                    67371008)
        sleep(0.01)


def pause_mission(drone):
    this_drone = drone[0]
    for iter_range in range(1, 5):
        this_drone.mav.set_mode_send(
                    this_drone.target_system,
                    29,
                    50593792)
        sleep(0.01)


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
    print("***************** warning " + str(this_drone.target_system) + " is killed, be careful. ********************")


def upload_mission(drone):
    this_drone = drone[0]
    this_drone.mav.mission_clear_all_send(
        this_drone.target_system,
        this_drone.target_component,
        0
    )
    file_name = "mission" + str(this_drone.target_system) + ".plan"
    print(file_name)
    with open(file_name, "r") as file:
        file_lines = file.read().split('\n')
    # read_sequence
    total_seq = int(file_lines[0])
    this_drone.waypoint_count_send(total_seq)
    print(str(this_drone.target_system) + '` upload start'
    for now_seq in range(total_seq):
        line = file_lines[now_seq+2]
        param = line.split()
        while(mission_cnt is int(param[0])):
            print(mission_cnt)
            print(int(param[0]))
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
            sleep(0.1)
        print(str(this_drone.target_system) + '` upload done'


# read a message
def read_message(drone):
    file_name = 'GPS_LOG_id_' + str(drone.target_system) + '_' + str(datetime.now()) + '.txt'
    while True:
        msg = drone.recv_match(
            type=['MISSION_COUNT', 'MISSION_ITEM_INT', 'MISSION_REQUEST_INT', 'MISSION_REQUEST',
                  'MISSION_ACK', 'GLOBAL_POSITION_INT'], blocking=True, timeout=0.5)
        try:
            if msg.name is 'MISSION_ACK' and msg.type is 0:
                print(str(drone.target_system) + "' upload done")
                global mission_cnt
                mission_cnt = 0
            elif msg.name is 'MISSION_REQUEST':
                #print(msg)
                mission_cnt = msg.seq
            # GPS_LOG
            #if msg.name is 'GLOBAL_POSITION_INT':            
                #with open(file_name, "a") as log_file:
                    #log_data = str(msg.lat) + ' ' + str(msg.lon) + ' ' + str(msg.relative_alt) + '\n'
                    #log_file.write(log_data)
        except:
            pass


def read_data():
    #file_name = raw_input("input file: ")
    file_name = "port.txt"
    with open(file_name, "r") as file:
        input_data = file.read().split('\n')
    #input_data.pop()  #remove last data
    return input_data


def make_connection(input_data):
    failed_data = list()
    for lines in input_data:
        input_datum = lines.split()
        port_num = input_datum[0]
        start_sequence = input_datum[1]
        # drones_set is list of drone-tuple(vehicle, port, sequence)
        drone = mavutil.mavlink_connection("udp:0.0.0.0:" + str(port_num))
        sleep(0.005)
        ack = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        print(ack)
        if ack is None:
            print("fail : " + str(int(port_num)-22000))
            failed_data.append(str(port_num) + " " + str(start_sequence))
        else:
            print("Heartbeat from drone (System %u)" % drone.target_system)
            var_drones_set.append((drone, start_sequence))
            threading.Thread(target=read_message, args=(drone,)).start()
    if len(failed_data) is not 0:
        check_retry = raw_input("retry the failure things? (y/n) ")
        print(check_retry)
        if check_retry is "y":
            make_connection(failed_data)
        elif check_retry is "n":
            return


if __name__ == "__main__":
    global mission_cnt
    var_drones_set = list()
    input_data = read_data()
    make_connection(input_data)
    commander_thread = threading.Thread(target=commander, args=[var_drones_set])
    commander_thread.start()
