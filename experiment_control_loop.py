#!/usr/bin/env python
# Copyright (c) 2016-2022, Universal Robots A/S,
# Modified by Mertcan Kaya (2023-04-04)

import sys

sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

# my imports
import os
from datetime import datetime
import csv
import trajectory as trj
import numpy as np
import subprocess as sp

import time
from NatNetClient import NatNetClient

import robotiq_gripper

## RTDE toggle - True: Simulation, False: Real
no_rtde = False

## Trial toggle
trial_on = True

## Change for each participant
participant_num = 49 # staring from 1, increase by 1
movement_num = 1 # 1,2,3,4,5,6,7,8

# logging.basicConfig(level=logging.INFO)

# reference configuration
# Shoulder [-15,-95,-65,-15,15,270]
# Table [110,-45,110,-70,110,0]
        
def rot_y(rad):
    return np.array([[np.cos(rad),0.,np.sin(rad)],[0.,1.,0.],[-np.sin(rad),0.,np.cos(rad)]])

def rot_z(rad):
    return np.array([[np.cos(rad),-np.sin(rad),0.],[np.sin(rad),np.cos(rad),0.],[0.,0.,1.]])

def rot_x(rad):
    return np.array([[1.,0.,0.],[0.,np.cos(rad),-np.sin(rad)],[0.,np.sin(rad),np.cos(rad)]])

def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict

# Default functions
def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list

if no_rtde == False:
    def list_to_setp(sp, list):
        for i in range(0, 6):
            sp.__dict__["input_double_register_%i" % i] = list[i]
        return sp
else:
    def list_to_setp(sp, list):
        for i in range(0, 6):
            sp.__dict__["input_double_register_%i" % i] = list[i]
        return sp

# My function(s)
def print_list(plist):
    for i in range(0,len(plist)):
        if i < len(plist)-1:
            print("%7.4f" % plist[i], end=", ")
        else:
            print("%7.4f" % plist[i], end="\n")
    
if __name__ == "__main__":

    # OptiTrack initialization
    
    if no_rtde == False:
        optionsDict = {}
        optionsDict["clientAddress"] = "127.0.0.1"
        optionsDict["serverAddress"] = "127.0.0.1"
        optionsDict["use_multicast"] = True

        # This will create a new NatNet client
        optionsDict = my_parse_args(sys.argv, optionsDict)
        print(optionsDict)
        
        streaming_client = NatNetClient()
        streaming_client.set_client_address(optionsDict["clientAddress"])
        streaming_client.set_server_address(optionsDict["serverAddress"])
        streaming_client.set_use_multicast(optionsDict["use_multicast"])

        streaming_client.set_print_level(1)
        
        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        is_running = streaming_client.run()

        if not is_running:
            print("ERROR: Could not start streaming client.")
            try:
                sys.exit(1)
            except SystemExit:
                print("...")
            finally:
                print("exiting")
        
        # configure recipes (host, port)
        ROBOT_HOST = "10.8.0.231" # default: "localhost"
        ROBOT_PORT = 30004

        ## robotiq gripper
        def log_info(gripper):
            print(f"Pos: {str(gripper.get_current_position()): >3}  "
                f"Open: {gripper.is_open(): <2}  "
                f"Closed: {gripper.is_closed(): <2}  ")

        print("Creating gripper...")
        gripper = robotiq_gripper.RobotiqGripper()
        print("Connecting to gripper...")
        gripper.connect(ROBOT_HOST, 63352)
        print("Activating gripper...")
        gripper.activate()

        # configure recipes
        config_filename = "control_loop_configuration.xml"
    else:
        class gripper:
            def move(position: int, speed: int, force: int):
                nothing = 0            

    ## My initial coding starts here

    print('===============================================================')
    print('Pick-and-Place Experiment: Participant #' + str(participant_num))
    print('Motion #' + str(movement_num))

    ## Enum

    # mounting configuration
    SHLDRM = 0 # shoulder mount
    TABLEM = 1 # table mount

    # motion direction
    SAME_D = 0 # same direction
    OPPO_D = 1 # opposide direction

    # trajectory curvature
    CURVED = 0 # curved
    STRGHT = 1 # straight

    ## Via-Points in time

    # Mot Dir;Trj Cur
    A = np.array([[SAME_D],[CURVED]])
    B = np.array([[OPPO_D],[CURVED]])
    C = np.array([[SAME_D],[STRGHT]])
    D = np.array([[OPPO_D],[STRGHT]])

    # Latin square
    lat_sqr = np.zeros((2,4,4))
    lat_sqr[:,:,0] = np.concatenate((A,B,D,C), axis=1)
    lat_sqr[:,:,1] = np.concatenate((B,C,A,D), axis=1)
    lat_sqr[:,:,2] = np.concatenate((C,D,B,A), axis=1)
    lat_sqr[:,:,3] = np.concatenate((D,A,C,B), axis=1)

    # mounting sequence
    pn_rem2 = (participant_num-1) % 2
    mn_div = (movement_num-1) // 4

    if (pn_rem2 == 0 and mn_div == 0) or (pn_rem2 == 1 and mn_div == 1):
        mounting_seq = SHLDRM
        print('Mounting configuretion: Shoulder')
    elif (pn_rem2 == 0 and mn_div == 1) or (pn_rem2 == 1 and mn_div == 0):
        mounting_seq = TABLEM
        print('Mounting configuretion: Table')
    else:
        mounting_seq = -1
        print('Mounting configuretion: ERROR!')

    pn_rem = (participant_num-1) % 4
    mn_rem = (movement_num-1) % 4

    direction_seq = lat_sqr[0,mn_rem,pn_rem]
    curvature_seq = lat_sqr[1,mn_rem,pn_rem]

    if direction_seq == SAME_D and curvature_seq == CURVED:
        mot_str = 'A'
        print('Motion sequence ' + mot_str)
        print(' Motion Direction: Same')
        print(' Trajectory Curvature: Curved')
    elif direction_seq == OPPO_D and curvature_seq == CURVED:
        mot_str = 'B'
        print('Motion sequence ' + mot_str)
        print(' Motion Direction: Opposite')
        print(' Trajectory Curvature: Curved')
    elif direction_seq == SAME_D and curvature_seq == STRGHT:
        mot_str = 'C'
        print('Motion sequence ' + mot_str)
        print(' Motion Direction: Same')
        print(' Trajectory Curvature: Straight')
    elif direction_seq == OPPO_D and curvature_seq == STRGHT:
        mot_str = 'D'
        print('Motion sequence ' + mot_str)
        print(' Motion Direction: Opposite')
        print(' Trajectory Curvature: Straight')
    else:
        mot_str = 'ERROR!'
    print('===============================================================')

    # ## Change when appearance changes
    # mount_config = 0 # 0: table, 1: shoulder

    ## constrain the minimum-jerk trajectory (False: curved, True: straight)
    if curvature_seq == CURVED:
        mj_constrained = False
    else:
        mj_constrained = True

    if trial_on == True:
        repeat_num = 2
    else:
        repeat_num = 15
    
    # Generate trajectory on task or joint space
    task_space = True

    # recording options
    if trial_on == True:
        record_external = False
        record_internal = False
    else:
        record_external = False
        record_internal = True

    ## Enum declarations

    m_approach = 1
    m_grab = 2
    m_transport_high = 3
    m_put = 4
    m_release = 5
    m_recede = 6
    m_pull = 7
    m_transport_low = 8
    m_wait = 9

    mot_seq = np.array([m_approach,m_grab,m_wait,m_transport_high,m_put,m_release,m_recede,m_wait,m_approach,m_grab,m_pull,m_transport_low,m_wait,m_release,m_recede,m_wait])

    sequence_num = mot_seq.shape[0]

    if mounting_seq == SHLDRM:
        # Shoulder
        Pos_ini = np.array([0.2000,-0.2500,0.5400])
        #Orn_ini = np.deg2rad(np.array([0,90,0]))
        Orn_ini = np.array([1.199626905,-1.207793505,1.204287902])
    else:
        # Table
        Pos_ini = np.array([0.2000,-0.2700,0.0400])
        Orn_ini = np.deg2rad(np.array([90,0,0]))

    # incremental values
    app_rec_dist = 0.12     # m
    if mounting_seq == SHLDRM:
        #hover_dist = 0.03      # m
        hover_dist = 0.0315      # m
    else:
        hover_dist = 0.027      # m
    app_rec_duration = 0.5    # sec
    grab_rel_duration = 0.5   # sec
    put_duration = 0.5        # sec
    wait_duration = 0.5       # sec

    transport_duration = np.array([1,1])

    Pos_transport = np.zeros((3,3))
    if mounting_seq == SHLDRM:
        # Shoulder
        Pos_transport[:,0] = np.array([-0.3000,0.0000,0.0000])
        Pos_transport[:,1] = np.array([0.0000,0.0000,-0.3000])
    else:
        # Table
        Pos_transport[:,0] = np.array([0.0000,0.0000,0.3000])
        Pos_transport[:,1] = np.array([-0.3000,0.0000,0.0000])
    #                                   x-axis,y-axis,z-axis

    OrnMJ = np.zeros((3,3))
    OrnMJ[:,0] = Orn_ini
    OrnMJ[:,1] = Orn_ini
    OrnMJ[:,2] = Orn_ini

    Orn = np.zeros((3,2))
    Orn[:,0] = Orn_ini
    Orn[:,1] = Orn_ini
    
    # reference configuration
    if mounting_seq == SHLDRM:
        # Shoulder
        qPrev = np.deg2rad(np.array([-15,-95,-65,-15,15,270]))
    else:
        # Table
        qPrev = np.deg2rad(np.array([110,-45,110,-70,110,0]))
    
    trnsprt_num = transport_duration.shape[0]

    Vel = np.zeros((3,2))
    Acc = np.zeros((3,2))

    if mj_constrained == True:
        VelMJ = np.zeros((3,trnsprt_num+1))
        AccMJ = np.zeros((3,trnsprt_num+1))
    else:
        VelMJ = np.zeros((3,2))
        AccMJ = np.zeros((3,2))
        
    point_num = sequence_num+trnsprt_num-1

    trnsprt_seq = 0
    for i in range(sequence_num):
        if mot_seq[i] == m_transport_high or mot_seq[i] == m_transport_low:
            trnsprt_seq = trnsprt_seq + 1

    Pos = np.zeros((3,sequence_num+trnsprt_num-1))
    Pos[:,0] = Pos_ini
    T = np.zeros((1,sequence_num+trnsprt_num-1))
    PosMJ = np.zeros((3,trnsprt_num+1))
    TMJ = np.zeros((1,trnsprt_num+1))

    if mounting_seq == SHLDRM:
        # Shoulder
        hover_vec = np.array([hover_dist,0,0])
    else:
        # Table
        hover_vec = np.array([0,0,-hover_dist])
            
    Cj = np.zeros((6*3,sequence_num))
    Coj = np.zeros((6*3,sequence_num))
    Cj_trnsprt = np.zeros((6*6,sequence_num))
    Coj_trnsprt = np.zeros((6*6,sequence_num))
    for i in range(1,sequence_num+1):
        if mot_seq[i-1] == m_approach or mot_seq[i-1] == m_recede or mot_seq[i-1] == m_put or mot_seq[i-1] == m_pull:
            if mot_seq[i-1] == m_approach:
                Pos[:,i] = Pos[:,i-1] - np.array([0,app_rec_dist,0])
                T[:,i] = T[:,i-1] + app_rec_duration
            elif mot_seq[i-1] == m_recede:
                Pos[:,i] = Pos[:,i-1] + np.array([0,app_rec_dist,0])
                T[:,i] = T[:,i-1] + app_rec_duration
            elif mot_seq[i-1] == m_put:
                Pos[:,i] = Pos[:,i-1] + hover_vec
                T[:,i] = T[:,i-1] + put_duration
            elif mot_seq[i-1] == m_pull:
                Pos[:,i] = Pos[:,i-1] - hover_vec
                T[:,i] = T[:,i-1] + put_duration
            Cj[:,i-1] = np.squeeze(trj.minimumJerkCoefficient(T[:,i-1:i+1]-T[:,i-1],Pos[:,i-1:i+1],Vel,Acc))
            Coj[:,i-1] = np.squeeze(trj.minimumJerkCoefficient(T[:,i-1:i+1]-T[:,i-1],Orn,Vel,Acc))
        elif mot_seq[i-1] == m_transport_high or mot_seq[i-1] == m_transport_low:
            PosMJ[:,0] = Pos[:,i-1]
            TMJ[:,0] = T[:,i-1]
            for k in range(trnsprt_num):
                if mot_seq[i-1] == m_transport_high:
                    PosMJ[:,k+1] = PosMJ[:,k] + Pos_transport[:,k]
                else:
                    PosMJ[:,k+1] = PosMJ[:,k] - Pos_transport[:,trnsprt_num-k-1]
                TMJ[:,k+1] = TMJ[:,k] + transport_duration[k]
            Pos[:,i] = PosMJ[:,trnsprt_num]
            T[:,i] = T[:,i-1] + np.sum(transport_duration)
            if mj_constrained == True:
                Cj_trnsprt[:,i-1] = np.squeeze(trj.minimumJerkConstrCoefficient(TMJ-TMJ[:,0],PosMJ,VelMJ,AccMJ))
                Coj_trnsprt[:,i-1] = np.squeeze(trj.minimumJerkConstrCoefficient(TMJ-TMJ[:,0],OrnMJ,VelMJ,AccMJ))
            else:
                Cj_trnsprt[:,i-1] = np.squeeze(trj.minimumJerkCoefficient(TMJ-TMJ[:,0],PosMJ,VelMJ,AccMJ))
                Coj_trnsprt[:,i-1] = np.squeeze(trj.minimumJerkCoefficient(TMJ-TMJ[:,0],OrnMJ,VelMJ,AccMJ))
        elif mot_seq[i-1] == m_grab or mot_seq[i-1] == m_release:
            Pos[:,i] = Pos[:,i-1]
            T[:,i] = T[:,i-1] + grab_rel_duration
        elif mot_seq[i-1] == m_wait:
            Pos[:,i] = Pos[:,i-1]
            T[:,i] = T[:,i-1] + wait_duration

    ## Trajectory interpolation

    tstp = 0.008

    total_step = int(T[0,point_num-1]/tstp+1)
    t = np.squeeze(T[0,0])

    ## Return to original coding here

    if no_rtde == False:
        logging.getLogger().setLevel(logging.INFO)

        conf = rtde_config.ConfigFile(config_filename)
        state_names, state_types = conf.get_recipe("state")
        setp_names, setp_types = conf.get_recipe("setp")
        command_names, command_types = conf.get_recipe("command") # my addition
        watchdog_names, watchdog_types = conf.get_recipe("watchdog")

        # connect to controller
        con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        con.connect()

        # get controller version
        con.get_controller_version()

        print('Connected to robot') # my notification display

        # setup recipes
        con.send_output_setup(state_names, state_types)
        setp = con.send_input_setup(setp_names, setp_types)
        command = con.send_input_setup(command_names, command_types) # my addition
        watchdog = con.send_input_setup(watchdog_names, watchdog_types)
    else:
        class doubleClass:
            def __init__(self):
                self.input_double_register_0 = 0.0
                self.input_double_register_1 = 0.0
                self.input_double_register_2 = 0.0
                self.input_double_register_3 = 0.0
                self.input_double_register_4 = 0.0
                self.input_double_register_5 = 0.0
        
        class intClass:
            def __init__(self):
                self.input_int_register_0 = 0
                self.input_int_register_1 = 0
                self.input_int_register_2 = 0
                self.input_int_register_3 = 0

        setp = doubleClass()
        watchdog = intClass()
        command = intClass()

        class con:
            def send(self):
                nothing = 0
            
            def send_start():
                return 1
            
            def receive():
                return 1
            
            def send_pause():
                nothing = 0

            def disconnect():
                nothing = 0

        play_state = 2
        continue_state = 1

    # My recording implementations
    if record_external == True or record_internal == True:
        # detect the current working directory and print it
        patha = os.getcwd()
        print ("The current working directory is %s" % patha)

        # define the name of the directory to be created
        save_path = "participant " + str(participant_num)
        pathb = patha + '/' + save_path

        try:
            os.mkdir(pathb)
        except OSError:
            print ("Creation of the directory %s failed" % pathb)
        else:
            print ("Successfully created the directory %s " % pathb)
    if record_external == True:
        # run record.py
        extProc = sp.Popen(['python','record.py'])
    if record_internal == True:
        now = datetime.now()
        d_string = now.strftime("%Y-%m-%d %H.%M.%S")
        
        if mounting_seq == SHLDRM:
            mnt_chr = "S"
        else:
            mnt_chr = "T"
        
        filename = str(movement_num) + '-' + mot_str + ' ' + mnt_chr + ' ' + d_string + '.csv'
        completeName = save_path + '\\' + filename
        # log actual joint pos
        outfile = open(completeName, 'w', newline='')
        writer = csv.writer(outfile, delimiter=',')
        list_time = ['time']
        list_state = ['state']
        if no_rtde == True:
            list_actual_data = ['actual_TCP_pose0', 'actual_TCP_pose1', 'actual_TCP_pose2', 'actual_TCP_pose3', 'actual_TCP_pose4', 'actual_TCP_pose5']
        else:
            list_actual_data = ['actual_q0', 'actual_q1', 'actual_q2', 'actual_q3', 'actual_q4', 'actual_q5']
        writer.writerow(list_time+list_state+list_actual_data)

    # Initialize joint positions
    pos_out = Pos[:,0]
    orn_out = Orn[:,0]

    setlist = [pos_out[0], pos_out[1], pos_out[2], orn_out[0], orn_out[1], orn_out[2]]

    list_to_setp(setp, setlist)

    # The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog (deactivated)
    watchdog.input_int_register_0 = 0

    # joint_space:0, task_space:1
    command.input_int_register_1 = 1

    # init_joint:0, init_task:1
    command.input_int_register_2 = 1
            
    # trajectory command
    command.input_int_register_3 = 1
    
    # start data synchronization
    if not con.send_start():
        sys.exit()

    con.send(command)

    print('Press play to start')

    # control loop
    idx = 1 # counter
    t_inner = t
    stepcount = 1
    cur_seq = 1
    repeatcount = 1

    keep_running = True
    while keep_running:

        if no_rtde == False:
            # receive the current state
            state = con.receive()

            if state is None:
                break

            play_state = state.runtime_state
            continue_state = state.output_int_register_0

        # do something...
        if play_state == 2: # Play button is pressed

            if continue_state == 1: # Continue button is pressed
                
                if record_internal == True:
                    now = datetime.now()
                    tm_string = now.strftime("%H.%M.%S.%f")

                    if no_rtde == False:
                        actqlist = [state.actual_q[0], state.actual_q[1], state.actual_q[2], state.actual_q[3], state.actual_q[4], state.actual_q[5]]
                    else:
                        actqlist = [pos_out[0], pos_out[1], pos_out[2], orn_out[0], orn_out[1], orn_out[2]]

                    # write data
                    writer.writerow([tm_string, stepcount]+actqlist)

                if idx < total_step: # robot arm is tracking the trajectory

                    if t > T[:,stepcount]:
                        stepcount = stepcount + 1
                        t_inner = 0
                        cur_seq = mot_seq[stepcount-1]
                    
                    if (cur_seq == m_grab):
                        gripper.move(255, 10, 255) # close
                    elif (cur_seq == m_release):
                        gripper.move(0, 10, 255) # open

                    if (cur_seq == m_approach or cur_seq == m_put or cur_seq == m_recede or cur_seq == m_pull):
                        pos_out = trj.minimumJerkPolynomial(t_inner,T[:,stepcount-1:stepcount+1]-T[:,stepcount-1],Cj[:,stepcount-1])
                        # orn_out = trj.minimumJerkPolynomial(t_inner,T[:,stepcount-1:stepcount+1]-T[:,stepcount-1],Coj[:,stepcount-1])
                    elif (cur_seq == m_transport_high or cur_seq == m_transport_low):
                        pos_out = trj.minimumJerkPolynomial(t_inner,TMJ-TMJ[:,0],Cj_trnsprt[:,stepcount-1])
                        # orn_out = trj.minimumJerkPolynomial(t_inner,TMJ-TMJ[:,0],Coj_trnsprt[:,stepcount-1])
                    
                    setlist = [pos_out[0], pos_out[1], pos_out[2], orn_out[0], orn_out[1], orn_out[2]]

                    t_inner = round(t_inner + tstp,3)
                    t = round(t + tstp,3)
                    idx = idx + 1
                else:
                    if repeatcount == repeat_num:
                        if record_internal == True:
                            # shut down
                            outfile.close()

                        if record_external == True:
                            # terminate record.py
                            sp.Popen.terminate(extProc)

                        if (no_rtde == False):
                            # trajectory command
                            command.input_int_register_3 = 0
                        else:
                            continue_state = 0

                        con.send(command)

                        keep_running = False
                    else:
                        repeatcount = repeatcount + 1
                        
                        t = np.squeeze(T[:,0])
                        idx = 1 # counter
                        t_inner = t
                        stepcount = 1
                        cur_seq = 1
                
            list_to_setp(setp, setlist)
            # send new setpoint        
            con.send(setp)

        # kick watchdog
        con.send(watchdog)

    #print(setp)
    # output: <rtde.serialize.DataObject object at 0x000001B09B24BCD0>
    #print(setp.__dict__)
    # output: {'input_double_register_0': 0.2, 'input_double_register_1': -0.25000060705079286, 'input_double_register_2': 0.54, 'input_double_register_3': 1.199626905, 'input_double_register_4': -1.207793505, 'input_double_register_5': 1.204287902, 'recipe_id': 1}

    print('Disconnected')
    con.send_pause()
    con.disconnect()
