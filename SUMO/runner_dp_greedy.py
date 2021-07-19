#!/usr/bin/env python
# old version of dp
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import numpy as np
from collections import namedtuple
Sol = namedtuple("Sol", "time table")
Vehicle = namedtuple("Vehicle", "id time")

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


def generate_routefile(timeStep, N, pA, pB, pC):
    # random.seed(42)  # make tests reproducible
    with open("./sumo_data/laneMerging.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeA" type="passenger" length="2" accel="1.5" decel="2" sigma="0.0" maxSpeed="20" color="yellow"/>
        <vType id="typeB" type="passenger" length="2" accel="1.5" decel="2" sigma="0.0" maxSpeed="20" color="blue"/>
        <vType id="typeC" type="passenger" length="2" accel="1.5" decel="2" sigma="0.0" maxSpeed="20" color="magenta"/>

        <route edges="E6 E9" color="yellow" id="route_0"/>
        <route edges="E7 E9" color="yellow" id="route_1"/>
        <route edges="E7 E10" color="yellow" id="route_2"/>
        <route edges="E8 E10" color="yellow" id="route_3"/>""", file=routes)
        num_A = 1
        num_B = 1
        num_C = 1
        t = 1.0
        while num_A <= N or num_B <= N or num_C <= N:
            if num_A <= N and random.uniform(0, 1) < pA:
                print('    <vehicle id="A_%i" type="typeA" route="route_0" depart="%i" departLane="0" departSpeed="random"/>' % (
                    num_A, t), file=routes)
                num_A += 1
            if num_B <= N and random.uniform(0, 1) < pB:
                print('    <vehicle id="B_%i" type="typeB" route="route_1" depart="%i" departLane="0" departSpeed="random"/>' % (
                    num_B, t), file=routes)
                num_B += 1
            if num_C <= N and random.uniform(0, 1) < pC:
                print('    <vehicle id="C_%i" type="typeC" route="route_3" depart="%i" departLane="0" departSpeed="random"/>' % (
                    num_C, t), file=routes)
                num_C += 1
            t += timeStep
        print("</routes>", file=routes)


def compute_earliest_arrival(laneLength, schedule_A, schedule_BX, schedule_BY, schedule_C):
    a = [Vehicle('', 0)]
    b = [Vehicle('', 0)]
    c = [Vehicle('', 0)]
    currentTime = traci.simulation.getTime()

    # Vehicles in communication range on lane A
    for vehID in traci.lanearea.getLastStepVehicleIDs("dA"):
        dist = laneLength - traci.vehicle.getDistance(vehID)
        speed = traci.vehicle.getSpeed(vehID)
        arrivalTime = 0
        if speed == 0:
            for i in schedule_A:
                if i.id == vehID:
                    arrivalTime = i.time
                    break
        elif speed != 0:
            arrivalTime = currentTime + (dist/speed)
            for i in schedule_A:
                if i.id == vehID:
                    if i.time < arrivalTime:
                        arrivalTime = i.time
                    break
        a.append(Vehicle(vehID, arrivalTime))

    # Vehicles in communication range on lane B
    for vehID in traci.lanearea.getLastStepVehicleIDs("dB"):
        dist = laneLength - traci.vehicle.getDistance(vehID)
        speed = traci.vehicle.getSpeed(vehID)
        arrivalTime = 0
        if speed == 0:
            isFound = False
            for i in schedule_BX:
                if i.id == vehID:
                    arrivalTime = i.time
                    isFound = True
                    break
            if not isFound:
                for i in schedule_BY:
                    if i.id == vehID:
                        arrivalTime = i.time
                        break
        elif speed != 0:
            arrivalTime = currentTime + (dist/speed)
            isFound = False
            for i in schedule_BX:
                if i.id == vehID:
                    if i.time < arrivalTime:
                        arrivalTime = i.time
                        isFound = True
                    break
            if not isFound:
                for i in schedule_BY:
                    if i.id == vehID:
                        if i.time < arrivalTime:
                            arrivalTime = i.time
                        break
        b.append(Vehicle(vehID, arrivalTime))

    # Vehicles in communication range on lane B
    for vehID in traci.lanearea.getLastStepVehicleIDs("dC"):
        dist = laneLength - traci.vehicle.getDistance(vehID)
        speed = traci.vehicle.getSpeed(vehID)
        arrivalTime = 0
        if speed == 0:
            for i in schedule_C:
                if i.id == vehID:
                    arrivalTime = i.time
        elif speed != 0:
            arrivalTime = currentTime + (dist/speed)
            for i in schedule_C:
                if i.id == vehID:
                    if i.time < arrivalTime:
                        arrivalTime = i.time
                    break
        c.append(Vehicle(vehID, arrivalTime))
        a[1:] = sorted(a[1:], key=lambda x: int(x.id.split('_')[1]))
        b[1:] = sorted(b[1:], key=lambda x: int(x.id.split('_')[1]))
        c[1:] = sorted(c[1:], key=lambda x: int(x.id.split('_')[1]))

    return a, b, c


def get_obj(sol):
    return max(sol.time)

def compute_entering_time(a, b, c, W_same, W_diff):
    alpha = len(a) - 1
    beta = len(b) - 1
    gamma = len(c) - 1

    LX_A = [[Sol(float('inf'), '') for j in range(beta+1)]
                 for i in range(alpha+1)]
    LX_B = [[Sol(float('inf'), '') for j in range(beta+1)]
                 for i in range(alpha+1)]
    LY_C = [[Sol(float('inf'), '') for j in range(beta+1)]
                 for i in range(gamma+1)]
    LY_B = [[Sol(float('inf'), '') for j in range(beta+1)]
                 for i in range(gamma+1)]

    # Initialize
    LX_A[0][0] = Sol(0, '')
    LX_B[0][0] = Sol(0, '')
    LX_A[1][0] = Sol(a[1].time, '')
    # L1[0][1][1] = b[1].time
    LY_C[0][0] = Sol(0, '')
    LY_B[0][0] = Sol(0, '')
    LY_C[1][0] = Sol(c[1].time, '')
    # L2[0][1][1] = b[1].time

    for i in range(2, alpha+1):
        LX_A[i][0] = Sol(max(a[i].time, LX_A[i-1][0].time+W_same), 'A')
    # for i in range(1, alpha+1):
    #     LX_B[i][0] = Sol(LX_A[i][0]+W_diff, 'A')
    for i in range(2, gamma+1):
        LY_C[i][0] = Sol(max(c[i].time, LY_C[i-1][0].time+W_same), 'C')
    # for i in range(1, gamma+1):
    #     LY_B[i][0] = Sol(LY_C[i][0].time+W_diff, 'C')

    beta_sum = 1
    beta_X = 1
    beta_Y = 1 
    id_BX = []
    id_BY = []
    while beta_sum <= beta:
        if beta_X == 1:
            LX_B[0][beta_X] = Sol(b[beta_sum].time, '0')
        else:
            LX_B[0][beta_X] = Sol(max(b[beta_sum].time, LX_B[0][beta_X-1].time+W_same), 'B')
        if beta_Y == 1:
            LY_B[0][beta_Y] = Sol(b[beta_sum].time, '0')
        else:
            LY_B[0][beta_Y] = Sol(max(b[beta_sum].time, LY_B[0][beta_Y-1].time+W_same), 'B')
        for i in range(1, alpha+1):
            LX_A[i][beta_X] = min(
                                    Sol(max(a[i].time, LX_A[i-1][beta_X].time+W_same), 'A'),
                                    Sol(max(a[i].time, LX_B[i-1][beta_X].time+W_diff), 'B'),
                                    key=lambda x:x.time
            )
            LX_B[i][beta_X] = min(
                                    Sol(max(b[beta_sum].time, LX_A[i][beta_X-1].time+W_diff), 'A'),
                                    Sol(max(b[beta_sum].time, LX_B[i][beta_X-1].time+W_same), 'B'),
                                    key=lambda x:x.time      
            )
        for i in range(1, gamma+1):
            LY_C[i][beta_Y] = min(
                                    Sol(max(c[i].time, LY_C[i-1][beta_Y].time+W_same), 'C'),
                                    Sol(max(c[i].time, LY_B[i-1][beta_Y].time+W_diff), 'B'),
                                    key=lambda x:x.time
            )
            LY_B[i][beta_Y] = min(
                                    Sol(max(b[beta_sum].time, LY_C[i][beta_Y-1].time+W_diff), 'C'),
                                    Sol(max(b[beta_sum].time, LY_B[i][beta_Y-1].time+W_same), 'B'),
                                    key=lambda x:x.time
            )

        min_X = min(LX_A[alpha][beta_X].time, LX_B[alpha][beta_X].time)
        min_Y = min(LY_C[gamma][beta_Y].time, LY_B[gamma][beta_Y].time)
        if min_X <= min_Y:
            id_BX.append(b[beta_sum].id)
            beta_X += 1
        else:
            id_BY.append(b[beta_sum].id)
            beta_Y += 1
        beta_sum += 1

    # Choose optimal solution for Lane X
    beta_X -= 1
    beta_Y -= 1
    stack_A = []
    stack_BX = []
    prevTable = ''
    i = alpha
    j = beta_X
    if LX_A[i][j].time <= LX_B[i][j].time:
        stack_A.append(Vehicle(a[i].id, LX_A[i][j].time))
        prevTable = LX_A[i][j].table
        i -= 1
    else:
        stack_BX.append(Vehicle(id_BX.pop(), LX_B[i][j].time))
        prevTable = LX_B[i][j].table
        j -= 1

    # Backtracking LX
    while i > 0 or j > 0:
        if prevTable == 'A':
            stack_A.append(Vehicle(a[i].id, LX_A[i][j].time))
            prevTable = LX_A[i][j].table
            i -= 1
        elif prevTable == 'B':
            stack_BX.append(Vehicle(id_BX.pop(), LX_B[i][j].time))
            prevTable = LX_B[i][j].table
            j -= 1
        else:
            print('BUG')

    # Choose optimal solution for Lane Y
    stack_C = []
    stack_BY = []
    i = gamma
    j = beta_Y
    if LY_C[i][j].time <= LY_B[i][j].time:
        stack_C.append(Vehicle(c[i].id, LY_C[i][j].time))
        prevTable = LY_C[i][j].table
        i -= 1
    else:
        stack_BY.append(Vehicle(id_BY.pop(), LY_B[i][j].time))
        prevTable = LY_B[i][j].table
        j -= 1
    
    # Backtracking LY
    while i > 0 or j > 0:
        if prevTable == 'C':
            stack_C.append(Vehicle(c[i].id, LY_C[i][j].time))
            prevTable = LY_C[i][j].table
            i -= 1
        elif prevTable == 'B':
            stack_BY.append(Vehicle(id_BY.pop(), LY_B[i][j].time))
            prevTable = LY_B[i][j].table
            j -= 1
        else:
            print('BUG')

    # Output order
    # while len(order_stack) > 0:
    #     print(order_stack.pop())

    stack_A.reverse()
    stack_BX.reverse()
    stack_BY.reverse()
    stack_C.reverse()

    return stack_A, stack_BX, stack_BY, stack_C


def run(W_same, W_diff):
    # step = 0
    period=400
    # junction_x = traci.junction.getPosition("gneJ20")[0]
    schedule_A=[]
    schedule_BX=[]
    schedule_BY=[]
    schedule_C=[]
    leaveA=False
    leaveBX=False
    leaveBY=False
    leaveC=False
    countdownX=0
    countdownY=0
    # endTime = 0
    gA=False
    gBX=False
    gBY=False
    gC=False
    timeStep_cnt=0
    laneLength=6000
    # window_size = 5
    passTime_dX=0
    passTime_dY=0

    """execute the TraCI control loop"""
    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("0", 2)
    # The number of vehicles which are in the net plus the ones still waiting to start.
    while traci.simulation.getMinExpectedNumber() > 0:
        for vehID in traci.simulation.getLoadedIDList():
            traci.vehicle.setLaneChangeMode(vehID, 0b000000000000)
        traci.simulationStep()
        timeStep_cnt += 1
        traci.trafficlight.setPhase("TL1", 1)
        # print(f'timeSteps: {timeStep_cnt}')

        for vehID in traci.inductionloop.getLastStepVehicleIDs("dX"):
            passTime_dX=traci.simulation.getTime()
            if vehID[0] == 'A':
                # print('leaveA', vehID)
                leaveA=True
                for s in schedule_A:
                    if s.id == vehID:
                        schedule_A.remove(s)
                        break
            elif vehID[0] == 'B':
                # print('leaveBX', vehID)
                leaveBX=True
                for s in schedule_BX:
                    if s.id == vehID:
                        schedule_BX.remove(s)
                        break
        for vehID in traci.inductionloop.getLastStepVehicleIDs("dY"):
            passTime_dY=traci.simulation.getTime()
            if vehID[0] == 'C':
                # print('leaveC', vehID)
                leaveC=True
                for s in schedule_C:
                    if s.id == vehID:
                        schedule_C.remove(s)
                        break
            elif vehID[0] == 'B':
                # print('leaveBY', vehID)
                leaveBY=True
                for s in schedule_BY:
                    if s.id == vehID:
                        schedule_BY.remove(s)
                        break

        if timeStep_cnt - period == 0:
            if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0 and traci.lanearea.getLastStepVehicleNumber("dC") > 0:
                a_all, b_all, c_all=compute_earliest_arrival(
                    laneLength, schedule_A, schedule_BX, schedule_BY, schedule_C)
                # print('a_all', a_all)
                # print('b_all', b_all)
                # print('c_all', c_all)
                schedule_A, schedule_BX, schedule_BY, schedule_C = compute_entering_time(a_all, b_all, c_all, W_same, W_diff)
                print('s_A', schedule_A)
                print('s_BX', schedule_BX)
                print('s_BY', schedule_BY)
                print('s_C', schedule_C)
                schedule_A.sort(key=lambda x: x[1])
                schedule_BX.sort(key=lambda x: x[1])
                schedule_BY.sort(key=lambda x: x[1])
                schedule_C.sort(key=lambda x: x[1])
                # print('s_A', schedule_A)
                # print('s_BX', schedule_BX)
                # print('s_BY', schedule_BY)
                # print('s_C', schedule_C)
                for veh in schedule_BX:
                    try:
                        traci.vehicle.setRouteID(veh.id, "route_1")
                    except:
                        # print('leaveBY', veh)
                        # print(f'Remove {veh.id} from schedule_BX')
                        schedule_BX.remove(veh)
                        leaveBY=True
                for veh in schedule_BY:
                    try:
                        traci.vehicle.setRouteID(veh.id, "route_2")
                    except:
                        # print('leaveBX', veh)
                        # print(f'Remove {veh.id} from schedule_BY')
                        schedule_BY.remove(veh)
                        leaveBX=True
            # step = 0
        # for vehID in traci.simulation.getLoadedIDList():
        #     traci.vehicle.setLaneChangeMode(vehID, 0b000000000000)
        # traci.simulationStep()
        # timeStep_cnt += 1
        # traci.trafficlight.setPhase("TL1",1)
        # step += 1
        # currentTime = traci.simulation.getTime()
        # endTime = currentTime
        # print(currentTime)
        # print("Pass", traci.lane.getLastStepVehicleIDs("E2_0"))

        # if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0 and traci.lanearea.getLastStepVehicleNumber("dC") > 0:
        if traci.lanearea.getLastStepVehicleNumber("dA") > 0 or traci.lanearea.getLastStepVehicleNumber("dB") > 0 or traci.lanearea.getLastStepVehicleNumber("dC") > 0:
            gA=False
            gBX=False
            gBY=False
            gC=False
            # Control outgoing lane 1
            if len(schedule_A) > 0 and len(schedule_BX) > 0:
                if schedule_A[0].time < schedule_BX[0].time:
                    if leaveA:
                        countdownX=W_same
                    elif leaveBX:
                        countdownX=W_diff
                    elif not countdownX:
                        gA=True
                    # else:
                    #     if countdownX:
                    #         traci.trafficlight.setPhase("TL1", 16)
                    #         countdownX -= 1
                    #     else:
                    #         gA = True
                else:
                    if leaveBX:
                        countdownX=W_same
                    elif leaveA:
                        countdownX=W_diff
                    elif not countdownX:
                        gBX=True
                    # else:
                    #     if countdownX:
                    #         traci.trafficlight.setPhase("TL1", 16)
                    #         countdownX -= 1
                    #     else:
                    #         gBX = True
            elif len(schedule_A) > 0:
                if leaveA:
                    countdownX=W_same
                elif leaveBX:
                    countdownX=W_diff
                elif not countdownX:
                    gA=True
                # else:
                #     if countdownX:
                #         traci.trafficlight.setPhase("TL1", 16)
                #         countdownX -= 1
                #     else:
                #         gA = True
            elif len(schedule_BX) > 0:
                if leaveBX:
                    countdownX=W_same
                elif leaveA:
                    countdownX=W_diff
                elif not countdownX:
                    gBX=True
                # else:
                #     if countdownX:
                #         traci.trafficlight.setPhase("TL1", 16)
                #         countdownX -= 1
                #     else:
                #         gBX = True
            elif not countdownX:
                gA=True
                gBX=True
            # else:
            #     if countdownX:
            #         traci.trafficlight.setPhase("TL1", 16)
            #         countdownX -= 1
            #     else:
            #         gA = True
            #         gBX = True

            # Control outgoing lane 2
            if len(schedule_C) > 0 and len(schedule_BY) > 0:
                if schedule_C[0].time < schedule_BY[0].time:
                    if leaveC:
                        countdownY=W_same
                    elif leaveBY:
                        countdownY=W_diff
                    elif not countdownY:
                        gC=True
                    # else:
                    #     if countdownY:
                    #         traci.trafficlight.setPhase("TL1", 18)
                    #         countdownY -= 1
                    #     else:
                    #         gC = True
                else:
                    if leaveBY:
                        countdownY=W_same
                    elif leaveC:
                        countdownY=W_diff
                    elif not countdownY:
                        gBY=True
                    # else:
                    #     if countdownY:
                    #         traci.trafficlight.setPhase("TL1", 18)
                    #         countdownY -= 1
                    #     else:
                    #         gBY = True
            elif len(schedule_C) > 0:
                if leaveC:
                    countdownY=W_same
                elif leaveBY:
                    countdownY=W_diff
                elif not countdownY:
                    gC=True
                # else:
                #     if countdownY:
                #         traci.trafficlight.setPhase("TL1", 18)
                #         countdownY -= 1
                #     else:
                #         gC = True
            elif len(schedule_BY) > 0:
                if leaveBY:
                    countdownY=W_same
                elif leaveC:
                    countdownY=W_diff
                elif not countdownY:
                    gBY=True
                # else:
                #     if countdownY:
                #         traci.trafficlight.setPhase("TL1", 18)
                #         countdownY -= 1
                #     else:
                #         gBY = True
            elif not countdownY:
                gC=True
                gBY=True
            # else:
            #     if countdownY:
            #         traci.trafficlight.setPhase("TL1", 18)
            #         countdownY -= 1
            #     else:
            #         gC = True
            #         gBY = True

            if gA and gBX and gBY and gC:
                traci.trafficlight.setPhase("TL1", 0)
            elif gA and gBY:
                traci.trafficlight.setPhase("TL1", 2)
            elif gA and gC:
                traci.trafficlight.setPhase("TL1", 4)
            elif gBX and gC:
                traci.trafficlight.setPhase("TL1", 6)
            elif gBX and gBY:
                traci.trafficlight.setPhase("TL1", 20)
            elif gA:
                traci.trafficlight.setPhase("TL1", 8)
            elif gBX:
                traci.trafficlight.setPhase("TL1", 10)
            elif gBY:
                traci.trafficlight.setPhase("TL1", 12)
            elif gC:
                traci.trafficlight.setPhase("TL1", 14)
            else:
                traci.trafficlight.setPhase("TL1", 1)
        '''
        elif traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0:
            if (not countdownX) and (not countdownY):
                traci.trafficlight.setPhase("TL1", 2)
            elif not countdownX:
                traci.trafficlight.setPhase("TL1", 8)
            elif not countdownY:
                traci.trafficlight.setPhase("TL1", 12)
            for vehID in traci.lanearea.getLastStepVehicleIDs("dB"):
                try:
                    traci.vehicle.setRouteID(vehID, "route_2")
                except Exception as e:
                    print(e)
        elif traci.lanearea.getLastStepVehicleNumber("dC") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0:
            if (not countdownX) and (not countdownY):
                traci.trafficlight.setPhase("TL1", 6)
            elif not countdownX:
                traci.trafficlight.setPhase("TL1", 10)
            elif not countdownY:
                traci.trafficlight.setPhase("TL1", 14)
            for vehID in traci.lanearea.getLastStepVehicleIDs("dB"):
                try:
                    traci.vehicle.setRouteID(vehID, "route_1")
                except Exception as e:
                    print(e)
        else:
            if (not countdownX) and (not countdownY):
                traci.trafficlight.setPhase("TL1", 0)
            if traci.lanearea.getLastStepVehicleNumber("dB") > 0:
                id_list=sorted(traci.lanearea.getLastStepVehicleIDs(
                    "dB"), key=lambda x: int(x.split('_')[1]))
                if passTime_dX <= passTime_dY:
                    for i in range(len(id_list)):
                        if i % 2 == 0:
                            try:
                                traci.vehicle.setRouteID(id_list[i], "route_1")
                            except:
                                pass
                        else:
                            try:
                                traci.vehicle.setRouteID(id_list[i], "route_2")
                            except:
                                pass
                else:
                    for i in range(len(id_list)):
                        if i % 2 == 0:
                            try:
                                traci.vehicle.setRouteID(id_list[i], "route_2")
                            except:
                                pass
                        else:
                            try:
                                traci.vehicle.setRouteID(id_list[i], "route_1")
                            except:
                                pass

            # elif countdownX:
            #     traci.trafficlight.setPhase("TL1", 16)
            #     countdownX -= 1
            # elif countdownY:
            #     traci.trafficlight.setPhase("TL1", 18)
            #     countdownY -= 1
        '''
        if countdownX:
            # traci.trafficlight.setPhase("TL1", 16)
            countdownX -= 1
        if countdownY:
            # traci.trafficlight.setPhase("TL1", 18)
            countdownY -= 1

        # print(f'traffic light state: {gA}, {gBX}, {gBY}, {gC}')
        # print(f'phase: {traci.trafficlight.getPhase("TL1")}')
        leaveA=False
        leaveBX=False
        leaveBY=False
        leaveC=False

        # step += 1
        currentTime=traci.simulation.getTime()
        # endTime = currentTime
        # print(currentTime)

    # print(endTime)
    if passTime_dX >= passTime_dY:
        print(passTime_dX)
    else:
        print(passTime_dY)

    traci.close()
    sys.stdout.flush()


def get_options():
    optParser=optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args=optParser.parse_args()
    return options


def main():
    options=get_options()
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary=checkBinary('sumo')
    else:
        sumoBinary=checkBinary('sumo-gui')

    try:
        p=float(sys.argv[1])  # lambda for Poisson distribution
        N=int(sys.argv[2])  # Number of vehicles in each lane
        # the waiting time if two consecutive vehicles are from the same lane
        W_same=float(sys.argv[3])
        # the waiting time if two consecutive vehicles are from different lanes
        W_diff=float(sys.argv[4])
        isNewTest=sys.argv[5]
    except:
        print('Arguments: lambda, N, W=, W+, isNewTest')
        return

    timeStep=1    # The precision of time (in second)
    pA=p
    pB=p
    pC=p

    # first, generate the route file for this simulation
    if isNewTest == 'T':
        print('generate a new file')
        generate_routefile(timeStep, N, pA, pB, pC)

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "sumo_data/laneMerging.sumocfg",
                            "--tripinfo-output", "sumo_data/tripinfo_dp.xml",
                            "-S",
                            "--no-step-log", "true", "-W", "--duration-log.disable", "true"])
    run(W_same, W_diff)


if __name__ == "__main__":
    main()
