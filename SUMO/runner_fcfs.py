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

from numpy.core.shape_base import stack
Vehicle = namedtuple("Vehicle", "id time")

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


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
        a[1:] = sorted(a[1:],key=lambda x: int(x.id.split('_')[1]))
        b[1:] = sorted(b[1:],key=lambda x: int(x.id.split('_')[1]))
        c[1:] = sorted(c[1:],key=lambda x: int(x.id.split('_')[1]))

    return a, b, c


def simple_compute_entering_time(lane, traffic, W_same, W_diff, prevLane, prevTime):
    schedule = []
    if prevLane == '':
        schedule.append(Vehicle(traffic[1].id, traffic[1].time))
    elif lane == prevLane:
        schedule.append(Vehicle(traffic[1].id, max(traffic[1].time, prevTime+W_same)))
    else:
        schedule.append(Vehicle(traffic[1].id, max(traffic[1].time, prevTime+W_diff)))
    for i in range(2, len(traffic)):
        prevTime = schedule[-1].time
        schedule.append(Vehicle(traffic[i].id, max(traffic[i].time, prevTime+W_same)))
    return schedule


def fcfs_compute_entering_time(a, b, c, W_same, W_diff, X_lastT, Y_lastT):
    a = a[1:]
    b = b[1:]
    c = c[1:]
    X_lastFrom = ''
    Y_lastFrom = ''
    schedule_A = []
    schedule_BX = []
    schedule_BY = []
    schedule_C = []

    while len(a) > 0 or len(b) > 0 or len(c) > 0:
        if len(a) > 0 and len(b) > 0 and len(c) > 0:
            first = min(a[0].time, b[0].time, c[0].time)
            if first == a[0].time:
                if X_lastFrom == 'A':
                    X_lastT = max(a[0].time, X_lastT+W_same)
                else:
                    X_lastT = max(a[0].time, X_lastT+W_diff)
                X_lastFrom = 'A'
                schedule_A.append(Vehicle(a[0].id, X_lastT))
                a.pop(0)
            if first == c[0].time:
                if Y_lastFrom == 'C':
                    Y_lastT = max(c[0].time, Y_lastT+W_same)
                else:
                    Y_lastT = max(c[0].time, Y_lastT+W_diff)
                Y_lastFrom = 'C'
                schedule_C.append(Vehicle(c[0].id, Y_lastT))
                c.pop(0)
            if first  == b[0].time:
                if X_lastT < Y_lastT:
                    if X_lastFrom == 'B':
                        X_lastT = max(b[0].time, X_lastT+W_same)
                    else:
                        X_lastT = max(b[0].time, X_lastT+W_diff)
                    X_lastFrom = 'B'
                    schedule_BX.append(Vehicle(b[0].id, X_lastT))
                    b.pop(0)
                elif Y_lastT < X_lastT:
                    if Y_lastFrom == 'B':
                        Y_lastT = max(b[0].time, Y_lastT+W_same)
                    else:
                        Y_lastT = max(b[0].time, Y_lastT+W_diff)
                    Y_lastFrom = 'B'
                    schedule_BY.append(Vehicle(b[0].id, Y_lastT))
                    b.pop(0)
                else: # X_lastT == Y_lastT
                    if a[0] > c[0]:
                        if X_lastFrom == 'B':
                            X_lastT = max(b[0].time, X_lastT+W_same)
                        else:
                            X_lastT = max(b[0].time, X_lastT+W_diff)
                        X_lastFrom = 'B'
                        schedule_BX.append(Vehicle(b[0].id, X_lastT))
                        b.pop(0)
                    elif c[0] > a[0]:
                        if Y_lastFrom == 'B':
                            Y_lastT = max(b[0].time, Y_lastT+W_same)
                        else:
                            Y_lastT = max(b[0].time, Y_lastT+W_diff)
                        Y_lastFrom = 'B'
                        schedule_BY.append(Vehicle(b[0].id, Y_lastT))
                        b.pop(0)
                    else:
                        if random.randint(0,1) == 0:
                            if X_lastFrom == 'B':
                                X_lastT = max(b[0].time, X_lastT+W_same)
                            else:
                                X_lastT = max(b[0].time, X_lastT+W_diff)
                            X_lastFrom = 'B'
                            schedule_BX.append(Vehicle(b[0].id, X_lastT))
                            b.pop(0)
                        else:
                            if Y_lastFrom == 'B':
                                Y_lastT = max(b[0].time, Y_lastT+W_same)
                            else:
                                Y_lastT = max(b[0].time, Y_lastT+W_diff)
                            Y_lastFrom = 'B'
                            schedule_BY.append(Vehicle(b[0].id, Y_lastT))
                            b.pop(0)
        
        elif len(a) > 0 and len(b) > 0:
            schedule_A += simple_compute_entering_time('A', a, W_same, W_diff, X_lastFrom, X_lastT)
            schedule_BY += simple_compute_entering_time('B', b, W_same, W_diff, Y_lastFrom, Y_lastT)
            a.clear()
            b.clear()
            # first = min(a[0].time, b[0].time)
            # if first == a[0].time:
            #     if X_lastFrom == 'A':
            #         X_lastT = max(a[0].time, X_lastT+W_same)
            #     else:
            #         X_lastT = max(a[0].time, X_lastT+W_diff)
            #     X_lastFrom = 'A'
            #     schedule_A.append(Vehicle(a[0].id, X_lastT))
            #     a.pop(0)
            # if first == b[0].time:
            #     if X_lastT < Y_lastT:
            #         if X_lastFrom == 'B':
            #             X_lastT = max(b[0].time, X_lastT+W_same)
            #         else:
            #             X_lastT = max(b[0].time, X_lastT+W_diff)
            #         X_lastFrom = 'B'
            #         schedule_BX.append(Vehicle(b[0].id, X_lastT))
            #         b.pop(0)
            #     else:
            #         if Y_lastFrom == 'B':
            #             Y_lastT = max(b[0].time, Y_lastT+W_same)
            #         else:
            #             Y_lastT = max(b[0].time, Y_lastT+W_diff)
            #         Y_lastFrom = 'B'
            #         schedule_BY.append(Vehicle(b[0].id, Y_lastT))
            #         b.pop(0)
        elif len(b) > 0 and len(c) > 0:
            schedule_BX += simple_compute_entering_time('B', b, W_same, W_diff, X_lastFrom, X_lastT)
            schedule_C += simple_compute_entering_time('C', c, W_same, W_diff, Y_lastFrom, Y_lastT)
            b.clear()
            c.clear()
            # first = min(b[0].time, c[0].time)
            # if first == c[0].time:
            #     if Y_lastFrom == 'C':
            #         Y_lastT = max(c[0].time, Y_lastT+W_same)
            #     else:
            #         Y_lastT = max(c[0].time, Y_lastT+W_diff)
            #     Y_lastFrom = 'C'
            #     schedule_C.append(Vehicle(c[0].id, Y_lastT))
            #     c.pop(0)
            # if first == b[0].time:
            #     if Y_lastT < X_lastT:
            #         if Y_lastFrom == 'B':
            #             Y_lastT = max(b[0].time, Y_lastT+W_same)
            #         else:
            #             Y_lastT = max(b[0].time, Y_lastT+W_diff)
            #         Y_lastFrom = 'B'
            #         schedule_BY.append(Vehicle(b[0].id, Y_lastT))
            #         b.pop(0)
            #     else:
            #         if X_lastFrom == 'B':
            #             X_lastT = max(b[0].time, X_lastT+W_same)
            #         else:
            #             X_lastT = max(b[0].time, X_lastT+W_diff)
            #         X_lastFrom = 'B'
            #         schedule_BX.append(Vehicle(b[0].id, X_lastT))
            #         b.pop(0)
        elif len(a) > 0 and len(c) > 0:
            schedule_A += simple_compute_entering_time('A', a, W_same, W_diff, X_lastFrom, X_lastT)
            schedule_C += simple_compute_entering_time('C', c, W_same, W_diff, Y_lastFrom, Y_lastT)
            a.clear()
            c.clear()
            # # a->X, c->Y
            # if X_lastFrom == 'A':
            #     X_lastT = max(a[0].time, X_lastT+W_same)
            # else:
            #     X_lastT = max(a[0].time, X_lastT+W_diff)
            # X_lastFrom = 'A'
            # schedule_A.append(Vehicle(a[0].id, X_lastT))
            # a.pop(0)
            # while len(a) > 0:
            #     X_lastT = max(a[0].time, X_lastT+W_same)
            #     schedule_A.append(Vehicle(a[0].id, X_lastT))
            #     a.pop(0)
            # if Y_lastFrom == 'C':
            #     Y_lastT = max(c[0].time, Y_lastT+W_same)
            # else:
            #     Y_lastT = max(c[0].time, Y_lastT+W_diff)
            # Y_lastFrom = 'C'
            # schedule_C.append(Vehicle(c[0].id, Y_lastT))
            # c.pop(0)
            # while len(c) > 0:
            #     Y_lastT = max(c[0].time, Y_lastT+W_same)
            #     schedule_C.append(Vehicle(c[0].id, Y_lastT))
            #     c.pop(0)
        elif len(a) > 0:
            schedule_A += simple_compute_entering_time('A', a, W_same, W_diff, X_lastFrom, X_lastT)
            a.clear()
            # # a->X
            # if X_lastFrom == 'A':
            #     X_lastT = max(a[0].time, X_lastT+W_same)
            # else:
            #     X_lastT = max(a[0].time, X_lastT+W_diff)
            # X_lastFrom = 'A'
            # schedule_A.append(Vehicle(a[0].id, X_lastT))
            # a.pop(0)
            # while len(a) > 0:
            #     X_lastT = max(a[0].time, X_lastT+W_same)
            #     schedule_A.append(Vehicle(a[0].id, X_lastT))
            #     a.pop(0)
        elif len(c) > 0:
            schedule_C += simple_compute_entering_time('C', c, W_same, W_diff, Y_lastFrom, Y_lastT)
            c.clear()
            # # c->Y
            # if Y_lastFrom == 'C':
            #     Y_lastT = max(c[0].time, Y_lastT+W_same)
            # else:
            #     Y_lastT = max(c[0].time, Y_lastT+W_diff)
            # Y_lastFrom = 'C'
            # schedule_C.append(Vehicle(c[0].id, Y_lastT))
            # c.pop(0)
            # while len(c) > 0:
            #     Y_lastT = max(c[0].time, Y_lastT+W_same)
            #     schedule_C.append(Vehicle(c[0].id, Y_lastT))
            #     c.pop(0)
        elif len(b) > 0:
            while len(b) > 0:
                if X_lastT < Y_lastT:
                    if X_lastFrom == 'B':
                        X_lastT = max(b[0].time, X_lastT+W_same)
                    else:
                        X_lastT = max(b[0].time, X_lastT+W_diff)
                    X_lastFrom = 'B'
                    schedule_BX.append(Vehicle(b[0].id, X_lastT))
                    b.pop(0)
                # elif Y_lastT < X_lastT:
                else:
                    if Y_lastFrom == 'B':
                        Y_lastT = max(b[0].time, Y_lastT+W_same)
                    else:
                        Y_lastT = max(b[0].time, Y_lastT+W_diff)
                    Y_lastFrom = 'B'
                    schedule_BY.append(Vehicle(b[0].id, Y_lastT))
                    b.pop(0)
                # else: # X_lastT == Y_lastT
                #     if random.randint(0,1) == 0:
                #         if X_lastFrom == 'B':
                #             X_lastT = max(b[0].time, X_lastT+W_same)
                #         else:
                #             X_lastT = max(b[0].time, X_lastT+W_diff)
                #         X_lastFrom = 'B'
                #         schedule_BX.append(Vehicle(b[0].id, X_lastT))
                #         b.pop(0)
                #     else:
                #         if Y_lastFrom == 'B':
                #             Y_lastT = max(b[0].time, Y_lastT+W_same)
                #         else:
                #             Y_lastT = max(b[0].time, Y_lastT+W_diff)
                #         Y_lastFrom = 'B'
                #         schedule_BY.append(Vehicle(b[0].id, Y_lastT))
                #         b.pop(0)
    
    return schedule_A, schedule_BX, schedule_BY, schedule_C


def run(W_same, W_diff):
    # step = 0
    period = 400
    # junction_x = traci.junction.getPosition("gneJ20")[0]
    schedule_A = []
    schedule_BX = []
    schedule_BY = []
    schedule_C = []
    leaveA = False
    leaveBX = False
    leaveBY = False
    leaveC = False
    countdownX = 0
    countdownY = 0
    # endTime = 0
    gA = False
    gBX = False
    gBY = False
    gC = False
    timeStep_cnt = 0
    laneLength = 6000
    passTime_dX = 0
    passTime_dY = 0
    X_lastT = -W_diff   
    Y_lastT = -W_diff


    """execute the TraCI control loop"""
    while traci.simulation.getMinExpectedNumber() > 0:  # The number of vehicles which are in the net plus the ones still waiting to start. 
        for vehID in traci.simulation.getLoadedIDList():
            traci.vehicle.setLaneChangeMode(vehID, 0b000000000000)
        traci.simulationStep()
        timeStep_cnt += 1
        traci.trafficlight.setPhase("TL1",1)
        # print(f'timeSteps: {timeStep_cnt}')
            
        for vehID in traci.inductionloop.getLastStepVehicleIDs("dX"):
            passTime_dX = traci.simulation.getTime()
            if vehID[0] == 'A':
                # print('leaveA', vehID)
                leaveA = True
                for s in schedule_A:
                    if s.id == vehID:
                        schedule_A.remove(s)
                        break
            elif vehID[0] == 'B':
                # print('leaveBX', vehID)
                leaveBX = True
                for s in schedule_BX:
                    if s.id == vehID:
                        schedule_BX.remove(s)
                        break
        for vehID in traci.inductionloop.getLastStepVehicleIDs("dY"):
            passTime_dY = traci.simulation.getTime()
            if vehID[0] == 'C':
                # print('leaveC', vehID)
                leaveC = True
                for s in schedule_C:
                    if s.id == vehID:
                        schedule_C.remove(s)
                        break
            elif vehID[0] == 'B':
                # print('leaveBY', vehID)
                leaveBY = True
                for s in schedule_BY:
                    if s.id == vehID:
                        schedule_BY.remove(s)
                        break

        if (timeStep_cnt - period) == 0:
            if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0 and traci.lanearea.getLastStepVehicleNumber("dC") > 0:
                a_all, b_all, c_all = compute_earliest_arrival(laneLength, schedule_A, schedule_BX, schedule_BY, schedule_C)
                # print('a_all', a_all)
                # print('b_all', b_all)
                # print('c_all', c_all)
                schedule_A, schedule_BX, schedule_BY, schedule_C = fcfs_compute_entering_time(a_all, b_all, c_all, W_same, W_diff, X_lastT, Y_lastT)
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
                        traci.vehicle.setRouteID(veh.id,"route_1")
                    except:
                        # print('-----leaveBY', veh)
                        # print(f'Remove {veh.id} from schedule_BX')
                        Y_lastT = veh.time if Y_lastT < veh.time else Y_lastT
                        schedule_BX.remove(veh)
                        leaveBY = True
                for veh in schedule_BY:
                    try:
                        traci.vehicle.setRouteID(veh.id,"route_2")
                    except:
                        # print('-----leaveBX', veh)
                        # print(f'Remove {veh.id} from schedule_BY')
                        X_lastT = veh.time if X_lastT < veh.time else X_lastT
                        schedule_BY.remove(veh)
                        leaveBX = True
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
                
        if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0 and traci.lanearea.getLastStepVehicleNumber("dC") > 0:
            gA = False
            gBX = False
            gBY = False
            gC = False
            # Control outgoing lane 1
            if len(schedule_A) > 0 and len(schedule_BX) > 0:
                if schedule_A[0].time < schedule_BX[0].time:
                    if leaveA:
                        countdownX = W_same
                    elif leaveBX:
                        countdownX = W_diff
                    elif not countdownX:
                        gA = True
                    # else:
                    #     if countdownX:
                    #         traci.trafficlight.setPhase("TL1", 16)
                    #         countdownX -= 1
                    #     else:
                    #         gA = True
                else:
                    if leaveBX:
                        countdownX = W_same
                    elif leaveA:
                        countdownX = W_diff
                    elif not countdownX:
                        gBX = True
                    # else:
                    #     if countdownX:
                    #         traci.trafficlight.setPhase("TL1", 16)
                    #         countdownX -= 1
                    #     else:
                    #         gBX = True
            elif len(schedule_A) > 0:
                if leaveA:
                    countdownX = W_same
                elif leaveBX:
                    countdownX = W_diff
                elif not countdownX:
                    gA = True
                # else:
                #     if countdownX:
                #         traci.trafficlight.setPhase("TL1", 16)
                #         countdownX -= 1
                #     else:
                #         gA = True
            elif len(schedule_BX) > 0:
                if leaveBX:
                    countdownX = W_same
                elif leaveA:
                    countdownX = W_diff
                elif not countdownX:
                    gBX = True
                # else:
                #     if countdownX:
                #         traci.trafficlight.setPhase("TL1", 16)
                #         countdownX -= 1
                #     else:
                #         gBX = True
            elif not countdownX:
                gA = True
                gBX = True
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
                        countdownY = W_same
                    elif leaveBY:
                        countdownY = W_diff
                    elif not countdownY:
                        gC = True
                    # else:
                    #     if countdownY:
                    #         traci.trafficlight.setPhase("TL1", 18)
                    #         countdownY -= 1
                    #     else:
                    #         gC = True
                else:
                    if leaveBY:
                        countdownY = W_same
                    elif leaveC:
                        countdownY = W_diff
                    elif not countdownY:
                        gBY = True
                    # else:
                    #     if countdownY:
                    #         traci.trafficlight.setPhase("TL1", 18)
                    #         countdownY -= 1
                    #     else:
                    #         gBY = True
            elif len(schedule_C) > 0:
                if leaveC:
                    countdownY = W_same
                elif leaveBY:
                    countdownY = W_diff
                elif not countdownY:
                    gC = True
                # else:
                #     if countdownY:
                #         traci.trafficlight.setPhase("TL1", 18)
                #         countdownY -= 1
                #     else:
                #         gC = True
            elif len(schedule_BY) > 0:
                if leaveBY:
                    countdownY = W_same
                elif leaveC:
                    countdownY = W_diff
                elif not countdownY:
                    gBY = True
                # else:
                #     if countdownY:
                #         traci.trafficlight.setPhase("TL1", 18)
                #         countdownY -= 1
                #     else:
                #         gBY = True
            elif not countdownY:
                gC = True
                gBY = True
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

        elif traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0:
            if (not countdownX) and (not countdownY):
                traci.trafficlight.setPhase("TL1", 2)
            elif not countdownX:
                traci.trafficlight.setPhase("TL1", 8)
            elif not countdownY:
                traci.trafficlight.setPhase("TL1", 12)
            for vehID in traci.lanearea.getLastStepVehicleIDs("dB"):
                try:
                    traci.vehicle.setRouteID(vehID,"route_2")
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
                    traci.vehicle.setRouteID(vehID,"route_1")
                except Exception as e:
                    print(e)
        else:
            if (not countdownX) and (not countdownY):
                traci.trafficlight.setPhase("TL1", 0)
            if traci.lanearea.getLastStepVehicleNumber("dB") > 0:
                id_list = sorted(traci.lanearea.getLastStepVehicleIDs("dB"), key=lambda x: int(x.split('_')[1]))
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


        if countdownX:
            # traci.trafficlight.setPhase("TL1", 16)
            countdownX -= 1
        if countdownY:
            # traci.trafficlight.setPhase("TL1", 18)
            countdownY -= 1

        # print(f'traffic light state: {gA}, {gBX}, {gBY}, {gC}')
        # print(f'phase: {traci.trafficlight.getPhase("TL1")}')
        leaveA = False
        leaveBX = False
        leaveBY = False
        leaveC = False

        # step += 1
        currentTime = traci.simulation.getTime()
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
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


def main():
    options = get_options()
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    try:
        W_same = float(sys.argv[1]) # the waiting time if two consecutive vehicles are from the same lane
        W_diff = float(sys.argv[2])  # the waiting time if two consecutive vehicles are from different lanes
    except:
        print('Arguments: W=, W+')
        return
    
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "sumo_data/laneMerging.sumocfg",
                            "--tripinfo-output", "tripinfo_dp.xml",
                            "-S",
                            "--no-step-log", "true", "-W", "--duration-log.disable", "true"])
    run(W_same, W_diff)


if __name__ == "__main__":
    main()
