# Run: python3 runner_fcfs.py [density] [number] [W=] [W+] [generateNewTest (T/F)] [inputPath]
from __future__ import absolute_import
from __future__ import print_function
from dis import dis

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


def generate_routefile(timeStep, N, pA, pB, pC):
    # random.seed(42)  # make tests reproducible
    with open("./sumo_data/laneMerging.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeA" type="passenger" length="4" accel="2.6" decel="4.5" sigma="0.0" maxSpeed="20" color="yellow"/>
        <vType id="typeB" type="passenger" length="4" accel="2.6" decel="4.5" sigma="0.0" maxSpeed="20" color="blue"/>
        <vType id="typeC" type="passenger" length="4" accel="2.6" decel="4.5" sigma="0.0" maxSpeed="20" color="magenta"/>

        <route edges="A X" color="yellow" id="route_0"/>
        <route edges="B X" color="yellow" id="route_1"/>
        <route edges="B Y" color="yellow" id="route_2"/>
        <route edges="C Y" color="yellow" id="route_3"/>""", file=routes)
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
        accel = traci.vehicle.getAcceleration(vehID)
        arrivalTime = 0
        if speed == 0:
            for i in schedule_A:
                if i.id == vehID:
                    arrivalTime = i.time
                    break
        else:
            if accel != 0:
                arrivalTime = currentTime + \
                    (((max(0, speed ** 2 + 2 * accel * dist)) ** 0.5 - speed) / accel)
            else:
                arrivalTime = currentTime + dist / speed
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
        accel = traci.vehicle.getAcceleration(vehID)
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
        else:
            if accel != 0:
                arrivalTime = currentTime + \
                    (((max(0, speed ** 2 + 2 * accel * dist)) ** 0.5 - speed) / accel)
            else:
                arrivalTime = currentTime + dist / speed
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

    # Vehicles in communication range on lane C
    for vehID in traci.lanearea.getLastStepVehicleIDs("dC"):
        dist = laneLength - traci.vehicle.getDistance(vehID)
        speed = traci.vehicle.getSpeed(vehID)
        accel = traci.vehicle.getAcceleration(vehID)
        arrivalTime = 0
        if speed == 0:
            for i in schedule_C:
                if i.id == vehID:
                    arrivalTime = i.time
        else:
            if accel != 0:
                arrivalTime = currentTime + \
                    (((max(0, speed ** 2 + 2 * accel * dist)) ** 0.5 - speed) / accel)
            else:
                arrivalTime = currentTime + dist / speed
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


def simple_compute_entering_time(lane, traffic, W_same, W_diff, prevLane, prevTime):
    schedule = []
    if prevLane == '':
        schedule.append(Vehicle(traffic[0].id, traffic[0].time))
    elif lane == prevLane:
        schedule.append(Vehicle(traffic[0].id, max(traffic[0].time, prevTime+W_same)))
    else:
        schedule.append(Vehicle(traffic[0].id, max(traffic[0].time, prevTime+W_diff)))
    for i in range(1, len(traffic)):
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
            if first == b[0].time:
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
                else:  # X_lastT == Y_lastT
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
                        if random.randint(0, 1) == 0:
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
            schedule_A += simple_compute_entering_time(
                'A', a, W_same, W_diff, X_lastFrom, X_lastT)
            schedule_BY += simple_compute_entering_time(
                'B', b, W_same, W_diff, Y_lastFrom, Y_lastT)
            a.clear()
            b.clear()
        elif len(b) > 0 and len(c) > 0:
            schedule_BX += simple_compute_entering_time(
                'B', b, W_same, W_diff, X_lastFrom, X_lastT)
            schedule_C += simple_compute_entering_time(
                'C', c, W_same, W_diff, Y_lastFrom, Y_lastT)
            b.clear()
            c.clear()
        elif len(a) > 0 and len(c) > 0:
            schedule_A += simple_compute_entering_time(
                'A', a, W_same, W_diff, X_lastFrom, X_lastT)
            schedule_C += simple_compute_entering_time(
                'C', c, W_same, W_diff, Y_lastFrom, Y_lastT)
            a.clear()
            c.clear()
        elif len(a) > 0:
            schedule_A += simple_compute_entering_time(
                'A', a, W_same, W_diff, X_lastFrom, X_lastT)
            a.clear()
        elif len(c) > 0:
            schedule_C += simple_compute_entering_time(
                'C', c, W_same, W_diff, Y_lastFrom, Y_lastT)
            c.clear()
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

    return schedule_A, schedule_BX, schedule_BY, schedule_C


def run(alpha, beta, gamma, W_same, W_diff):
    # step = 0
    period = 300
    global a_all, b_all, c_all
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
    laneLength = 6500
    passTime_dX = 0
    passTime_dY = 0
    X_lastT = -W_diff
    Y_lastT = -W_diff
    A_IDs = []
    B_IDs = []
    C_IDs = []
    A_head = "A_1"
    B_head = "B_1"
    C_head = "C_1"
    waitingTime = 0

    """execute the TraCI control loop"""
    while traci.simulation.getMinExpectedNumber() > 0:  # The number of vehicles which are in the net plus the ones still waiting to start.
        for vehID in traci.simulation.getLoadedIDList():
            traci.vehicle.setLaneChangeMode(vehID, 0b000000000000)
        traci.simulationStep()
        timeStep_cnt += 1
        traci.trafficlight.setPhase("TL1", 1)
        # print(f'timeSteps: {timeStep_cnt}')

        leaveA = False
        leaveBX = False
        leaveBY = False
        leaveC = False

        # Detect the passing vehicles
        A_IDs = traci.edge.getLastStepVehicleIDs("A")
        B_IDs = traci.edge.getLastStepVehicleIDs("B")
        C_IDs = traci.edge.getLastStepVehicleIDs("C")
        A_IDs = sorted(A_IDs, key=lambda x: int(x.split('_')[1]))
        B_IDs = sorted(B_IDs, key=lambda x: int(x.split('_')[1]))
        C_IDs = sorted(C_IDs, key=lambda x: int(x.split('_')[1]))
        if len(A_IDs) > 0 and A_IDs[0] != A_head:
            # print(A_head, "leaves")
            waitingTime += traci.vehicle.getWaitingTime(A_head)
            passTime_dX = traci.simulation.getTime()
            leaveA = True
            for s in schedule_A:
                if s.id == A_head:
                    schedule_A.remove(s)
                    break
            for s in a_all:
                if s.id == A_head:
                    waitingTime += passTime_dX - s.time
                    break
            A_head = A_IDs[0]
        elif len(A_IDs) == 0 and len(A_head) > 0:
            if int(A_head.split('_')[1]) == alpha:
                # print(A_head, "leaves")
                waitingTime += traci.vehicle.getWaitingTime(A_head)
                passTime_dX = traci.simulation.getTime()
                leaveA = True
                for s in schedule_A:
                    if s.id == A_head:
                        schedule_A.remove(s)
                        break
                for s in a_all:
                    if s.id == A_head:
                        waitingTime += passTime_dX - s.time
                        break
                A_head = ""
        if len(B_IDs) > 0 and B_IDs[0] != B_head:
            # print(B_head, "leaves")
            waitingTime += traci.vehicle.getWaitingTime(B_head)
            isFound = False
            for s in schedule_BX:
                if s.id == B_head:
                    passTime_dX = traci.simulation.getTime()
                    leaveBX = True
                    schedule_BX.remove(s)
                    isFound = True
                    for s in b_all:
                        if s.id == B_head:
                            waitingTime += passTime_dX - s.time
                            break
                    break
            if not isFound:
                for s in schedule_BY:
                    if s.id == B_head:
                        passTime_dY = traci.simulation.getTime()
                        leaveBY = True
                        schedule_BY.remove(s)
                        for s in b_all:
                            if s.id == B_head:
                                waitingTime += passTime_dY - s.time
                                break
                        break
            B_head = B_IDs[0]
        elif len(B_IDs) == 0 and len(B_head) > 0:
            if int(B_head.split('_')[1]) == beta:
                # print(B_head, "leaves")
                waitingTime += traci.vehicle.getWaitingTime(B_head)
                isFound = False
                for s in schedule_BX:
                    if s.id == B_head:
                        passTime_dX = traci.simulation.getTime()
                        leaveBX = True
                        schedule_BX.remove(s)
                        isFound = True
                        for s in b_all:
                            if s.id == B_head:
                                waitingTime += passTime_dX - s.time
                                break
                        break
                if not isFound:
                    for s in schedule_BY:
                        if s.id == B_head:
                            passTime_dY = traci.simulation.getTime()
                            leaveBY = True
                            schedule_BY.remove(s)
                            for s in b_all:
                                if s.id == B_head:
                                    waitingTime += passTime_dY - s.time
                                    break
                            break
                B_head = ""
        if len(C_IDs) > 0 and C_IDs[0] != C_head:
            # print(C_head, "leaves")
            waitingTime += traci.vehicle.getWaitingTime(C_head)
            passTime_dY = traci.simulation.getTime()
            leaveC = True
            for s in schedule_C:
                if s.id == C_head:
                    schedule_C.remove(s)
                    break
            for s in c_all:
                if s.id == C_head:
                    waitingTime += passTime_dY - s.time
                    break
            C_head = C_IDs[0]
        elif len(C_IDs) == 0 and len(C_head) > 0:
            if int(C_head.split('_')[1]) == gamma:
                # print(C_head, "leaves")
                waitingTime += traci.vehicle.getWaitingTime(C_head)
                passTime_dY = traci.simulation.getTime()
                leaveC = True
                for s in schedule_C:
                    if s.id == C_head:
                        schedule_C.remove(s)
                        break
                for s in c_all:
                    if s.id == C_head:
                        waitingTime += passTime_dY - s.time
                        break
                C_head = ""

        if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0 and traci.lanearea.getLastStepVehicleNumber("dC") > 0:
            gA = False
            gBX = False
            gBY = False
            gC = False
            # Control outgoing lane 1
            if len(schedule_A) > 0 and len(schedule_BX) > 0:
                if schedule_A[0].time < schedule_BX[0].time:
                    if leaveA:
                        countdownX = W_same - 1
                    elif leaveBX:
                        countdownX = W_diff - 1
                    if not countdownX:
                        gA = True
                else:
                    if leaveBX:
                        countdownX = W_same - 1
                    elif leaveA:
                        countdownX = W_diff - 1
                    if not countdownX:
                        gBX = True
            elif len(schedule_A) > 0:
                if leaveA:
                    countdownX = W_same - 1
                elif leaveBX:
                    countdownX = W_diff - 1
                if not countdownX:
                    gA = True
            elif len(schedule_BX) > 0:
                if leaveBX:
                    countdownX = W_same - 1
                elif leaveA:
                    countdownX = W_diff - 1
                if not countdownX:
                    gBX = True
            elif not countdownX:
                gA = True
                gBX = True

            # Control outgoing lane 2
            if len(schedule_C) > 0 and len(schedule_BY) > 0:
                if schedule_C[0].time < schedule_BY[0].time:
                    if leaveC:
                        countdownY = W_same - 1
                    elif leaveBY:
                        countdownY = W_diff - 1
                    if not countdownY:
                        gC = True
                else:
                    if leaveBY:
                        countdownY = W_same - 1
                    elif leaveC:
                        countdownY = W_diff - 1
                    if not countdownY:
                        gBY = True
            elif len(schedule_C) > 0:
                if leaveC:
                    countdownY = W_same - 1
                elif leaveBY:
                    countdownY = W_diff - 1
                if not countdownY:
                    gC = True
            elif len(schedule_BY) > 0:
                if leaveBY:
                    countdownY = W_same - 1
                elif leaveC:
                    countdownY = W_diff - 1
                if not countdownY:
                    gBY = True
            elif not countdownY:
                gC = True
                gBY = True

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
                id_list = sorted(traci.lanearea.getLastStepVehicleIDs(
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

        if countdownX:
            # traci.trafficlight.setPhase("TL1", 16)
            countdownX -= 1
        if countdownY:
            # traci.trafficlight.setPhase("TL1", 18)
            countdownY -= 1

        # Schedule
        if (timeStep_cnt - period) == 0:
            if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0 and traci.lanearea.getLastStepVehicleNumber("dC") > 0:
                a_all, b_all, c_all = compute_earliest_arrival(
                    laneLength, schedule_A, schedule_BX, schedule_BY, schedule_C)
                # print('a_all', a_all)
                # print('b_all', b_all)
                # print('c_all', c_all)
                schedule_A, schedule_BX, schedule_BY, schedule_C = fcfs_compute_entering_time(
                    a_all, b_all, c_all, W_same, W_diff, X_lastT, Y_lastT)
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
                        # print('-----leaveBY', veh)
                        # print(f'Remove {veh.id} from schedule_BX')
                        Y_lastT = veh.time if Y_lastT < veh.time else Y_lastT
                        schedule_BX.remove(veh)
                        leaveBY = True
                for veh in schedule_BY:
                    try:
                        traci.vehicle.setRouteID(veh.id, "route_2")
                    except:
                        # print('-----leaveBX', veh)
                        # print(f'Remove {veh.id} from schedule_BY')
                        X_lastT = veh.time if X_lastT < veh.time else X_lastT
                        schedule_BY.remove(veh)
                        leaveBX = True
            # step = 0

        # print(f'traffic light state: {gA}, {gBX}, {gBY}, {gC}')
        # print(f'phase: {traci.trafficlight.getPhase("TL1")}')

        # step += 1
        currentTime = traci.simulation.getTime()
        # endTime = currentTime
        # print(currentTime)

    # print(endTime)
    print(max(passTime_dX, passTime_dY), waitingTime / (alpha + beta + gamma))

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
        p = float(sys.argv[1])  # lambda for Poisson distribution
        N = int(sys.argv[2])  # Number of vehicles in each lane
        alpha = N
        beta = N
        gamma = N
        # the waiting time if two consecutive vehicles are from the same lane
        W_same = float(sys.argv[3])
        # the waiting time if two consecutive vehicles are from different lanes
        W_diff = float(sys.argv[4])
        isNewTest = sys.argv[5]
    except:
        print('Arguments: lambda, N, W=, W+, isNewTest')
        return

    timeStep = 1    # The precision of time (in second)
    pA = p
    pB = p
    pC = p

    if len(sys.argv) > 6:
        inputPath = sys.argv[6]
        outputPath = "output/output" + inputPath[inputPath.find("_"):-8] + "_FCFS.xml"
        traci.start([sumoBinary, "-c", inputPath,
                     "--tripinfo-output", outputPath,
                     "-S",
                     "--no-step-log", "true", "-W", "--duration-log.disable", "true"])
    else:
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

    run(alpha, beta, gamma, W_same, W_diff)


if __name__ == "__main__":
    main()
