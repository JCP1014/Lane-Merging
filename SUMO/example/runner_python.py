#!/usr/bin/env python
# old version of dp
from __future__ import absolute_import
from __future__ import print_function
from http.client import NotConnected

import os
import sys
import optparse
import random
from tkinter.messagebox import NO
import numpy as np
from collections import namedtuple

Vehicle = namedtuple("Vehicle", "id time")

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci


def generate_routefile(timeStep, N, pA, pB, pC):
    # random.seed(42)  # make tests reproducible
    with open("./sumo_data/laneMerging.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeA" type="passenger" length="4.5" accel="2.6" decel="4.5" sigma="0.0" maxSpeed="25" color="yellow"/>
        <vType id="typeB" type="passenger" length="4.5" accel="2.6" decel="4.5" sigma="0.0" maxSpeed="25" color="blue"/>
        <vType id="typeC" type="passenger" length="4.5" accel="2.6" decel="4.5" sigma="0.0" maxSpeed="25" color="magenta"/>

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


def run(alpha, beta, gamma, W_same, W_diff):
    period = 20
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
    gA = False
    gBX = False
    gBY = False
    gC = False
    timeStep_cnt = 0
    passTime_dX = 0
    passTime_dY = 0
    A_IDs = []
    B_IDs = []
    C_IDs = []
    A_head = "A_1"
    B_head = "B_1"
    C_head = "C_1"

    """execute the TraCI control loop"""
    while traci.simulation.getMinExpectedNumber() > 0:  # The number of vehicles which are in the net plus the ones still waiting to start.
        # Disable lane changing
        for vehID in traci.simulation.getLoadedIDList():
            traci.vehicle.setLaneChangeMode(vehID, 0b000000000000)
        # Forward
        traci.simulationStep()
        timeStep_cnt += 1
        # Initially set all traffic lights to green
        traci.trafficlight.setPhase("TL1", 1)

        leaveA = False  # Whether a vehicle left from lane A at the last step
        leaveBX = False  # Whether a vehicle left from lane B and go to lane X at the last step
        leaveBY = False  # Whether a vehicle left from lane B and go to lane Y at the last step
        leaveC = False  # Whether a vehicle left from lane C at the last step

        ''' Schedule '''
        if timeStep_cnt % period == 0:  # Every period
            # If all the three lanes are not empty
            if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0 and traci.lanearea.getLastStepVehicleNumber("dC") > 0:
                a, b, c = compute_earliest_arrival()
                schedule_A, schedule_BX, schedule_BY, schedule_C = schedule(
                    a, b, c)
                # Sort the schedule by the entering time
                schedule_A.sort(key=lambda x: x[1])
                schedule_BX.sort(key=lambda x: x[1])
                schedule_BY.sort(key=lambda x: x[1])
                schedule_C.sort(key=lambda x: x[1])
                # Set the route for vehicles on lane B
                for veh in schedule_BX:
                    try:
                        traci.vehicle.setRouteID(veh.id, "route_1")
                    except:  # Too late to set the route
                        # print('leaveBY', veh)
                        # print(f'Remove {veh.id} from schedule_BX')
                        schedule_BX.remove(veh)
                        leaveBY = True
                for veh in schedule_BY:
                    try:
                        traci.vehicle.setRouteID(veh.id, "route_2")
                    except:
                        # print('leaveBX', veh)
                        # print(f'Remove {veh.id} from schedule_BY')
                        schedule_BY.remove(veh)
                        leaveBX = True

        ''' Detect the passing vehicles '''
        # Get the ID list of the vehicles on each lane
        A_IDs = traci.edge.getLastStepVehicleIDs("A")
        B_IDs = traci.edge.getLastStepVehicleIDs("B")
        C_IDs = traci.edge.getLastStepVehicleIDs("C")
        # Sort the list by the ID number
        A_IDs = sorted(A_IDs, key=lambda x: int(x.split('_')[1]))
        B_IDs = sorted(B_IDs, key=lambda x: int(x.split('_')[1]))
        C_IDs = sorted(C_IDs, key=lambda x: int(x.split('_')[1]))
        # A vehicle left from lane A at the last step
        if len(A_IDs) > 0 and A_IDs[0] != A_head:
            # print(A_head, "leaves")
            passTime_dX = traci.simulation.getTime() - 1    # Update the last passing time
            leaveA = True
            for s in schedule_A:    # Remove it from the schedule
                if s.id == A_head:
                    schedule_A.remove(s)
                    break
            A_head = A_IDs[0]   # Update the first vehicle on the lane
        elif len(A_IDs) == 0 and len(A_head) > 0:
            # The last vehicle left from lane A at the last step
            if int(A_head.split('_')[1]) == alpha:
                # print(A_head, "leaves")
                passTime_dX = traci.simulation.getTime() - 1
                leaveA = True
                for s in schedule_A:
                    if s.id == A_head:
                        schedule_A.remove(s)
                        break
                A_head = ""
        if len(B_IDs) > 0 and B_IDs[0] != B_head:
            # print(B_head, "leaves")
            isFound = False  # Find the vehicle is scheduled to lane X or lane Y
            for s in schedule_BX:
                if s.id == B_head:
                    passTime_dX = traci.simulation.getTime()
                    leaveBX = True
                    schedule_BX.remove(s)
                    isFound = True
                    break
            if not isFound:
                for s in schedule_BY:
                    if s.id == B_head:
                        passTime_dY = traci.simulation.getTime()
                        leaveBY = True
                        schedule_BY.remove(s)
                        break
            B_head = B_IDs[0]
        elif len(B_IDs) == 0 and len(B_head) > 0:
            if int(B_head.split('_')[1]) == beta:
                # print(B_head, "leaves")
                isFound = False
                for s in schedule_BX:
                    if s.id == B_head:
                        passTime_dX = traci.simulation.getTime()
                        leaveBX = True
                        schedule_BX.remove(s)
                        isFound = True
                        break
                if not isFound:
                    for s in schedule_BY:
                        if s.id == B_head:
                            passTime_dY = traci.simulation.getTime()
                            leaveBY = True
                            schedule_BY.remove(s)
                            break
                B_head = ""
        if len(C_IDs) > 0 and C_IDs[0] != C_head:
            # print(C_head, "leaves")
            passTime_dY = traci.simulation.getTime()
            leaveC = True
            for s in schedule_C:
                if s.id == C_head:
                    schedule_C.remove(s)
                    break
            C_head = C_IDs[0]
        elif len(C_IDs) == 0 and len(C_head) > 0:
            if int(C_head.split('_')[1]) == gamma:
                # print(C_head, "leaves")
                passTime_dY = traci.simulation.getTime()
                leaveC = True
                for s in schedule_C:
                    if s.id == C_head:
                        schedule_C.remove(s)
                        break
                C_head = ""

        ''' Decide the phase of traffic lights '''
        # If there is at least one vehicle on an incoming lane
        if traci.lanearea.getLastStepVehicleNumber("dA") > 0 or traci.lanearea.getLastStepVehicleNumber("dB") > 0 or traci.lanearea.getLastStepVehicleNumber("dC") > 0:
            gA = False  # Whether to set the traffic light to green for lane A
            gBX = False  # Whether to set the traffic light to green for lane B to lane X
            gBY = False  # Whether to set the traffic light to green for lane B to lane Y
            gC = False  # Whether to set the traffic light to green for lane C
            ''' Control outgoing lane X '''
            if len(schedule_A) > 0 and len(schedule_BX) > 0:
                # Let the vehicle on lane A go first
                if schedule_A[0].time < schedule_BX[0].time:
                    if leaveA:  # If a vehicle on lane A left at last step
                        countdownX = W_same  # Wait for W=
                    elif leaveBX:  # If a vehicle on lane B left at last step
                        countdownX = W_diff  # Wait for W+
                    elif not countdownX:    # If there is no need to wait or the waiting time is over
                        gA = True   # We can set the traffic light to green
                else:  # Let the vehicle on lane B go first
                    if leaveBX:
                        countdownX = W_same
                    elif leaveA:
                        countdownX = W_diff
                    elif not countdownX:
                        gBX = True
            elif len(schedule_A) > 0:
                if leaveA:
                    countdownX = W_same
                elif leaveBX:
                    countdownX = W_diff
                elif not countdownX:
                    gA = True
            elif len(schedule_BX) > 0:
                if leaveBX:
                    countdownX = W_same
                elif leaveA:
                    countdownX = W_diff
                elif not countdownX:
                    gBX = True
            elif not countdownX:
                gA = True
                gBX = True

            ''' Control outgoing lane Y '''
            if len(schedule_C) > 0 and len(schedule_BY) > 0:
                if schedule_C[0].time < schedule_BY[0].time:
                    if leaveC:
                        countdownY = W_same
                    elif leaveBY:
                        countdownY = W_diff
                    elif not countdownY:
                        gC = True
                else:
                    if leaveBY:
                        countdownY = W_same
                    elif leaveC:
                        countdownY = W_diff
                    elif not countdownY:
                        gBY = True
            elif len(schedule_C) > 0:
                if leaveC:
                    countdownY = W_same
                elif leaveBY:
                    countdownY = W_diff
                elif not countdownY:
                    gC = True
            elif len(schedule_BY) > 0:
                if leaveBY:
                    countdownY = W_same
                elif leaveC:
                    countdownY = W_diff
                elif not countdownY:
                    gBY = True
            elif not countdownY:
                gC = True
                gBY = True

            # Set the traffic lights according to the boolean variables
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
        # Subtract the waiting time
        if countdownX:
            countdownX -= 1
        if countdownY:
            countdownY -= 1

    # The last passing time
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

    # Randomly generate the traffics
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
