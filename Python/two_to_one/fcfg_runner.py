#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import numpy as np

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

def compute_earliest_arrival(junction_x, schedule_A, schedule_B):
    a = np.array([0])
    b = np.array([0])
    id_a = np.array([''])
    id_b = np.array([''])
    currentTime = traci.simulation.getTime()
    # Vehicles in communication range on lane A
    for vehID in traci.lanearea.getLastStepVehicleIDs("dA"):
        dist = junction_x - traci.vehicle.getPosition(vehID)[0]
        speed = traci.vehicle.getSpeed(vehID)
        arrivalTime = 0
        if speed == 0:
            for i in schedule_A:
                if i[0] == vehID:
                    arrivalTime = i[1]
        elif speed != 0:
            arrivalTime = currentTime + (dist/speed)
            for i in schedule_A:
                if i[0] == vehID:
                    if i[1] < arrivalTime:
                        arrivalTime = i[1]
                    break
        a = np.append(a, arrivalTime)
        id_a = np.append(id_a, vehID)

    # Vehicles in communication range on lane B
    for vehID in traci.lanearea.getLastStepVehicleIDs("dB"):
        dist = junction_x - traci.vehicle.getPosition(vehID)[0]
        speed = traci.vehicle.getSpeed(vehID)
        arrivalTime = 0
        if speed == 0:
            for i in schedule_B:
                if i[0] == vehID:
                    arrivalTime = i[1]
        elif speed != 0:
            arrivalTime = currentTime + (dist/speed)
            for i in schedule_B:
                if i[0] == vehID:
                    if i[1] < arrivalTime:
                        arrivalTime = i[1]
                    break
        b = np.append(b, arrivalTime)
        id_b = np.append(id_b, vehID)

    return a, b, id_a, id_b
    

def run_scheduled():
    W_same = 1  # the waiting time if two consecutive vehicles are from the same lane
    W_diff = 2  # the waiting time if two consecutive vehicles are from different lanes
    step = 0
    period = 4
    junction_x = traci.junction.getPosition("gneJ1")[0]
    schedule_A = []
    schedule_B = []
    leaveA = False
    leaveB = False
    countdown = 0
    endTime = 0

    """execute the TraCI control loop"""
    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("0", 2)
    while traci.simulation.getMinExpectedNumber() > 0:  # The number of vehicles which are in the net plus the ones still waiting to start. 
        if step == period:
            if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0:
                a, b, id_a, id_b = compute_earliest_arrival(junction_x, schedule_A, schedule_B)
                schedule_A = []
                schedule_B = []
                for i in range(1, len(a)):
                    schedule_A.append([id_a[i], a[i]])
                schedule_A.sort(key=lambda x: x[1])
                for i in range(1, len(b)):
                    schedule_B.append([id_b[i], b[i]])
                schedule_B.sort(key=lambda x: x[1])
            step = 0

        for vehID in traci.simulation.getLoadedIDList():
            traci.vehicle.setLaneChangeMode(vehID, 0b000000000000)
        traci.simulationStep()
        step += 1
        currentTime = traci.simulation.getTime()
        endTime = currentTime
        # print(currentTime)
        if len(schedule_A) > 0 and traci.vehicle.getPosition(schedule_A[0][0])[0] >= junction_x:
            schedule_A.remove(schedule_A[0])
            leaveA = True
        elif len(schedule_B) > 0 and traci.vehicle.getPosition(schedule_B[0][0])[0] >= junction_x:
            schedule_B.remove(schedule_B[0])
            leaveB = True
            
        if len(schedule_A) > 0 and len(schedule_B) > 0:
            if schedule_A[0][1] < schedule_B[0][1]:
                if leaveA:
                    countdown = W_same - 1
                elif leaveB:
                    countdown = W_diff - 1
                else:
                    if countdown:
                        traci.trafficlight.setPhase("TL1", 1)
                        countdown -= 1
                    else:
                        traci.trafficlight.setPhase("TL1", 2)
            else:
                if leaveB:
                    countdown = W_same - 1
                elif leaveA:
                    countdown = W_diff - 1
                else:
                    if countdown:
                        traci.trafficlight.setPhase("TL1", 1)
                        countdown -= 1
                    else:
                        traci.trafficlight.setPhase("TL1", 4)
        elif len(schedule_A) > 0:
            if leaveA:
                    countdown = W_same - 1
            elif leaveB:
                countdown = W_diff - 1
            else:
                if countdown:
                    traci.trafficlight.setPhase("TL1", 1)
                    countdown -= 1
                else:
                    traci.trafficlight.setPhase("TL1", 2)
        elif len(schedule_B) > 0:
            if leaveB:
                    countdown = W_same - 1
            elif leaveA:
                countdown = W_diff - 1
            else:
                if countdown:
                    traci.trafficlight.setPhase("TL1", 1)
                    countdown -= 1
                else:
                    traci.trafficlight.setPhase("TL1", 4)
        else:
            if countdown:
                traci.trafficlight.setPhase("TL1", 1)
                countdown -= 1
            else:
                traci.trafficlight.setPhase("TL1", 0)
        leaveA = False
        leaveB = False


    # print(endTime)
    traci.close()
    sys.stdout.flush()

    
def run_noSchedule():
    W_same = 1  # the waiting time if two consecutive vehicles are from the same lane
    W_diff = 3  # the waiting time if two consecutive vehicles are from different lanes
    junction_x = traci.junction.getPosition("gneJ1")[0]
    leaveA = False
    leaveB = False
    countdown = 0
    lastPass = ''
    endTime = 0

    """execute the TraCI control loop"""
    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("0", 2)
    while traci.simulation.getMinExpectedNumber() > 0:  # The number of vehicles which are in the net plus the ones still waiting to start. 
        for vehID in traci.simulation.getLoadedIDList():
            traci.vehicle.setLaneChangeMode(vehID, 0b000000000000)
        traci.simulationStep()
        currentTime = traci.simulation.getTime()
        endTime = currentTime
        # print(currentTime)

        numA = traci.lanearea.getLastStepVehicleNumber("dA")
        numB = traci.lanearea.getLastStepVehicleNumber("dB")
        # print("Pass", traci.lane.getLastStepVehicleIDs("E2_0"))
        if numA > 0 and numB > 0:
            if traci.lane.getLastStepVehicleNumber("E2_0") > 0:
                if lastPass != traci.lane.getLastStepVehicleIDs("E2_0")[0]:
                    lastPass = traci.lane.getLastStepVehicleIDs("E2_0")[0]
                    if lastPass.strip('_')[0] == 'A':  # From lane A
                        leaveA = True
                    else:   # From lane B
                        leaveB = True
            firstA = traci.lanearea.getLastStepVehicleIDs("dA")[0]
            firstB = traci.lanearea.getLastStepVehicleIDs("dB")[0]
            if traci.vehicle.getPosition(firstA)[0] >= traci.vehicle.getPosition(firstB)[0]:
                if leaveA:
                    countdown = W_same - 1
                elif leaveB:
                    countdown = W_diff - 1
                else:
                    if countdown:
                        traci.trafficlight.setPhase("TL1", 1)
                        countdown -= 1
                    else:
                        traci.trafficlight.setPhase("TL1", 2)
            else:
                if leaveB:
                    countdown = W_same - 1
                elif leaveA:
                    countdown = W_diff - 1
                else:
                    if countdown:
                        traci.trafficlight.setPhase("TL1", 1)
                        countdown -= 1
                    else:
                        traci.trafficlight.setPhase("TL1", 4)
        elif numA > 0:
            if leaveA:
                    countdown = W_same - 1
            elif leaveB:
                countdown = W_diff - 1
            else:
                if countdown:
                    traci.trafficlight.setPhase("TL1", 1)
                    countdown -= 1
                else:
                    traci.trafficlight.setPhase("TL1", 2)
        elif numB > 0:
            if leaveB:
                countdown = W_same - 1
            elif leaveA:
                countdown = W_diff - 1
            else:
                if countdown:
                    traci.trafficlight.setPhase("TL1", 1)
                    countdown -= 1
                else:
                    traci.trafficlight.setPhase("TL1", 4)
        else:
            if countdown:
                traci.trafficlight.setPhase("TL1", 1)
                countdown -= 1
            else:
                traci.trafficlight.setPhase("TL1", 0)

        leaveA = False
        leaveB = False

    # print(endTime)
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "laneMerging.sumocfg",
                             "--tripinfo-output", "tripinfo_fcfg.xml",
                             "-S",
                             "--no-step-log", "true", "-W", "--duration-log.disable", "true"])
    run_scheduled()