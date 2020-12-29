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

from sumolib import checkBinary  # noqa
import traci  # noqa

def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 600  # number of time steps
    # demand per second from different directions
    pA = 1. / 15 # a vehicle is generated every 30 seconds in average.
    pB = 1. / 20 # a vehicle is generated every 30 seconds in average.
    with open("laneMerging.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeA" type="passenger" length="5" accel="10" decel="10" sigma="0.0" maxSpeed="50" color="yellow"/>
        <vType id="typeB" type="passenger" length="5" accel="10" decel="10" sigma="0.0" maxSpeed="50" color="blue"/>

        <route id="right" edges="E1 E2" />""", file=routes)
        num_A = 1
        num_B = 1
        for i in range(N):
            if random.uniform(0, 1) < pA:
                print('    <vehicle id="A_%i" type="typeA" route="right" depart="%i" departLane="1" departSpeed="random"/>' % (
                    num_A, i), file=routes)
                num_A += 1
            if random.uniform(0, 1) < pB:
                print('    <vehicle id="B_%i" type="typeB" route="right" depart="%i" departLane="0" departSpeed="random"/>' % (
                    num_B, i), file=routes)
                num_B += 1
        print("</routes>", file=routes)


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
        b = np.append(b, arrivalTime)
        id_b = np.append(id_b, vehID)

    return a, b, id_a, id_b


def compute_entering_time(a, b):
    W_same = 1  # the waiting time if two consecutive vehicles are from the same lane
    W_diff = 3  # the waiting time if two consecutive vehicles are from different lanes
    alpha = len(a) - 1
    beta = len(b) - 1
    L = np.zeros((alpha+1, beta+1, 2))  # dp table

    # Initialize
    L[0][0][0] = 0
    L[0][0][1] = 0
    L[1][0][0] = a[1]
    L[0][1][1] = b[1]
    for i in range(2, alpha+1):
        L[i][0][0] = max(a[i], L[i-1][0][0]+W_same)
    for j in range(2, beta+1):
        L[0][j][1] = max(b[j], L[0][j-1][1]+W_same)
    for j in range(1, beta+1):
        L[0][j][0] = L[0][j][1] + W_diff
    for i in range(1, alpha+1):
        L[i][0][1] = L[i][0][0] + W_diff
    
    # Compute table
    for i in range(1, alpha+1):
        for j in range(1, beta+1):
            L[i][j][0] = min( max(a[i], L[i-1][j][0]+W_same), max(a[i], L[i-1][j][1]+W_diff) )
            L[i][j][1] = min( max(b[j], L[i][j-1][0]+W_diff), max(b[j], L[i][j-1][1]+W_same) )

    # Choose optimal solution
    order_stack_A = []
    order_stack_B = []
    i = alpha
    j = beta
    while i>0 or j>0:
        if L[i][j][1] < L[i][j][0]:
            order_stack_B.append(('B', j, L[i][j][1]))
            j -= 1
        else:
            order_stack_A.append(('A', i, L[i][j][0]))
            i -= 1

    # Output order
    # while len(order_stack) > 0:
    #     print(order_stack.pop())

    return order_stack_A, order_stack_B


def print_table(L):
    for k in range(L.shape[2]):
        for i in range(L.shape[0]):
            for j in range(L.shape[1]):
                print(L[i][j][k], end=' ')
            print('')
        print('')


def run():
    step = 0
    period = 10
    junction_x = traci.junction.getPosition("gneJ1")[0]
    offset = 5
    schedule_A = []
    schedule_B = []
    """execute the TraCI control loop"""
    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("0", 2)
    while traci.simulation.getMinExpectedNumber() > 0:  # The number of vehicles which are in the net plus the ones still waiting to start. 
        if traci.simulation.getLoadedNumber() > 0:
            for vehID in traci.simulation.getLoadedIDList():
                traci.vehicle.setLaneChangeMode(vehID, 0b000000000000)
        traci.simulationStep()
        step += 1
        currentTime = traci.simulation.getTime()
        print(currentTime)
        if len(schedule_A) > 0 and schedule_A[0][2] == False:
            t = schedule_A[0][1]
            d = t-traci.simulation.getTime()
            print('stop', schedule_A[0][0], )
            try:
                traci.vehicle.setStop(schedule_A[0][0], "E1", pos=junction_x, laneIndex=1, duration=0, until=t)
                schedule_A[0][2] = True
            except traci.exceptions.TraCIException:
                pass
        if len(schedule_B) > 0 and schedule_B[0][2] == False:
            t = schedule_B[0][1]
            d = t-traci.simulation.getTime()
            print('stop', schedule_B[0][0], t)
            try:
                traci.vehicle.setStop(schedule_B[0][0], "E1", pos=junction_x, laneIndex=0, duration=0, until=t)
                schedule_B[0][2] = True
            except traci.exceptions.TraCIException:
                pass

        # if len(schedule_A) > 0:
        for i in schedule_A:
            if i[1] < traci.simulation.getTime():
                schedule_A.remove(i)
        
        # if len(schedule_B) > 0:
        for i in schedule_B:
            if i[1] < traci.simulation.getTime():
                schedule_B.remove(i)

        if step == period:
            if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0:
                a, b, id_a, id_b = compute_earliest_arrival(junction_x, schedule_A, schedule_B)
                order_stack_A, order_stack_B = compute_entering_time(a, b)
                index_A = 1
                index_B = 1
                schedule_A = []
                schedule_B = []
                while len(order_stack_A) > 0:
                    top = order_stack_A.pop()
                    schedule_A.append([id_a[index_A], top[2], False])
                    print(id_a[index_A], 'enter =', top[2])
                    # if index_A == 1:
                    #     d = top[2]-currentTime
                    #     print('stop', id_a[index_A])
                    #     try:
                    #         traci.vehicle.setStop(id_a[1], "E1", pos=junction_x, laneIndex=1, duration=d)
                    #     except traci.exceptions.TraCIException:
                    #         pass
                    index_A += 1
                while len(order_stack_B) > 0:
                    top = order_stack_B.pop()
                    schedule_B.append([id_b[index_B], top[2], False])
                    print(id_b[index_B], 'enter =', top[2])
                    # if index_B == 1:
                    #     d = top[2]-currentTime
                    #     print('stop', id_b[index_B])
                    #     try:
                    #         traci.vehicle.setStop(id_b[1], "E1", pos=junction_x, laneIndex=0, duration=d)
                    #     except traci.exceptions.TraCIException:
                    #         pass
                    index_B += 1
            step = 0
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

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "laneMerging.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
