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

def generate_routefile(N, p):
    # random.seed(42)  # make tests reproducible
    # demand per second from different directions
    pA = 1. / p # a vehicle is generated every 8 seconds in average.
    pB = 1. / p # a vehicle is generated every 12 seconds in average.
    with open("laneMerging.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeA" type="passenger" length="5" accel="1.5" decel="2" sigma="0.0" maxSpeed="20" color="yellow"/>
        <vType id="typeB" type="passenger" length="5" accel="1.5" decel="2" sigma="0.0" maxSpeed="20" color="blue"/>

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
                    break
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


def compute_entering_time(a, b, W_same, W_diff):
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
    W_same = 1  # the waiting time if two consecutive vehicles are from the same lane
    W_diff = 2  # the waiting time if two consecutive vehicles are from different lanes
    step = 0
    period = 3
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
                order_stack_A, order_stack_B = compute_entering_time(a, b, W_same, W_diff)
                index_A = 1
                index_B = 1
                schedule_A = []
                schedule_B = []
                while len(order_stack_A) > 0:
                    top = order_stack_A.pop()
                    schedule_A.append([id_a[index_A], top[2], False])
                    index_A += 1
                schedule_A.sort(key=lambda x: x[1])
                while len(order_stack_B) > 0:
                    top = order_stack_B.pop()
                    schedule_B.append([id_b[index_B], top[2], False])
                    index_B += 1
                schedule_B.sort(key=lambda x: x[1])
            step = 0

        for vehID in traci.simulation.getLoadedIDList():
            traci.vehicle.setLaneChangeMode(vehID, 0b000000000000)
        traci.simulationStep()
        step += 1
        currentTime = traci.simulation.getTime()
        endTime = currentTime
        # print(currentTime)
        # print("Pass", traci.lane.getLastStepVehicleIDs("E2_0"))
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
                        traci.trafficlight.setPhase("gneJ1", 1)
                        countdown -= 1
                    else:
                        traci.trafficlight.setPhase("gneJ1", 2)
                # traci.trafficlight.setPhaseDuration("gneJ1", schedule_A[0][1]-currentTime)
            else:
                if leaveB:
                    countdown = W_same - 1
                elif leaveA:
                    countdown = W_diff - 1
                else:
                    if countdown:
                        traci.trafficlight.setPhase("gneJ1", 1)
                        countdown -= 1
                    else:
                        traci.trafficlight.setPhase("gneJ1", 4)
                # traci.trafficlight.setPhaseDuration("gneJ1", schedule_B[0][1]-currentTime)
        elif len(schedule_A) > 0:
            if leaveA:
                    countdown = W_same - 1
            elif leaveB:
                countdown = W_diff - 1
            else:
                if countdown:
                    traci.trafficlight.setPhase("gneJ1", 1)
                    countdown -= 1
                else:
                    traci.trafficlight.setPhase("gneJ1", 2)
        elif len(schedule_B) > 0:
            if leaveB:
                    countdown = W_same - 1
            elif leaveA:
                countdown = W_diff - 1
            else:
                if countdown:
                    traci.trafficlight.setPhase("gneJ1", 1)
                    countdown -= 1
                else:
                    traci.trafficlight.setPhase("gneJ1", 4)
        else:
            if countdown:
                traci.trafficlight.setPhase("gneJ1", 1)
                countdown -= 1
            else:
                traci.trafficlight.setPhase("gneJ1", 0)

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


def main():
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    if len(sys.argv) < 2:
        print("Please input a number for N")
        return
    else:
        # first, generate the route file for this simulation
        N = int(sys.argv[1])  # number of time steps
        p = int(sys.argv[2])
        generate_routefile(N, p)

        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        traci.start([sumoBinary, "-c", "laneMerging.sumocfg",
                                "--tripinfo-output", "tripinfo_dp.xml",
                                "-S",
                                "--no-step-log", "true", "-W", "--duration-log.disable", "true"])
        run()


if __name__ == "__main__":
    main()
