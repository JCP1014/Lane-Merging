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
Vehicle = namedtuple("Vehicle", "id time")

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
    pA = 1. / p # a vehicle is generated every p seconds in average.
    pB = 1. / p
    pC = 1. / p
    print("11")
    with open("laneMerging.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeA" type="passenger" length="5" accel="1.5" decel="2" sigma="0.0" maxSpeed="20" color="yellow"/>
        <vType id="typeB" type="passenger" length="5" accel="1.5" decel="2" sigma="0.0" maxSpeed="20" color="blue"/>
        <vType id="typeC" type="passenger" length="5" accel="1.5" decel="2" sigma="0.0" maxSpeed="20" color="magenta"/>

        <route edges="E0 E3 E6 E9" color="yellow" id="route_0"/>
        <route edges="E1 E4 E7 E9" color="yellow" id="route_1"/>
        <route edges="E1 E4 E7 E10" color="yellow" id="route_2"/>
        <route edges="E2 E5 E8 E10" color="yellow" id="route_3"/>""", file=routes)
        num_A = 1
        num_B = 1
        num_C = 1
        for i in range(N):
            if random.uniform(0, 1) < pA:
                print('    <vehicle id="A_%i" type="typeA" route="route_0" depart="%i" departLane="0" departSpeed="random"/>' % (
                    num_A, i), file=routes)
                num_A += 1
            if random.uniform(0, 1) < pB:
                print('    <vehicle id="B_%i" type="typeB" route="route_1" depart="%i" departLane="0" departSpeed="random"/>' % (
                    num_B, i), file=routes)
                num_B += 1
            if random.uniform(0, 1) < pC:
                print('    <vehicle id="C_%i" type="typeC" route="route_3" depart="%i" departLane="0" departSpeed="random"/>' % (
                    num_C, i), file=routes)
                num_C += 1
        print("</routes>", file=routes)


def compute_earliest_arrival(laneLength, schedule_A, schedule_B1, schedule_B2, schedule_C):
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
            for i in schedule_B1:
                if i.id == vehID:
                    arrivalTime = i.time
                    isFound = True
                    break
            if not isFound:
                for i in schedule_B2:
                    if i.id == vehID:
                        arrivalTime = i.time
                        break
        elif speed != 0:
            arrivalTime = currentTime + (dist/speed)
            isFound = False
            for i in schedule_B1:
                if i.id == vehID:
                    if i.time < arrivalTime:
                        arrivalTime = i.time
                        isFound = True
                    break
            if not isFound:
                for i in schedule_B2:
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


def compute_entering_time(a, b, c, W_same, W_diff):
    alpha = len(a) - 1
    beta = len(b) - 1
    gamma = len(c) - 1

    L1 = np.zeros((alpha+1, beta+1, 2))  # dp table
    L2 = np.zeros((gamma+1, beta+1, 2))  # dp table

    # Initialize
    L1[0][0][0] = 0
    L1[0][0][1] = 0
    L1[1][0][0] = a[1].time
    # L1[0][1][1] = b[1].time
    L2[0][0][0] = 0
    L2[0][0][1] = 0
    L2[1][0][0] = c[1].time
    # L2[0][1][1] = b[1].time
    cnt = 8
    for i in range(2, alpha+1):
        L1[i][0][0] = max(a[i].time, L1[i-1][0][0]+W_same)
        cnt += 1
    for i in range(1, alpha+1):
        L1[i][0][1] = L1[i][0][0] + W_diff
        cnt += 1
    for i in range(2, gamma+1):
        L2[i][0][0] = max(c[i].time, L2[i-1][0][0]+W_same)
        cnt += 1
    for i in range(1, gamma+1):
        L2[i][0][1] = L2[i][0][0] + W_diff
        cnt += 1

    beta_sum = 1
    beta_1 = 1
    beta_2 = 1
    queue_1 = []
    queue_2 = []
    while beta_sum <= beta:
        L1[0][beta_1][1] = max(b[beta_sum].time, L1[0][beta_1-1][1]+W_same)
        L1[0][beta_1][0] = L1[0][beta_1][1] + W_diff
        L2[0][beta_2][1] = max(b[beta_sum].time, L2[0][beta_2-1][1]+W_same)
        L2[0][beta_2][0] = L2[0][beta_2][1] + W_diff    
        cnt += 4
        for i in range(1, alpha+1):
            L1[i][beta_1][0] = min( max(a[i].time, L1[i-1][beta_1][0]+W_same), max(a[i].time, L1[i-1][beta_1][1]+W_diff) )
            L1[i][beta_1][1] = min( max(b[beta_sum].time, L1[i][beta_1-1][0]+W_diff), max(b[beta_sum].time, L1[i][beta_1-1][1]+W_same) )
            cnt += 2
        for i in range(1, gamma+1):
            L2[i][beta_2][0] = min( max(c[i].time, L2[i-1][beta_2][0]+W_same), max(c[i].time, L2[i-1][beta_2][1]+W_diff) )
            L2[i][beta_2][1] = min( max(b[beta_sum].time, L2[i][beta_2-1][0]+W_diff), max(b[beta_sum].time, L2[i][beta_2-1][1]+W_same) )
            cnt += 2

        min_1 = min(L1[alpha][beta_1][0], L1[alpha][beta_1][1])
        min_2 = min(L2[gamma][beta_2][0], L2[gamma][beta_2][1])
        if min_1 <= min_2:
            queue_1.append(b[beta_1].id)
            beta_1 += 1
        else:
            queue_2.append(b[beta_2].id)
            beta_2 += 1
        beta_sum += 1
    beta_1 -= 1
    beta_2 -= 1

    # Choose optimal solution
    schedule_A = []
    schedule_B1 = []
    i = alpha
    j = beta_1
    while i>0 or j>0:
        if L1[i][j][1] < L1[i][j][0]:
            schedule_B1.append(Vehicle(queue_1.pop(), L1[i][j][1]))
            j -= 1
        else:
            schedule_A.append(Vehicle(a[i].id, L1[i][j][0]))
            i -= 1

    schedule_C = []
    schedule_B2 = []
    i = gamma
    j = beta_2
    while i>0 or j>0:
        if L2[i][j][1] < L2[i][j][0]:
            schedule_B2.append(Vehicle(queue_2.pop(), L2[i][j][1]))
            j -= 1
        else:
            schedule_C.append(Vehicle(c[i].id, L2[i][j][0]))
            i -= 1
    
    # Output order
    # while len(order_stack) > 0:
    #     print(order_stack.pop())

    schedule_A.reverse()
    schedule_B1.reverse()
    schedule_B2.reverse()
    schedule_C.reverse()

    return schedule_A, schedule_B1, schedule_B2, schedule_C

def print_table(L):
    for k in range(L.shape[2]):
        for i in range(L.shape[0]):
            for j in range(L.shape[1]):
                print(L[i][j][k], end=' ')
            print('')
        print('')


def run():
    W_same = 1  # the waiting time if two consecutive vehicles are from the same lane
    W_diff = 3  # the waiting time if two consecutive vehicles are from different lanes
    step = 0
    period = 4
    junction_x = traci.junction.getPosition("gneJ20")[0]
    schedule_A = []
    schedule_B1 = []
    schedule_B2 = []
    schedule_C = []
    leaveA = False
    leaveB1 = False
    leaveB2 = False
    leaveC = False
    countdown = 0
    endTime = 0
    gA = False
    gB1 = False
    gB2 = False
    gC = False
    cnt = 0
    laneLength = 600

    """execute the TraCI control loop"""
    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("0", 2)
    while traci.simulation.getMinExpectedNumber() > 0:  # The number of vehicles which are in the net plus the ones still waiting to start. 
        print(cnt)
        print(traci.trafficlight.getPhase('TL1'))
        cnt += 1
        print(traci.lanearea.getLastStepVehicleIDs("dA"))
        print(traci.lanearea.getLastStepVehicleIDs("dB"))
        print(traci.lanearea.getLastStepVehicleIDs("dC"))
        if step == period:
            if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0 and traci.lanearea.getLastStepVehicleNumber("dC") > 0:
                a, b, c = compute_earliest_arrival(laneLength, schedule_A, schedule_B1, schedule_B2, schedule_C)
                schedule_A, schedule_B1, schedule_B2, schedule_C = compute_entering_time(a, b, c, W_same, W_diff)
                schedule_A.sort(key=lambda x: x[1])
                schedule_B1.sort(key=lambda x: x[1])
                schedule_B2.sort(key=lambda x: x[1])
                schedule_C.sort(key=lambda x: x[1])
                print('a', a)
                print('b', b)
                print('c', c)
                print('s_A', schedule_A)
                print('s_B1', schedule_B1)
                print('s_B2', schedule_B2)
                print('s_C', schedule_C)
            step = 0

        for vehID in traci.simulation.getLoadedIDList():
            traci.vehicle.setLaneChangeMode(vehID, 0b000000000000)
        traci.simulationStep()
        b_car = traci.lanearea.getLastStepVehicleIDs("dB")
        if (cnt > 10 and cnt < 45):
             print(traci.vehicle.getSpeed('A_1'),traci.vehicle.getDistance("A_1"))
        for i in range (1,len(b_car)):
            #print(traci.vehicle.getDistance(b_car[i]))
            if(traci.vehicle.getDistance(b_car[i])> 590 and traci.vehicle.getSpeed(b_car[i]) > 0):
                #print(b_car[i],traci.vehicle.getDistance(b_car[i]))
                continue
            traci.vehicle.setRouteID(b_car[i],"route_1")
            print(f'set_route_1: {b_car[i]}')
        traci.trafficlight.setPhase("TL1",1)
        step += 1
        currentTime = traci.simulation.getTime()
        endTime = currentTime
        # print(currentTime)
        # print("Pass", traci.lane.getLastStepVehicleIDs("E2_0"))
        if len(schedule_A) > 0 and traci.vehicle.getPosition(schedule_A[0].id)[0] >= junction_x:
            schedule_A.remove(schedule_A[0])
            leaveA = True
        elif len(schedule_B1) > 0 and traci.vehicle.getPosition(schedule_B1[0].id)[0] >= junction_x:
            schedule_B1.remove(schedule_B1[0])
            leaveB1 = True
        if len(schedule_C) > 0 and traci.vehicle.getPosition(schedule_C[0].id)[0] >= junction_x:
            schedule_C.remove(schedule_C[0])
            leaveC = True
        elif len(schedule_B2) > 0 and traci.vehicle.getPosition(schedule_B2[0].id)[0] >= junction_x:
            schedule_B2.remove(schedule_B2[0])
            leaveB1 = True
            
        if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0 and traci.lanearea.getLastStepVehicleNumber("dC") > 0:
            gA = False
            gB1 = False
            gB2 = False
            gC = False
            # Control outgoing lane 1
            if len(schedule_A) > 0 and len(schedule_B1) > 0:
                if schedule_A[0].time < schedule_B1[0].time:
                    if leaveA:
                        countdown = W_same - 1
                    elif leaveB1:
                        countdown = W_diff - 1
                    else:
                        if countdown:
                            traci.trafficlight.setPhase("TL1", 1)
                            countdown -= 1
                        else:
                            # traci.trafficlight.setPhase("TL1", 2)
                            gA = True
                else:
                    if leaveB1:
                        countdown = W_same - 1
                    elif leaveA:
                        countdown = W_diff - 1
                    else:
                        if countdown:
                            traci.trafficlight.setPhase("TL1", 1)
                            countdown -= 1
                        else:
                            # traci.trafficlight.setPhase("TL1", 4)
                            gB1 = True
            elif len(schedule_A) > 0:
                if leaveA:
                    countdown = W_same - 1
                elif leaveB1:
                    countdown = W_diff - 1
                else:
                    if countdown:
                        traci.trafficlight.setPhase("TL1", 1)
                        countdown -= 1
                    else:
                        # traci.trafficlight.setPhase("TL1", 2)
                        
                        gA = True
            elif len(schedule_B1) > 0:
                if leaveB1:
                        countdown = W_same - 1
                elif leaveA:
                    countdown = W_diff - 1
                else:
                    if countdown:
                        traci.trafficlight.setPhase("TL1", 1)
                        countdown -= 1
                    else:
                        # traci.trafficlight.setPhase("TL1", 4)
                        gB1 = True
            else:
                if countdown:
                    traci.trafficlight.setPhase("TL1", 1)
                    countdown -= 1
                else:
                    # traci.trafficlight.setPhase("TL1", 0)
                    gA = True
                    gB1 = True

            # Control outgoing lane 2
            if len(schedule_C) > 0 and len(schedule_B2) > 0:
                if schedule_C[0].time < schedule_B2[0].time:
                    if leaveC:
                        countdown = W_same - 1
                    elif leaveB2:
                        countdown = W_diff - 1
                    else:
                        if countdown:
                            traci.trafficlight.setPhase("TL1", 1)
                            countdown -= 1
                        else:
                            # traci.trafficlight.setPhase("TL1", 8)
                            print(schedule_A,schedule_B1,schedule_B2,schedule_C)
                            gC = True
                else:
                    if leaveB2:
                        countdown = W_same - 1
                    elif leaveC:
                        countdown = W_diff - 1
                    else:
                        if countdown:
                            traci.trafficlight.setPhase("TL1", 1)
                            countdown -= 1
                        else:
                            # traci.trafficlight.setPhase("TL1", 6)
                            gB2 = True
            elif len(schedule_C) > 0:
                if leaveC:
                        countdown = W_same - 1
                elif leaveB2:
                    countdown = W_diff - 1
                else:
                    if countdown:
                        traci.trafficlight.setPhase("TL1", 1)
                        countdown -= 1
                    else:
                        # traci.trafficlight.setPhase("TL1", 8)
                        print(schedule_A,schedule_B1,schedule_B2,schedule_C)
                        gC = True
            elif len(schedule_B2) > 0:
                if leaveB2:
                        countdown = W_same - 1
                elif leaveC:
                    countdown = W_diff - 1
                else:
                    if countdown:
                        traci.trafficlight.setPhase("TL1", 1)
                        countdown -= 1
                    else:
                        # traci.trafficlight.setPhase("TL1", 6)
                        gB2 = True
            else:
                if countdown:
                    traci.trafficlight.setPhase("TL1", 1)
                    countdown -= 1
                else:
                    # traci.trafficlight.setPhase("TL1", 0)
                    gC = True
                    gB2 = True
            
            if gA and gB1 and gB2 and gC:
                traci.trafficlight.setPhase("TL1", 0)
                print(0)
            elif gA and gB2:
                traci.trafficlight.setPhase("TL1", 2)
                print("aa2")
                b_car = traci.lanearea.getLastStepVehicleIDs("dB")
                for i in range (len(b_car)):
                    if(traci.vehicle.getDistance(b_car[i]) > 590 and traci.vehicle.getSpeed(b_car[i]) > 0):
                        continue
                    traci.vehicle.setRouteID(b_car[i],"route_2")
                    print(f'set_route_2: {b_car[i]}')
                print("aa2")
            elif gA and gC:
                traci.trafficlight.setPhase("TL1", 4)
                print("aa4")
            elif gB1 and gC:
                traci.trafficlight.setPhase("TL1", 6)
                print('first',6)
            else:
                traci.trafficlight.setPhase("TL1", 1)

        elif traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0:
            
            if countdown:
                traci.trafficlight.setPhase("TL1", 1)
                countdown -= 1
            else:
                traci.trafficlight.setPhase("TL1", 2)
                for i in range (len(b_car)):
                     traci.vehicle.setRouteID(b_car[i],"route_2")
                
                print(2)
        elif traci.lanearea.getLastStepVehicleNumber("dC") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0:
            if countdown:
                traci.trafficlight.setPhase("TL1", 1)
                countdown -= 1
            else:
                traci.trafficlight.setPhase("TL1", 6)
                for veh in schedule_B1:
                    traci.vehicle.updateBestLanes(veh.id)
                    print('second',6)
        else:
            if countdown:
                traci.trafficlight.setPhase("TL1", 1)
                countdown -= 1
            else:
                traci.trafficlight.setPhase("TL1", 0)
        leaveA = False
        leaveB1 = False
        leaveB2 = False
        leaveC = False

    print(endTime)
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
        traci.start([sumoBinary, "-c", "sumo_input/laneMerging.sumocfg",
                                "--tripinfo-output", "tripinfo_dp.xml",
                                "-S",
                                "--no-step-log", "true", "-W", "--duration-log.disable", "true"])
        run()


if __name__ == "__main__":
    main()
