# Run: python3 runner_dp_v2.py [density] [number] [W=] [W+] [windowSize=100] [generateNewTest (T/F)] [inputPath]
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

Sol = namedtuple("Sol", "time table lane src")
Lane = namedtuple("Lane", "traffic num num_tmp")
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


def choose_best_sol(sol_list):
    minMax = max(sol_list[0].time)
    index = 0
    for i in range(1, len(sol_list)):
        tmpMax = max(sol_list[i].time)
        if tmpMax < minMax:
            minMax = tmpMax
            index = i
        elif tmpMax == minMax:
            tmpSum = sum(sol_list[i].time)
            minSum = sum(sol_list[index].time)
            if tmpSum < minSum:
                minMax = tmpMax
                index = i
            elif tmpSum == minSum and sol_list[i].src is not None:
                tmpSrcMax = max(sol_list[i].src.time)
                minSrcMax = max(sol_list[index].src.time)
                if tmpSrcMax < minSrcMax:
                    minMax = tmpMax
                    index = i
                elif tmpSrcMax == minSrcMax:
                    tmpSrcSum = sum(sol_list[i].src.time)
                    minSrcSum = sum(sol_list[index].src.time)
                    if tmpSrcSum < minSrcSum:
                        minMax = tmpMax
                        index = i
    return sol_list[index]


def dp_compute_entering_time(a, b, c, W_same, W_diff, last_X, last_Y):
    alpha = len(a) - 1
    beta = len(b) - 1
    gamma = len(c) - 1

    L_AB = [[[Sol((float('inf'), float('inf')), '', '', None) for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_AC = [[[Sol((float('inf'), float('inf')), '', '', None) for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_BB = [[[Sol((float('inf'), float('inf')), '', '', None) for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_BC = [[[Sol((float('inf'), float('inf')), '', '', None) for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]

    # Initialize
    L_AB[0][0][0] = Sol((last_X[1], last_Y[1]), '', '', None)
    L_AC[0][0][0] = Sol((last_X[1], last_Y[1]), '', '', None)
    L_BB[0][0][0] = Sol((last_X[1], last_Y[1]), '', '', None)
    L_BC[0][0][0] = Sol((last_X[1], last_Y[1]), '', '', None)

    last_XY = last_X[0] + last_Y[0]
    T_X = last_X[1]
    T_Y = last_Y[1]
    if last_XY == 'AB':
        L_AB[1][1][0] = Sol((max(a[1].time, T_X+W_same),
                             max(b[1].time, T_Y+W_same)), 'AB', 'XY', None)
        L_AC[1][0][1] = Sol((max(a[1].time, T_X+W_same),
                             max(c[1].time, T_Y+W_diff)), 'AC', 'XY', None)
        L_BC[0][1][1] = Sol((max(b[1].time, T_X+W_diff),
                             max(c[1].time, T_Y+W_diff)), 'BC', 'XY', None)
        L_BB[0][1][0] = choose_best_sol([
            Sol((max(b[1].time, T_X+W_diff), T_Y), 'BB', 'X', None),
            Sol((T_X, max(b[1].time, T_Y+W_same)), 'BB', 'Y', None)]
        )
        L_AB[1][0][0] = Sol((max(a[1].time, T_X+W_same), T_Y), 'AB', 'X', None)
        L_AB[0][1][0] = Sol((T_X, max(b[1].time, T_Y+W_same)), 'AB', 'Y', None)
        L_AC[1][0][0] = Sol((max(a[1].time, T_X+W_same), T_Y), 'AC', 'X', None)
        L_AC[0][0][1] = Sol((T_X, max(c[1].time, T_Y+W_diff)), 'AC', 'Y', None)
        L_BC[0][1][0] = Sol((max(b[1].time, T_X+W_diff), T_Y), 'BC', 'X', None)
        L_BC[0][0][1] = Sol((T_X, max(c[1].time, T_Y+W_diff)), 'BC', 'Y', None)
    elif last_XY == 'AC':
        L_AB[1][1][0] = Sol((max(a[1].time, T_X+W_same),
                             max(b[1].time, T_Y+W_diff)), 'AB', 'XY', None)
        L_AC[1][0][1] = Sol((max(a[1].time, T_X+W_same),
                             max(c[1].time, T_Y+W_same)), 'AC', 'XY', None)
        L_BC[0][1][1] = Sol((max(b[1].time, T_X+W_diff),
                             max(c[1].time, T_Y+W_same)), 'BC', 'XY', None)
        L_BB[0][1][0] = choose_best_sol([
            Sol((max(b[1].time, T_X+W_diff), T_Y), 'BB', 'X', None),
            Sol((T_X, max(b[1].time, T_Y+W_diff)), 'BB', 'Y', None)]
        )
        L_AB[1][0][0] = Sol((max(a[1].time, T_X+W_same), T_Y), 'AB', 'X', None)
        L_AB[0][1][0] = Sol((T_X, max(b[1].time, T_Y+W_diff)), 'AB', 'Y', None)
        L_AC[1][0][0] = Sol((max(a[1].time, T_X+W_same), T_Y), 'AC', 'X', None)
        L_AC[0][0][1] = Sol((T_X, max(c[1].time, T_Y+W_same)), 'AC', 'Y', None)
        L_BC[0][1][0] = Sol((max(b[1].time, T_X+W_diff), T_Y), 'BC', 'X', None)
        L_BC[0][0][1] = Sol((T_X, max(c[1].time, T_Y+W_same)), 'BC', 'Y', None)
    elif last_XY == 'BB':
        L_AB[1][1][0] = Sol((max(a[1].time, T_X+W_diff),
                             max(b[1].time, T_Y+W_same)), 'AB', 'XY', None)
        L_AC[1][0][1] = Sol((max(a[1].time, T_X+W_diff),
                             max(c[1].time, T_Y+W_diff)), 'AC', 'XY', None)
        L_BC[0][1][1] = Sol((max(b[1].time, T_X+W_same),
                             max(c[1].time, T_Y+W_diff)), 'BC', 'XY', None)
        L_BB[0][1][0] = choose_best_sol([
            Sol((max(b[1].time, T_X+W_same), T_Y), 'BB', 'X', None),
            Sol((T_X, max(b[1].time, T_Y+W_same)), 'BB', 'Y', None)]
        )
        L_AB[1][0][0] = Sol((max(a[1].time, T_X+W_diff), T_Y), 'AB', 'X', None)
        L_AB[0][1][0] = Sol((T_X, max(b[1].time, T_Y+W_same)), 'AB', 'Y', None)
        L_AC[1][0][0] = Sol((max(a[1].time, T_X+W_diff), T_Y), 'AC', 'X', None)
        L_AC[0][0][1] = Sol((T_X, max(c[1].time, T_Y+W_diff)), 'AC', 'Y', None)
        L_BC[0][1][0] = Sol((max(b[1].time, T_X+W_same), T_Y), 'BC', 'X', None)
        L_BC[0][0][1] = Sol((T_X, max(c[1].time, T_Y+W_diff)), 'BC', 'Y', None)
    elif last_XY == 'BC':
        L_AB[1][1][0] = Sol((max(a[1].time, T_X+W_diff),
                             max(b[1].time, T_Y+W_diff)), 'AB', 'XY', None)
        L_AC[1][0][1] = Sol((max(a[1].time, T_X+W_diff),
                             max(c[1].time, T_Y+W_same)), 'AC', 'XY', None)
        L_BC[0][1][1] = Sol((max(b[1].time, T_X+W_same),
                             max(c[1].time, T_Y+W_same)), 'BC', 'XY', None)
        L_BB[0][1][0] = choose_best_sol([
            Sol((max(b[1].time, T_X+W_same), T_Y), 'BB', 'X', None),
            Sol((T_X, max(b[1].time, T_Y+W_diff)), 'BB', 'Y', None)]
        )
        L_AB[1][0][0] = Sol((max(a[1].time, T_X+W_diff), T_Y), 'AB', 'X', None)
        L_AB[0][1][0] = Sol((T_X, max(b[1].time, T_Y+W_diff)), 'AB', 'Y', None)
        L_AC[1][0][0] = Sol((max(a[1].time, T_X+W_diff), T_Y), 'AC', 'X', None)
        L_AC[0][0][1] = Sol((T_X, max(c[1].time, T_Y+W_same)), 'AC', 'Y', None)
        L_BC[0][1][0] = Sol((max(b[1].time, T_X+W_same), T_Y), 'BC', 'X', None)
        L_BC[0][0][1] = Sol((T_X, max(c[1].time, T_Y+W_same)), 'BC', 'Y', None)
    else:
        L_AB[1][1][0] = Sol((a[1].time, b[1].time), 'AB', 'XY', None)
        L_AC[1][0][1] = Sol((a[1].time, c[1].time), 'AC', 'XY', None)
        L_BC[0][1][1] = Sol((b[1].time, c[1].time), 'BC', 'XY', None)
        L_BB[0][1][0] = Sol((-W_diff, b[1].time), 'BB', 'Y',
                            None) if a[1].time <= c[1].time else Sol((b[1].time, -W_diff), 'BB', 'X', None)
        L_AB[1][0][0] = Sol((a[1].time, -W_diff), 'AB', 'X', None)
        L_AB[0][1][0] = Sol((-W_diff, b[1].time), 'AB', 'Y', None)
        L_AC[1][0][0] = Sol((a[1].time, -W_diff), 'AC', 'X', None)
        L_AC[0][0][1] = Sol((-W_diff, c[1].time), 'AC', 'Y', None)
        L_BC[0][1][0] = Sol((b[1].time, -W_diff), 'BC', 'X', None)
        L_BC[0][0][1] = Sol((-W_diff, c[1].time), 'BC', 'Y', None)

    for j in range(2, beta+1):
        L_BB[0][j][0] = choose_best_sol([
            Sol((max(b[j].time, L_BB[0][j-1][0].time[0]+W_same),
                 L_BB[0][j-1][0].time[1]), 'BB', 'X', None),
            Sol((L_BB[0][j-1][0].time[0], max(b[j].time, L_BB[0][j-1][0].time[1]+W_same)), 'BB', 'Y', None)]
        )

    for i in range(2, alpha+1):
        L_AB[i][0][0] = Sol((max(a[i].time, L_AB[i-1][0][0].time[0] +
                                 W_same), L_AB[i-1][0][0].time[1]), 'AB', 'X', None)
    for j in range(2, beta+1):
        L_AB[0][j][0] = Sol((L_AB[0][j-1][0].time[0], max(b[j].time,
                                                          L_AB[0][j-1][0].time[1]+W_same)), 'AB', 'Y', None)
    for i in range(2, alpha+1):
        L_AC[i][0][0] = Sol((max(a[i].time, L_AC[i-1][0][0].time[0] +
                                 W_same), L_AC[i-1][0][0].time[1]), 'AC', 'X', None)
    for k in range(2, gamma+1):
        L_AC[0][0][k] = Sol((L_AC[0][0][k-1].time[0], max(c[k].time,
                                                          L_AC[0][0][k-1].time[1]+W_same)), 'AC', 'Y', None)
    for j in range(2, beta+1):
        L_BC[0][j][0] = Sol((max(b[j].time, L_BC[0][j-1][0].time[0] +
                                 W_same), L_BC[0][j-1][0].time[1]), 'BC', 'X', None)
    for k in range(2, gamma+1):
        L_BC[0][0][k] = Sol((L_BC[0][0][k-1].time[0], max(c[k].time,
                                                          L_BC[0][0][k-1].time[1]+W_same)), 'BC', 'Y', None)

    if beta > 1:
        if last_XY == 'AB':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = choose_best_sol([
                    Sol((max(b[1].time, L_AC[i][0][0].time[0]+W_diff),
                         max(b[2].time, T_Y+W_same)), 'AC', 'XY', None),
                    Sol((max(b[2].time, L_AC[i][0][0].time[0]+W_diff), max(b[1].time, T_Y+W_same)), 'AC', 'YX', None)]
                )
            for k in range(1, gamma+1):
                L_BB[0][2][k] = choose_best_sol([
                    Sol((max(b[1].time, T_X+W_diff), max(b[2].time,
                                                         L_AC[0][0][k].time[1]+W_diff)), 'AC', 'XY', None),
                    Sol((max(b[2].time, T_X+W_diff), max(b[1].time, L_AC[0][0][k].time[1]+W_diff)), 'AC', 'YX', None)]
                )
        elif last_XY == 'AC':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = choose_best_sol([
                    Sol((max(b[1].time, L_AC[i][0][0].time[0]+W_diff),
                         max(b[2].time, T_Y+W_diff)), 'AC', 'XY', None),
                    Sol((max(b[2].time, L_AC[i][0][0].time[0]+W_diff), max(b[1].time, T_Y+W_diff)), 'AC', 'YX', None)]
                )
            for k in range(1, gamma+1):
                L_BB[0][2][k] = choose_best_sol([
                    Sol((max(b[1].time, T_X+W_diff), max(b[2].time,
                                                         L_AC[0][0][k].time[1]+W_diff)), 'AC', 'XY', None),
                    Sol((max(b[2].time, T_X+W_diff), max(b[1].time, L_AC[0][0][k].time[1]+W_diff)), 'AC', 'YX', None)]
                )
        elif last_XY == 'BB':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = choose_best_sol([
                    Sol((max(b[1].time, L_AC[i][0][0].time[0]+W_diff),
                         max(b[2].time, T_Y+W_same)), 'AC', 'XY', None),
                    Sol((max(b[2].time, L_AC[i][0][0].time[0]+W_diff), max(b[1].time, T_Y+W_same)), 'AC', 'YX', None)]
                )
            for k in range(1, gamma+1):
                L_BB[0][2][k] = choose_best_sol([
                    Sol((max(b[1].time, T_X+W_same), max(b[2].time,
                                                         L_AC[0][0][k].time[1]+W_diff)), 'AC', 'XY', None),
                    Sol((max(b[2].time, T_X+W_same), max(b[1].time, L_AC[0][0][k].time[1]+W_diff)), 'AC', 'YX', None)]
                )
        elif last_XY == 'BC':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = choose_best_sol([
                    Sol((max(b[1].time, L_AC[i][0][0].time[0]+W_diff),
                         max(b[2].time, T_Y+W_diff)), 'AC', 'XY', None),
                    Sol((max(b[2].time, L_AC[i][0][0].time[0]+W_diff), max(b[1].time, T_Y+W_diff)), 'AC', 'YX', None)]
                )
            for k in range(1, gamma+1):
                L_BB[0][2][k] = choose_best_sol([
                    Sol((max(b[1].time, T_X+W_same), max(b[2].time,
                                                         L_AC[0][0][k].time[1]+W_diff)), 'AC', 'XY', None),
                    Sol((max(b[2].time, T_X+W_same), max(b[1].time, L_AC[0][0][k].time[1]+W_diff)), 'AC', 'YX', None)]
                )
        else:
            for i in range(1, alpha+1):
                L_BB[i][2][0] = choose_best_sol([
                    Sol((max(b[1].time, L_AC[i][0][0].time[0] +
                             W_diff), b[2].time), 'AC', 'XY', None),
                    Sol((max(b[2].time, L_AC[i][0][0].time[0]+W_diff), b[1].time), 'AC', 'YX', None)]
                )
            for k in range(1, gamma+1):
                L_BB[0][2][k] = choose_best_sol([
                    Sol((b[1].time, max(b[2].time, L_AC[0][0]
                                        [k].time[1]+W_diff)), 'AC', 'XY', None),
                    Sol((b[2].time, max(b[1].time, L_AC[0][0][k].time[1]+W_diff)), 'AC', 'YX', None)]
                )

    for i in range(1, alpha+1):
        for j in range(2, beta+1):
            L_AB[i][j][0] = choose_best_sol([
                Sol((max(a[i].time, L_AB[i-1][j][0].time[0]+W_same),
                     L_AB[i-1][j][0].time[1]), 'AB', 'X', L_AB[i-1][j][0]),
                Sol((max(a[i].time, L_BB[i-1][j][0].time[0]+W_diff),
                     L_BB[i-1][j][0].time[1]), 'BB', 'X', L_BB[i-1][j][0]),
                Sol((L_AB[i][j-1][0].time[0], max(b[j].time, L_AB[i][j-1][0].time[1]+W_same)), 'AB', 'Y', L_AB[i][j-1][0])]
            )
    for i in range(2, alpha+1):
        for k in range(2, gamma+1):
            L_AC[i][0][k] = Sol((max(a[i].time, L_AC[i-1][0][k-1].time[0]+W_same), max(
                c[k].time, L_AC[i-1][0][k-1].time[1]+W_same)), 'AC', 'XY', None)
    for j in range(2, beta+1):
        for k in range(1, gamma+1):
            L_BC[0][j][k] = choose_best_sol([
                Sol((max(b[j].time, L_BC[0][j-1][k].time[0]+W_same),
                     L_BC[0][j-1][k].time[1]), 'BC', 'X', L_BC[0][j-1][k]),
                Sol((L_BC[0][j][k-1].time[0], max(c[k].time, L_BC[0][j]
                                                  [k-1].time[1]+W_same)), 'BC', 'Y', L_BC[0][j][k-1]),
                Sol((L_BB[0][j][k-1].time[0], max(c[k].time, L_BB[0][j][k-1].time[1]+W_diff)), 'BB', 'Y', L_BB[0][j][k-1])]
            )
    for i in range(1, alpha+1):
        for j in range(3, beta+1):
            L_BB[i][j][0] = choose_best_sol([
                Sol((max(b[j].time, L_BB[i][j-1][0].time[0]+W_same),
                     L_BB[i][j-1][0].time[1]), 'BB', 'X', L_BB[i][j-1][0]),
                Sol((max(b[j].time, L_AB[i][j-1][0].time[0]+W_diff),
                     L_AB[i][j-1][0].time[1]), 'AB', 'X', L_AB[i][j-1][0]),
                Sol((L_BB[i][j-1][0].time[0], max(b[j].time, L_BB[i][j-1][0].time[1]+W_same)), 'BB', 'Y', L_BB[i][j-1][0])]
            )

    for i in range(1, alpha+1):
        for j in range(1, beta+1):
            for k in range(1, gamma+1):
                L_AB[i][j][k] = choose_best_sol([
                    Sol((max(a[i].time, L_AB[i-1][j][k].time[0]+W_same),
                         L_AB[i-1][j][k].time[1]), 'AB', 'X', L_AB[i-1][j][k]),
                    Sol((max(a[i].time, L_BB[i-1][j][k].time[0]+W_diff),
                         L_BB[i-1][j][k].time[1]), 'BB', 'X', L_BB[i-1][j][k]),
                    Sol((L_AB[i][j-1][k].time[0], max(b[j].time, L_AB[i]
                                                      [j-1][k].time[1]+W_same)), 'AB', 'Y', L_AB[i][j-1][k]),
                    Sol((L_AC[i][j-1][k].time[0], max(b[j].time, L_AC[i][j-1][k].time[1]+W_diff)), 'AC', 'Y', L_AC[i][j-1][k])]
                )
                L_AC[i][j][k] = choose_best_sol([
                    Sol((max(a[i].time, L_AC[i-1][j][k].time[0]+W_same),
                         L_AC[i-1][j][k].time[1]), 'AC', 'X', L_AC[i-1][j][k]),
                    Sol((max(a[i].time, L_BC[i-1][j][k].time[0]+W_diff),
                         L_BC[i-1][j][k].time[1]), 'BC', 'X', L_BC[i-1][j][k]),
                    Sol((L_AC[i][j][k-1].time[0], max(c[k].time, L_AC[i][j]
                                                      [k-1].time[1]+W_same)), 'AC', 'Y', L_AC[i][j][k-1]),
                    Sol((L_AB[i][j][k-1].time[0], max(c[k].time, L_AB[i][j][k-1].time[1]+W_diff)), 'AB', 'Y', L_AB[i][j][k-1])]
                )
                L_BC[i][j][k] = choose_best_sol([
                    Sol((max(b[j].time, L_BC[i][j-1][k].time[0]+W_same),
                         L_BC[i][j-1][k].time[1]), 'BC', 'X', L_BC[i][j-1][k]),
                    Sol((max(b[j].time, L_AC[i][j-1][k].time[0]+W_diff),
                         L_AC[i][j-1][k].time[1]), 'AC', 'X', L_AC[i][j-1][k]),
                    Sol((L_BC[i][j][k-1].time[0], max(c[k].time, L_BC[i][j]
                                                      [k-1].time[1]+W_same)), 'BC', 'Y', L_BC[i][j][k-1]),
                    Sol((L_BB[i][j][k-1].time[0], max(c[k].time, L_BB[i][j][k-1].time[1]+W_diff)), 'BB', 'Y', L_BB[i][j][k-1])]
                )
                L_BB[i][j][k] = choose_best_sol([
                    Sol((max(b[j].time, L_BB[i][j-1][k].time[0]+W_same),
                         L_BB[i][j-1][k].time[1]), 'BB', 'X', L_BB[i][j-1][k]),
                    Sol((max(b[j].time, L_AB[i][j-1][k].time[0]+W_diff),
                         L_AB[i][j-1][k].time[1]), 'AB', 'X', L_AB[i][j-1][k]),
                    Sol((L_BB[i][j-1][k].time[0], max(b[j].time, L_BB[i]
                                                      [j-1][k].time[1]+W_same)), 'BB', 'Y', L_BB[i][j-1][k]),
                    Sol((L_BC[i][j-1][k].time[0], max(b[j].time, L_BC[i][j-1][k].time[1]+W_diff)), 'BC', 'Y', L_BC[i][j-1][k])]
                )

    stack_A = []
    stack_BX = []
    stack_BY = []
    stack_C = []
    i = alpha
    j = beta
    k = gamma
    # Choose the optimal solution and the table to start backtracking
    opt = choose_best_sol([L_AB[i][j][k], L_AC[i][j][k], L_BB[i]
                           [j][k], L_BC[i][j][k]])
    table = ''
    lanes = ''
    # print(opt)
    if opt.time == L_AB[i][j][k].time:
        # print('AB')
        table = L_AB[i][j][k].table
        lanes = L_AB[i][j][k].lane
        if lanes == 'X':
            stack_A.append(Vehicle(a[i].id, L_AB[i][j][k].time[0]))
            i -= 1
        elif lanes == 'Y':
            stack_BY.append(Vehicle(b[j].id, L_AB[i][j][k].time[1]))
            j -= 1
        elif lanes == 'XY':
            stack_A.append(Vehicle(a[i].id, L_AB[i][j][k].time[0]))
            stack_BY.append(Vehicle(b[j].id, L_AB[i][j][k].time[1]))
            i -= 1
            j -= 1
    elif opt.time == L_AC[i][j][k].time:
        # print('AC')
        table = L_AC[i][j][k].table
        lanes = L_AC[i][j][k].lane
        if lanes == 'X':
            stack_A.append(Vehicle(a[i].id, L_AC[i][j][k].time[0]))
            i -= 1
        elif lanes == 'Y':
            stack_C.append(Vehicle(c[k].id, L_AC[i][j][k].time[1]))
            k -= 1
        elif lanes == 'XY':
            stack_A.append(Vehicle(a[i].id, L_AC[i][j][k].time[0]))
            stack_C.append(Vehicle(c[k].id, L_AC[i][j][k].time[1]))
            i -= 1
            k -= 1
    elif opt.time == L_BB[i][j][k].time:
        # print('BB')
        table = L_BB[i][j][k].table
        lanes = L_BB[i][j][k].lane
        if lanes == 'X':
            stack_BX.append(Vehicle(b[j].id, L_BB[i][j][k].time[0]))
            j -= 1
        elif lanes == 'Y':
            stack_BY.append(Vehicle(b[j].id, L_BB[i][j][k].time[1]))
            j -= 1
        elif lanes == 'XY':
            stack_BX.append(Vehicle(b[j-1].id, L_BB[i][j][k].time[0]))
            stack_BY.append(Vehicle(b[j].id, L_BB[i][j][k].time[1]))
            j -= 2
        elif lanes == 'YX':
            stack_BY.append(Vehicle(b[j-1].id, L_BB[i][j][k].time[1]))
            stack_BX.append(Vehicle(b[j].id, L_BB[i][j][k].time[0]))
            j -= 2
    elif opt.time == L_BC[i][j][k].time:
        # print('BC')
        table = L_BC[i][j][k].table
        lanes = L_BC[i][j][k].lane
        if lanes == 'X':
            stack_BX.append(Vehicle(b[j].id, L_BC[i][j][k].time[0]))
            j -= 1
        elif lanes == 'Y':
            stack_C.append(Vehicle(c[k].id, L_BC[i][j][k].time[1]))
            k -= 1
        elif lanes == 'XY':
            stack_BX.append(Vehicle(b[j].id, L_BC[i][j][k].time[0]))
            stack_C.append(Vehicle(c[k].id, L_BC[i][j][k].time[1]))
            j -= 1
            k -= 1

    # Backtracking
    while i > 0 or j > 0 or k > 0:
        # print(table, i, j, k)
        if table == 'AB':
            # print('AB')
            table = L_AB[i][j][k].table
            lanes = L_AB[i][j][k].lane
            if lanes == 'X':
                stack_A.append(Vehicle(a[i].id, L_AB[i][j][k].time[0]))
                i -= 1
            elif lanes == 'Y':
                stack_BY.append(Vehicle(b[j].id, L_AB[i][j][k].time[1]))
                j -= 1
            elif lanes == 'XY':
                stack_A.append(Vehicle(a[i].id, L_AB[i][j][k].time[0]))
                stack_BY.append(Vehicle(b[j].id, L_AB[i][j][k].time[1]))
                i -= 1
                j -= 1
        elif table == 'AC':
            # print('AC')
            table = L_AC[i][j][k].table
            lanes = L_AC[i][j][k].lane
            if lanes == 'X':
                stack_A.append(Vehicle(a[i].id, L_AC[i][j][k].time[0]))
                i -= 1
            elif lanes == 'Y':
                stack_C.append(Vehicle(c[k].id, L_AC[i][j][k].time[1]))
                k -= 1
            elif lanes == 'XY':
                stack_A.append(Vehicle(a[i].id, L_AC[i][j][k].time[0]))
                stack_C.append(Vehicle(c[k].id, L_AC[i][j][k].time[1]))
                i -= 1
                k -= 1
        elif table == 'BB':
            # print('BB')
            table = L_BB[i][j][k].table
            lanes = L_BB[i][j][k].lane
            if lanes == 'X':
                stack_BX.append(Vehicle(b[j].id, L_BB[i][j][k].time[0]))
                j -= 1
            elif lanes == 'Y':
                stack_BY.append(Vehicle(b[j].id, L_BB[i][j][k].time[1]))
                j -= 1
            elif lanes == 'XY':
                stack_BX.append(Vehicle(b[j-1].id, L_BB[i][j][k].time[0]))
                stack_BY.append(Vehicle(b[j].id, L_BB[i][j][k].time[1]))
                j -= 2
            elif lanes == 'YX':
                stack_BY.append(Vehicle(b[j-1].id, L_BB[i][j][k].time[1]))
                stack_BX.append(Vehicle(b[j].id, L_BB[i][j][k].time[0]))
                j -= 2
        elif table == 'BC':
            # print('BC')
            table = L_BC[i][j][k].table
            lanes = L_BC[i][j][k].lane
            if lanes == 'X':
                stack_BX.append(Vehicle(b[j].id, L_BC[i][j][k].time[0]))
                j -= 1
            elif lanes == 'Y':
                stack_C.append(Vehicle(c[k].id, L_BC[i][j][k].time[1]))
                k -= 1
            elif lanes == 'XY':
                stack_BX.append(Vehicle(b[j].id, L_BC[i][j][k].time[0]))
                stack_C.append(Vehicle(c[k].id, L_BC[i][j][k].time[1]))
                j -= 1
                k -= 1

    # Delete the redundant element (i==0 or j==0 or k==0)
    while len(stack_A) > 0 and stack_A[-1][0] == '':
        stack_A.pop()
    while len(stack_BX) > 0 and stack_BX[-1][0] == '':
        stack_BX.pop()
    while len(stack_BY) > 0 and stack_BY[-1][0] == '':
        stack_BY.pop()
    while len(stack_C) > 0 and stack_C[-1][0] == '':
        stack_C.pop()

    return stack_A, stack_BX, stack_BY, stack_C


def simple_compute_entering_time(lane, traffic, W_same, W_diff, prev):
    schedule = []
    prevLane = prev[0]
    prevTime = prev[1]
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


def get_window_by_num(traffic, carNum):
    if carNum >= len(traffic):
        carNum = len(traffic) - 1
    window = traffic[:(carNum+1)]
    del traffic[1:(carNum+1)]
    return window, traffic


def schedule_by_num_window(a_all, b_all, c_all, W_same, W_diff, carNum):
    last_X = ('', 0, 0.0)
    last_Y = ('', 0, 0.0)
    schedule_A = []
    schedule_BX = []
    schedule_BY = []
    schedule_C = []
    # t0 = time.time()

    while len(a_all) > 1 or len(b_all) > 1 or len(c_all) > 1:
        a, a_all = get_window_by_num(a_all, carNum)
        b, b_all = get_window_by_num(b_all, carNum)
        c, c_all = get_window_by_num(c_all, carNum)
        # print(len(a), len(b), len(c))
        if len(a) > 1 and len(b) > 1 and len(c) > 1:
            stack_A, stack_BX, stack_BY, stack_C = dp_compute_entering_time(a, b, c, W_same, W_diff, last_X, last_Y)
            if len(stack_A) > 0 and len(stack_BX) > 0:
                last_X = ('A', stack_A[0].time) if stack_A[0].time >= stack_BX[0].time else ('B', stack_BX[0].time)
            elif len(stack_A) > 0:
                last_X = ('A', stack_A[0].time)
            elif len(stack_BX) > 0:
                last_X = ('B', stack_BX[0].time)
            if len(stack_C) > 0 and len(stack_BY) > 0:
                last_Y = ('C', stack_C[0].time) if stack_C[0].time >= stack_BY[0].time else ('B', stack_BY[0].time)
            elif len(stack_C) > 0:
                last_Y = ('C', stack_C[0].time)
            elif len(stack_BY) > 0:
                last_Y = ('B', stack_BY[0].time)
            stack_A.reverse()
            stack_BX.reverse()
            stack_BY.reverse()
            stack_C.reverse()
            schedule_A += stack_A
            schedule_BX += stack_BX
            schedule_BY += stack_BY
            schedule_C += stack_C
        elif len(a) > 1 and len(b) > 1:
            schedule_A += simple_compute_entering_time(
                'A', a, W_same, W_diff, last_X)
            schedule_BY += simple_compute_entering_time(
                'B', b, W_same, W_diff, last_Y)
        elif len(a) > 1 and len(c) > 1:
            schedule_A += simple_compute_entering_time(
                'A', a, W_same, W_diff, last_X)
            schedule_C += simple_compute_entering_time(
                'C', c, W_same, W_diff, last_Y)
        elif len(b) > 1 and len(c) > 1:
            schedule_BX += simple_compute_entering_time(
                'B', b, W_same, W_diff, last_X)
            schedule_C += simple_compute_entering_time(
                'C', c, W_same, W_diff, last_Y)
        elif len(a) > 1:
            schedule_A += simple_compute_entering_time(
                'A', a, W_same, W_diff, last_X)
        elif len(c) > 1:
            schedule_C += simple_compute_entering_time(
                'C', c, W_same, W_diff, last_Y)
        elif len(b) > 1:
            if last_X[1] < last_Y[1]:
                if last_X[0] == '':
                    schedule_BX.append(Vehicle(b[1].id, b[1].time))
                elif last_X[0] == 'A':
                    schedule_BX.append(
                        Vehicle(b[1].id, max(b[1].time, last_X[1]+W_diff)))
                else:
                    schedule_BX.append(
                        Vehicle(b[1].id, max(b[1].time, last_X[1]+W_same)))
                if len(b) > 2:
                    if last_Y[0] == '':
                        schedule_BY.append(Vehicle(b[2].id, b[2].time))
                    elif last_Y[0] == 'C':
                        schedule_BY.append(
                            Vehicle(b[2].id, max(b[2].time, last_Y[1]+W_diff)))
                    else:
                        schedule_BY.append(
                            Vehicle(b[2].id, max(b[2].time, last_Y[1]+W_same)))
                    for i in range(3, len(b)):
                        if i % 2 == 1:
                            schedule_BX.append(Vehicle(b[i].id, max(
                                b[i].time, schedule_BX[-1].time+W_same)))
                        else:
                            schedule_BY.append(Vehicle(b[i].id, max(
                                b[i].time, schedule_BY[-1].time+W_same)))
            # elif last_Y[1] < last_X[1]:
            else:
                if last_Y[0] == '':
                    schedule_BY.append(Vehicle(b[1].id, b[1].time))
                elif last_Y[0] == 'C':
                    schedule_BY.append(
                        Vehicle(b[1].id, max(b[1].time, last_Y[1]+W_diff)))
                else:
                    schedule_BY.append(
                        Vehicle(b[1].id, max(b[1].time, last_Y[1]+W_same)))
                if len(b) > 2:
                    if last_X[0] == '':
                        schedule_BX.append(Vehicle(b[2].id, b[2].time))
                    elif last_X[0] == 'A':
                        schedule_BX.append(
                            Vehicle(b[2].id, max(b[2].time, last_X[1]+W_diff)))
                    else:
                        schedule_BX.append(
                            Vehicle(b[2].id, max(b[2].time, last_X[1]+W_same)))
                    for i in range(3, len(b)):
                        if i % 2 == 1:
                            schedule_BY.append(Vehicle(b[i].id, max(
                                b[i].time, schedule_BY[-1].time+W_same)))
                        else:
                            schedule_BX.append(Vehicle(b[i].id, max(
                                b[i].time, schedule_BX[-1].time+W_same)))

        # print(f'last_X: {last_X}')
        # print(f'last_Y: {last_Y}')
    # computeTime = time.time() - t0
    # T_last = last_X[1] if last_X[1] >= last_Y[1] else last_Y[1]
    return schedule_A, schedule_BX, schedule_BY, schedule_C


def run(alpha, beta, gamma, W_same, W_diff, windowSize):
    # step = 0
    period = 300
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
    # window_size = 5
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
            passTime_dX = traci.simulation.getTime()
            leaveA = True
            for s in schedule_A:
                if s.id == A_head:
                    schedule_A.remove(s)
                    break
            A_head = A_IDs[0]
        elif len(A_IDs) == 0 and len(A_head) > 0:
            if int(A_head.split('_')[1]) == alpha:
                # print(A_head, "leaves")
                passTime_dX = traci.simulation.getTime()
                leaveA = True
                for s in schedule_A:
                    if s.id == A_head:
                        schedule_A.remove(s)
                        break
                A_head = ""
        if len(B_IDs) > 0 and B_IDs[0] != B_head:
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

        if traci.lanearea.getLastStepVehicleNumber("dA") > 0 or traci.lanearea.getLastStepVehicleNumber("dB") > 0 or traci.lanearea.getLastStepVehicleNumber("dC") > 0:
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
        if countdownX:
            # traci.trafficlight.setPhase("TL1", 16)
            countdownX -= 1
        if countdownY:
            # traci.trafficlight.setPhase("TL1", 18)
            countdownY -= 1

        # Schedule
        if timeStep_cnt - period == 0:
            if traci.lanearea.getLastStepVehicleNumber("dA") > 0 and traci.lanearea.getLastStepVehicleNumber("dB") > 0 and traci.lanearea.getLastStepVehicleNumber("dC") > 0:
                a_all, b_all, c_all = compute_earliest_arrival(
                    laneLength, schedule_A, schedule_BX, schedule_BY, schedule_C)
                # print('a_all', a_all)
                # print('b_all', b_all)
                # print('c_all', c_all)
                schedule_A, schedule_BX, schedule_BY, schedule_C = schedule_by_num_window(
                    a_all, b_all, c_all, W_same, W_diff, windowSize)
                schedule_A.sort(key=lambda x: x[1])
                schedule_BX.sort(key=lambda x: x[1])
                schedule_BY.sort(key=lambda x: x[1])
                schedule_C.sort(key=lambda x: x[1])
                print('s_A', schedule_A)
                print('s_BX', schedule_BX)
                print('s_BY', schedule_BY)
                print('s_C', schedule_C)
                for veh in schedule_BX:
                    try:
                        traci.vehicle.setRouteID(veh.id, "route_1")
                    except:
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
            # step = 0

        # print(f'traffic light state: {gA}, {gBX}, {gBY}, {gC}')
        # print(f'phase: {traci.trafficlight.getPhase("TL1")}')
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
        p = float(sys.argv[1])  # lambda for Poisson distribution
        N = int(sys.argv[2])  # Number of vehicles in each lane
        alpha = N
        beta = N
        gamma = N
        # the waiting time if two consecutive vehicles are from the same lane
        W_same = float(sys.argv[3])
        # the waiting time if two consecutive vehicles are from different lanes
        W_diff = float(sys.argv[4])
        windowSize = int(sys.argv[5])
        isNewTest = sys.argv[6]
    except:
        print('Arguments: lambda, N, W=, W+, windowSize, isNewTest')
        return

    timeStep = 1    # The precision of time (in second)
    pA = p
    pB = p
    pC = p

    if len(sys.argv) > 7:
        inputPath = sys.argv[7]
        outputPath = "output/output" + inputPath[inputPath.find("_"):-8] + "_DP.xml"
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

    run(alpha, beta, gamma, W_same, W_diff, windowSize)


if __name__ == "__main__":
    main()
