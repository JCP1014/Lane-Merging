import numpy as np
import sys
import time
import random
import copy
from collections import namedtuple
Sol = namedtuple("Sol", "time table idx lane")
Lane = namedtuple("Lane", "traffic num num_tmp")


def print_table(L, name):
    print(name)
    for k in range(len(L[0][0])):
        print('k =', k)
        for i in range(len(L)):
            for j in range(len(L[0])):
                print(str(L[i][j][k].time)+'('+L[i][j][k].table +
                      ')'+'('+L[i][j][k].lane+')', end=' ')
            print('')
        print('')


def print_solNum(L, name):
    print(name)
    for k in range(len(L[0][0])):
        print('k =', k)
        for i in range(len(L)):
            for j in range(len(L[0])):
                print(len(L[i][j][k]), end=' ')
            print('')
        print('')


def generate_traffic_v1(timeStep, alpha, beta, gamma, pA, pB, pC):
    a = [0]  # List of earliest arrival time of vehicles in lane A
    b = [0]  # List of earliest arrival time of vehicles in lane B
    c = [0]  # List of earliest arrival time of vehicles in lane C

    alpha_tmp = alpha   # temp variable for countdown
    beta_tmp = beta
    gamma_tmp = gamma

    # Randomly generate earliest arrival time of vehicles in lane A
    t = 1.0  # time(sec)
    while alpha_tmp > 0:
        if np.random.uniform(0, 1) < pA:
            a.append(round(t, 1))
            alpha_tmp -= 1
        t += timeStep
    # Randomly generate earliest arrival time of vehicles in lane B
    t = 1.0
    while beta_tmp > 0:
        if np.random.uniform(0, 1) < pB:
            b.append(round(t, 1))
            beta_tmp -= 1
        t += timeStep
    # Randomly generate earliest arrival time of vehicles in lane C
    t = 1.0
    while gamma_tmp > 0:
        if np.random.uniform(0, 1) < pC:
            c.append(round(t, 1))
            gamma_tmp -= 1
        t += timeStep

    return a, b, c


# v2
def generate_traffic_v2(timeStep, alpha, beta, gamma, p):
    a_all = [0]
    b_all = [0]
    c_all = [0]
    # Randomly generate earliest arrival time
    # p = 1. / 10 # a vehicle is generated every 10 seconds in average.
    alpha_tmp = alpha
    beta_tmp = beta
    gamma_tmp = gamma
    num = alpha_tmp + beta_tmp + gamma_tmp
    t = 1.0
    cars = []
    while num > 0:
        if np.random.uniform(0, 3) < p:
            cars.append(round(t, 1))
            num -= 1
        t += timeStep

    A = Lane(a_all, alpha, alpha_tmp)
    B = Lane(b_all, beta, beta_tmp)
    C = Lane(c_all, gamma, gamma_tmp)
    lanes = [A, B, C]
    laneNum = len(lanes)
    carIndex = 0
    while laneNum > 0:
        try:
            laneIndex = random.randrange(0, laneNum, 1)
        except:
            laneIndex = 0
        lanes[laneIndex].traffic.append(cars[carIndex])
        if len(lanes[laneIndex].traffic) == (lanes[laneIndex].num+1):
            del lanes[laneIndex]
            laneNum -= 1
        carIndex += 1
    # print(a_all)
    # print(b_all)
    # print(c_all)
    return a_all, b_all, c_all


def split_traffic(a, b, c, p):
    gap = (1. / p) * 2
    a_cut = []
    b_cut = []
    c_cut = []

    for i in range(len(a)):
        if a[i+1] - a[i] > gap:
            a_cut.append((a[i], a[i+1]))
    for i in range(len(b)):
        if b[i+1] - b[i] > gap:
            b_cut.append((b[i], b[i+1]))
    for i in range(len(c)):
        if c[i+1] - c[i] > gap:
            c_cut.append((c[i], c[i+1]))
    # print('a_cut', a_cut)
    # print('b_cut', b_cut)
    # print('c_cut', c_cut)


def get_window_by_time(a_all, b_all, c_all, cutTime, keep):
    a = [0]
    b = [0]
    c = [0]
    keepTime = cutTime - keep
    keepNum = 0
    for i in range(1, len(a_all)):
        if a_all[i] <= cutTime:
            a.append(a_all[i])
            if a_all[i] >= keepTime:
                keepNum += 1
        else:
            del a_all[1:(i-keepNum)]
    keepNum = 0
    for i in range(1, len(b_all)):
        if b_all[i] <= cutTime:
            b.append(b_all[i])
            if b_all[i] >= keepTime:
                keepNum += 1
        else:
            del b_all[1:(i-keepNum)]
    keepNum = 0
    for i in range(1, len(c_all)):
        if c_all[i] <= cutTime:
            c.append(c_all[i])
            if c_all[i] >= keepTime:
                keepNum += 1
        else:
            del c_all[1:(i-keepNum)]
    return a, b, c, a_all, b_all, c_all


# def get_window_by_num(a_all, b_all, c_all, carNum, keep):
#     a = a_all[:(carNum+1)]
#     b = b_all[:(carNum+1)]
#     c = c_all[:(carNum+1)]
#     del a_all[1:(carNum+1-keep)]
#     del b_all[1:(carNum+1-keep)]
#     del c_all[1:(carNum+1-keep)]
#     return a, b, c, a_all, b_all, c_all
def get_window_by_num(traffic, carNum, keep):
    if carNum >= len(traffic):
        carNum = len(traffic) - 1
    window = traffic[:(carNum+1)]
    del traffic[1:(carNum+1-keep)]
    return window, traffic


def get_obj(sol):
    return max(sol.time)


def window_oneSol_dp(a, b, c, W_same, W_diff, last_X, last_Y, keep):
    t0 = time.time()
    alpha = len(a) - 1
    beta = len(b) - 1
    gamma = len(c) - 1
    L_AB = [[[Sol((float('inf'), float('inf')), '', 0, '') for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_AC = [[[Sol((float('inf'), float('inf')), '', 0, '') for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_BB = [[[Sol((float('inf'), float('inf')), '', 0, '') for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_BC = [[[Sol((float('inf'), float('inf')), '', 0, '') for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]

    # Initialize
    L_AB[0][0][0] = Sol((last_X[2], last_Y[2]), '', 0, '')
    L_AC[0][0][0] = Sol((last_X[2], last_Y[2]), '', 0, '')
    L_BB[0][0][0] = Sol((last_X[2], last_Y[2]), '', 0, '')
    L_BC[0][0][0] = Sol((last_X[2], last_Y[2]), '', 0, '')

    last_XY = last_X[0] + last_Y[0]
    T_X = last_X[2]
    T_Y = last_Y[2]
    # print('last_XY', last_XY, T_X, T_Y)
    if last_XY == 'AB':
        L_AB[1][1][0] = Sol(
            (max(a[1], T_X+W_same), max(b[1], T_Y+W_same)), 'AB', 0, 'XY')
        L_AC[1][0][1] = Sol(
            (max(a[1], T_X+W_same), max(c[1], T_Y+W_diff)), 'AC', 0, 'XY')
        L_BC[0][1][1] = Sol(
            (max(b[1], T_X+W_diff), max(c[1], T_Y+W_diff)), 'BC', 0, 'XY')
        L_BB[0][1][0] = min(
            Sol((max(b[1], T_X+W_diff), T_Y), 'BB', 0, 'X0'),
            Sol((T_X, max(b[1], T_Y+W_same)), 'BB', 0, '0Y'),
            key=get_obj
        )
        if beta >= 2:
            L_BB[0][2][0] = min(
                Sol((max(b[1], T_X+W_diff), max(b[2], T_Y+W_same)),
                    'BB', 0, 'XY'),
                Sol((max(b[2], T_X+W_diff), max(b[1], T_Y+W_same)),
                    'BB', 0, 'YX'),
                key=get_obj
            )
        L_AB[1][0][0] = Sol((max(a[1], T_X+W_same), T_Y), 'AB', 0, 'X0')
        L_AB[0][1][0] = Sol((T_X, max(b[1], T_Y+W_same)), 'AB', 0, '0Y')
        L_AC[1][0][0] = Sol((max(a[1], T_X+W_same), T_Y), 'AC', 0, 'X0')
        L_AC[0][0][1] = Sol((T_X, max(c[1], T_Y+W_diff)), 'AC', 0, '0Y')
        L_BC[0][1][0] = Sol((max(b[1], T_X+W_diff), T_Y), 'BC', 0, 'X0')
        L_BC[0][0][1] = Sol((T_X, max(c[1], T_Y+W_diff)), 'BC', 0, '0Y')
    elif last_XY == 'AC':
        L_AB[1][1][0] = Sol(
            (max(a[1], T_X+W_same), max(b[1], T_Y+W_diff)), 'AB', 0, 'XY')
        L_AC[1][0][1] = Sol(
            (max(a[1], T_X+W_same), max(c[1], T_Y+W_same)), 'AC', 0, 'XY')
        L_BC[0][1][1] = Sol(
            (max(b[1], T_X+W_diff), max(c[1], T_Y+W_same)), 'BC', 0, 'XY')
        L_BB[0][1][0] = min(
            Sol((max(b[1], T_X+W_diff), T_Y), 'BB', 0, 'X0'),
            Sol((T_X, max(b[1], T_Y+W_diff)), 'BB', 0, '0Y'),
            key=get_obj
        )
        if beta >= 2:
            L_BB[0][2][0] = min(
                Sol((max(b[1], T_X+W_diff), max(b[2], T_Y+W_diff)),
                    'BB', 0, 'XY'),
                Sol((max(b[2], T_X+W_diff), max(b[1], T_Y+W_diff)),
                    'BB', 0, 'YX'),
                key=get_obj
            )
        L_AB[1][0][0] = Sol((max(a[1], T_X+W_same), T_Y), 'AB', 0, 'X0')
        L_AB[0][1][0] = Sol((T_X, max(b[1], T_Y+W_diff)), 'AB', 0, '0Y')
        L_AC[1][0][0] = Sol((max(a[1], T_X+W_same), T_Y), 'AC', 0, 'X0')
        L_AC[0][0][1] = Sol((T_X, max(c[1], T_Y+W_same)), 'AC', 0, '0Y')
        L_BC[0][1][0] = Sol((max(b[1], T_X+W_diff), T_Y), 'BC', 0, 'X0')
        L_BC[0][0][1] = Sol((T_X, max(c[1], T_Y+W_same)), 'BC', 0, '0Y')
    elif last_XY == 'BB':
        L_AB[1][1][0] = Sol(
            (max(a[1], T_X+W_diff), max(b[1], T_Y+W_same)), 'AB', 0, 'XY')
        L_AC[1][0][1] = Sol(
            (max(a[1], T_X+W_diff), max(c[1], T_Y+W_diff)), 'AC', 0, 'XY')
        L_BC[0][1][1] = Sol(
            (max(b[1], T_X+W_same), max(c[1], T_Y+W_diff)), 'BC', 0, 'XY')
        L_BB[0][1][0] = min(
            Sol((max(b[1], T_X+W_same), T_Y), 'BB', 0, 'X0'),
            Sol((T_X, max(b[1], T_Y+W_same)), 'BB', 0, '0Y'),
            key=get_obj
        )
        if beta >= 2:
            L_BB[0][2][0] = min(
                Sol((max(b[1], T_X+W_same), max(b[2], T_Y+W_same)),
                    'BB', 0, 'XY'),
                Sol((max(b[2], T_X+W_same), max(b[1], T_Y+W_same)),
                    'BB', 0, 'YX'),
                key=get_obj
            )
        L_AB[1][0][0] = Sol((max(a[1], T_X+W_diff), T_Y), 'AB', 0, 'X0')
        L_AB[0][1][0] = Sol((T_X, max(b[1], T_Y+W_same)), 'AB', 0, '0Y')
        L_AC[1][0][0] = Sol((max(a[1], T_X+W_diff), T_Y), 'AC', 0, 'X0')
        L_AC[0][0][1] = Sol((T_X, max(c[1], T_Y+W_diff)), 'AC', 0, '0Y')
        L_BC[0][1][0] = Sol((max(b[1], T_X+W_same), T_Y), 'BC', 0, 'X0')
        L_BC[0][0][1] = Sol((T_X, max(c[1], T_Y+W_diff)), 'BC', 0, '0Y')
    elif last_XY == 'BC':
        L_AB[1][1][0] = Sol(
            (max(a[1], T_X+W_diff), max(b[1], T_Y+W_diff)), 'AB', 0, 'XY')
        L_AC[1][0][1] = Sol(
            (max(a[1], T_X+W_diff), max(c[1], T_Y+W_same)), 'AC', 0, 'XY')
        L_BC[0][1][1] = Sol(
            (max(b[1], T_X+W_same), max(c[1], T_Y+W_same)), 'BC', 0, 'XY')
        L_BB[0][1][0] = min(
            Sol((max(b[1], T_X+W_same), T_Y), 'BB', 0, 'X0'),
            Sol((T_X, max(b[1], T_Y+W_diff)), 'BB', 0, '0Y'),
            key=get_obj
        )
        if beta >= 2:
            L_BB[0][2][0] = min(
                Sol((max(b[1], T_X+W_same), max(b[2], T_Y+W_diff)),
                    'BB', 0, 'XY'),
                Sol((max(b[2], T_X+W_same), max(b[1], T_Y+W_diff)),
                    'BB', 0, 'YX'),
                key=get_obj
            )
        L_AB[1][0][0] = Sol((max(a[1], T_X+W_diff), T_Y), 'AB', 0, 'X0')
        L_AB[0][1][0] = Sol((T_X, max(b[1], T_Y+W_diff)), 'AB', 0, '0Y')
        L_AC[1][0][0] = Sol((max(a[1], T_X+W_diff), T_Y), 'AC', 0, 'X0')
        L_AC[0][0][1] = Sol((T_X, max(c[1], T_Y+W_same)), 'AC', 0, '0Y')
        L_BC[0][1][0] = Sol((max(b[1], T_X+W_same), T_Y), 'BC', 0, 'X0')
        L_BC[0][0][1] = Sol((T_X, max(c[1], T_Y+W_same)), 'BC', 0, '0Y')
    else:
        L_AB[1][1][0] = Sol((a[1], b[1]), 'AB', 0, 'XY')
        L_AC[1][0][1] = Sol((a[1], c[1]), 'AC', 0, 'XY')
        L_BC[0][1][1] = Sol((b[1], c[1]), 'BC', 0, 'XY')
        L_BB[0][1][0] = Sol((-W_diff, b[1]), 'BB', 0,
                            '0Y') if a[1] <= c[1] else Sol((b[1], -W_diff), 'BB', 0, 'X0')
        if beta >= 2:
            L_BB[0][2][0] = Sol((b[1], b[2]), 'BB', 0, 'XY') if a[1] <= c[1] else Sol(
                (b[2], b[1]), 'BB', 0, 'YX')
        L_AB[1][0][0] = Sol((a[1], -W_diff), 'AB', 0, 'X0')
        L_AB[0][1][0] = Sol((-W_diff, b[1]), 'AB', 0, '0Y')
        L_AC[1][0][0] = Sol((a[1], -W_diff), 'AC', 0, 'X0')
        L_AC[0][0][1] = Sol((-W_diff, c[1]), 'AC', 0, '0Y')
        L_BC[0][1][0] = Sol((b[1], -W_diff), 'BC', 0, 'X0')
        L_BC[0][0][1] = Sol((-W_diff, c[1]), 'BC', 0, '0Y')
        # if beta >= 3:
        #     L_BB[0][3][0] = min(
        #                         Sol((b[1], max(b[3], b[2]+W_same)), 'BB', 'YY'),
        #                         Sol((max(b[3], b[1]+W_same), b[2]), 'BB', 'YX'),
        #                         Sol((max(b[2], b[1]+W_same), b[3]), 'BB', 'XY'),
        #                         key=get_obj
        #     )
        #     if L_BB[0][3][0][0] > L_BB[0][3][0][1]:
        #         if a[1] < c[1]:
        #             L_BB[0][3][0] = L_BB[0][3][0][::-1]
        #     elif L_BB[0][3][0][0] < L_BB[0][3][0][1]:
        #         if c[1] < a[1]:
        #             L_BB[0][3][0] = L_BB[0][3][0][::-1]
    for i in range(2, alpha+1):
        L_AB[i][1][0] = Sol(
            (max(a[i], L_AB[i-1][1][0].time[0]+W_same), L_AB[i-1][1][0].time[1]), 'AB', 0, 'XY')
    for i in range(2, alpha+1):
        L_AC[i][0][1] = Sol(
            (max(a[i], L_AC[i-1][0][1].time[0]+W_same), L_AC[i-1][0][1].time[1]), 'AC', 0, 'XY')
    for k in range(2, gamma+1):
        L_AC[1][0][k] = Sol(
            (L_AC[1][0][k-1].time[0], max(c[k], L_AC[1][0][k-1].time[1]+W_same)), 'AC', 0, 'XY')
    for k in range(2, gamma+1):
        L_BC[0][1][k] = Sol(
            (L_BC[0][1][k-1].time[0], max(c[k], L_BC[0][1][k-1].time[1]+W_same)), 'BC', 0, 'XY')
    if beta >= 3:
        for j in range(3, beta+1):
            L_BB[0][j][0] = min(
                Sol((max(b[j-1], L_BB[0][j-2][0].time[0]+W_same),
                     max(b[j], L_BB[0][j-2][0].time[1]+W_same)), 'BB', 0, 'XY'),
                Sol((max(b[j], L_BB[0][j-2][0].time[0]+W_same),
                     max(b[j-1], L_BB[0][j-2][0].time[1]+W_same)), 'BB', 0, 'YX'),
                Sol((max(b[j], max(b[j-1], L_BB[0][j-2][0].time[0]+W_same) +
                         W_same), L_BB[0][j-2][0].time[1]), 'BB', 0, 'XX'),
                Sol((L_BB[0][j-2][0].time[0], max(b[j], max(b[j-1],
                                                            L_BB[0][j-2][0].time[1]+W_same)+W_same)), 'BB', 0, 'YY'),
                key=get_obj
            )
            # if L_BB[0][j][0][0] > L_BB[0][j][0][1]:
            #     if a[1] < c[1]:
            #         L_BB[0][j][0] = L_BB[0][j][0][::-1]
            # elif L_BB[0][j][0][0] < L_BB[0][j][0][1]:
            #     if c[1] < a[1]:
            #         L_BB[0][j][0] = L_BB[0][j][0][::-1]
    for i in range(2, alpha+1):
        L_AB[i][0][0] = Sol(
            (max(a[i], L_AB[i-1][0][0].time[0]+W_same), L_AB[i-1][0][0].time[1]), 'AB', 0, 'X0')
    for j in range(2, beta+1):
        L_AB[0][j][0] = Sol(
            (L_AB[0][j-1][0].time[0], max(b[j], L_AB[0][j-1][0].time[1]+W_same)), 'AB', 0, '0Y')
    for i in range(2, alpha+1):
        L_AC[i][0][0] = Sol(
            (max(a[i], L_AC[i-1][0][0].time[0]+W_same), L_AC[i-1][0][0].time[1]), 'AC', 0, 'X0')
    for k in range(2, gamma+1):
        L_AC[0][0][k] = Sol(
            (L_AC[0][0][k-1].time[0], max(c[k], L_AC[0][0][k-1].time[1]+W_same)), 'AC', 0, '0Y')
    for j in range(2, beta+1):
        L_BC[0][j][0] = Sol(
            (max(b[j], L_BC[0][j-1][0].time[0]+W_same), L_BC[0][j-1][0].time[1]), 'BC', 0, 'X0')
    for k in range(2, gamma+1):
        L_BC[0][0][k] = Sol(
            (L_BC[0][0][k-1].time[0], max(c[k], L_BC[0][0][k-1].time[1]+W_same)), 'BC', 0, '0Y')

    if beta >= 2:
        if last_XY == 'AB':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = min(
                    Sol((max(b[1], L_AC[i][0][0].time[0]+W_diff),
                         max(b[2], T_Y+W_same)), 'AC', 0, 'XY'),
                    Sol((max(b[2], L_AC[i][0][0].time[0]+W_diff),
                         max(b[1], T_Y+W_same)), 'AC', 0, 'YX'),
                    key=get_obj
                )
            for k in range(1, gamma+1):
                L_BB[0][2][k] = min(
                    Sol((max(b[1], T_X+W_diff), max(b[2], L_AC[0]
                                                    [0][k].time[1]+W_diff)), 'AC', 0, 'XY'),
                    Sol((max(b[2], T_X+W_diff), max(b[1], L_AC[0]
                                                    [0][k].time[1]+W_diff)), 'AC', 0, 'YX'),
                    key=get_obj
                )
        elif last_XY == 'AC':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = min(
                    Sol((max(b[1], L_AC[i][0][0].time[0]+W_diff),
                         max(b[2], T_Y+W_diff)), 'AC', 0, 'XY'),
                    Sol((max(b[2], L_AC[i][0][0].time[0]+W_diff),
                         max(b[1], T_Y+W_diff)), 'AC', 0, 'YX'),
                    key=get_obj
                )
            for k in range(1, gamma+1):
                L_BB[0][2][k] = min(
                    Sol((max(b[1], T_X+W_diff), max(b[2], L_AC[0]
                                                    [0][k].time[1]+W_diff)), 'AC', 0, 'XY'),
                    Sol((max(b[2], T_X+W_diff), max(b[1], L_AC[0]
                                                    [0][k].time[1]+W_diff)), 'AC', 0, 'YX'),
                    key=get_obj
                )
        elif last_XY == 'BB':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = min(
                    Sol((max(b[1], L_AC[i][0][0].time[0]+W_diff),
                         max(b[2], T_Y+W_same)), 'AC', 0, 'XY'),
                    Sol((max(b[2], L_AC[i][0][0].time[0]+W_diff),
                         max(b[1], T_Y+W_same)), 'AC', 0, 'YX'),
                    key=get_obj
                )
            for k in range(1, gamma+1):
                L_BB[0][2][k] = min(
                    Sol((max(b[1], T_X+W_same), max(b[2], L_AC[0]
                                                    [0][k].time[1]+W_diff)), 'AC', 0, 'XY'),
                    Sol((max(b[2], T_X+W_same), max(b[1], L_AC[0]
                                                    [0][k].time[1]+W_diff)), 'AC', 0, 'YX'),
                    key=get_obj
                )
        elif last_XY == 'BC':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = min(
                    Sol((max(b[1], L_AC[i][0][0].time[0]+W_diff),
                         max(b[2], T_Y+W_diff)), 'AC', 0, 'XY'),
                    Sol((max(b[2], L_AC[i][0][0].time[0]+W_diff),
                         max(b[1], T_Y+W_diff)), 'AC', 0, 'YX'),
                    key=get_obj
                )
            for k in range(1, gamma+1):
                L_BB[0][2][k] = min(
                    Sol((max(b[1], T_X+W_same), max(b[2], L_AC[0]
                                                    [0][k].time[1]+W_diff)), 'AC', 0, 'XY'),
                    Sol((max(b[2], T_X+W_same), max(b[1], L_AC[0]
                                                    [0][k].time[1]+W_diff)), 'AC', 0, 'YX'),
                    key=get_obj
                )
        else:
            for i in range(1, alpha+1):
                L_BB[i][2][0] = min(
                    Sol((max(b[1], L_AC[i][0][0].time[0]+W_diff),
                         b[2]), 'AC', 0, 'XY'),
                    Sol((max(b[2], L_AC[i][0][0].time[0]+W_diff),
                         b[1]), 'AC', 0, 'YX'),
                    key=get_obj
                )
            for k in range(1, gamma+1):
                L_BB[0][2][k] = min(
                    Sol((b[1], max(b[2], L_AC[0][0][k].time[1]+W_diff)),
                        'AC', 0, 'XY'),
                    Sol((b[2], max(b[1], L_AC[0][0][k].time[1]+W_diff)),
                        'AC', 0, 'YX'),
                    key=get_obj
                )

    for i in range(1, alpha+1):
        for j in range(2, beta+1):
            L_AB[i][j][0] = min(
                Sol((max(a[i], L_AB[i-1][j-1][0].time[0]+W_same),
                     max(b[j], L_AB[i-1][j-1][0].time[1]+W_same)), 'AB', 0, 'XY'),
                Sol((max(a[i], max(b[j], L_AB[i-1][j-1][0].time[0]+W_diff) +
                         W_diff), L_AB[i-1][j-1][0].time[1]), 'AB', 0, 'XX'),
                Sol((max(a[i], L_BB[i-1][j-1][0].time[0]+W_diff),
                     max(b[j], L_BB[i-1][j-1][0].time[1]+W_same)), 'BB', 0, 'XY'),
                Sol((max(a[i], max(b[j], L_BB[i-1][j-1][0].time[0]+W_same) +
                         W_diff), L_BB[i-1][j-1][0].time[1]), 'BB', 0, 'XX'),
                key=get_obj
            )
    for i in range(2, alpha+1):
        for k in range(2, gamma+1):
            L_AC[i][0][k] = Sol((max(a[i], L_AC[i-1][0][k-1].time[0]+W_same),
                                 max(c[k], L_AC[i-1][0][k-1].time[1]+W_same)), 'AC', 0, 'XY')
    for j in range(2, beta+1):
        for k in range(1, gamma+1):
            L_BC[0][j][k] = min(
                Sol((max(b[j], L_BB[0][j-1][k-1].time[0]+W_same),
                     max(c[k], L_BB[0][j-1][k-1].time[1]+W_diff)), 'BB', 0, 'XY'),
                Sol((L_BB[0][j-1][k-1].time[0], max(c[k], max(b[j], L_BB[0]
                                                              [j-1][k-1].time[1]+W_same)+W_diff)), 'BB', 0, 'YY'),
                Sol((max(b[j], L_BC[0][j-1][k-1].time[0]+W_same),
                     max(c[k], L_BC[0][j-1][k-1].time[1]+W_same)), 'BC', 0, 'XY'),
                Sol((L_BC[0][j-1][k-1].time[0], max(c[k], max(b[j], L_BC[0]
                                                              [j-1][k-1].time[1]+W_diff)+W_diff)), 'BC', 0, 'YY'),
                key=get_obj
            )
    for i in range(1, alpha+1):
        for j in range(3, beta+1):
            L_BB[i][j][0] = min(
                Sol((max(b[j-1], L_AB[i][j-2][0].time[0]+W_diff),
                     max(b[j], L_AB[i][j-2][0].time[1]+W_same)), 'AB', 0, 'XY'),
                Sol((max(b[j], L_AB[i][j-2][0].time[0]+W_diff),
                     max(b[j-1], L_AB[i][j-2][0].time[1]+W_same)), 'AB', 0, 'YX'),
                Sol((max(b[j-1], L_BB[i][j-2][0].time[0]+W_same),
                     max(b[j], L_BB[i][j-2][0].time[1]+W_same)), 'BB', 0, 'XY'),
                Sol((max(b[j], L_BB[i][j-2][0].time[0]+W_same),
                     max(b[j-1], L_BB[i][j-2][0].time[1]+W_same)), 'BB', 0, 'YX'),
                Sol((max(b[j], max(b[j-1], L_BB[i][j-2][0].time[0]+W_same) +
                         W_same), L_BB[i][j-2][0].time[1]), 'BB', 0, 'XX'),
                Sol((L_BB[i][j-2][0].time[0], max(b[j], max(b[j-1],
                                                            L_BB[i][j-2][0].time[1]+W_same)+W_same)), 'BB', 0, 'YY'),
                key=get_obj
            )

    for i in range(1, alpha+1):
        for j in range(1, beta+1):
            for k in range(1, gamma+1):
                L_AB[i][j][k] = min(
                    Sol((max(a[i], L_AB[i-1][j-1][k].time[0]+W_same),
                         max(b[j], L_AB[i-1][j-1][k].time[1]+W_same)), 'AB', 0, 'XY'),
                    Sol((max(a[i], max(b[j], L_AB[i-1][j-1][k].time[0]+W_diff) +
                             W_diff), L_AB[i-1][j-1][k].time[1]), 'AB', 0, 'XX'),
                    Sol((max(a[i], L_AC[i-1][j-1][k].time[0]+W_same),
                         max(b[j], L_AC[i-1][j-1][k].time[1]+W_diff)), 'AC', 0, 'XY'),
                    Sol((max(a[i], L_BB[i-1][j-1][k].time[0]+W_diff),
                         max(b[j], L_BB[i-1][j-1][k].time[1]+W_same)), 'BB', 0, 'XY'),
                    Sol((max(a[i], max(b[j], L_BB[i-1][j-1][k].time[0]+W_same) +
                             W_diff), L_BB[i-1][j-1][k].time[1]), 'BB', 0, 'XX'),
                    Sol((max(a[i], L_BC[i-1][j-1][k].time[0]+W_diff),
                         max(b[j], L_BC[i-1][j-1][k].time[1]+W_diff)), 'BC', 0, 'XY'),
                    key=get_obj
                )
                L_AC[i][j][k] = min(
                    Sol((max(a[i], L_AB[i-1][j][k-1].time[0]+W_same),
                         max(c[k], L_AB[i-1][j][k-1].time[1]+W_diff)), 'AB', 0, 'XY'),
                    Sol((max(a[i], L_AC[i-1][j][k-1].time[0]+W_same),
                         max(c[k], L_AC[i-1][j][k-1].time[1]+W_same)), 'AC', 0, 'XY'),
                    Sol((max(a[i], L_BB[i-1][j][k-1].time[0]+W_diff),
                         max(c[k], L_BB[i-1][j][k-1].time[1]+W_diff)), 'BB', 0, 'XY'),
                    Sol((max(a[i], L_BC[i-1][j][k-1].time[0]+W_diff),
                         max(c[k], L_BC[i-1][j][k-1].time[1]+W_same)), 'BC', 0, 'XY'),
                    key=get_obj
                )
                L_BC[i][j][k] = min(
                    Sol((max(b[j], L_AB[i][j-1][k-1].time[0]+W_diff),
                         max(c[k], L_AB[i][j-1][k-1].time[1]+W_diff)), 'AB', 0, 'XY'),
                    Sol((max(b[j], L_AC[i][j-1][k-1].time[0]+W_diff),
                         max(c[k], L_AC[i][j-1][k-1].time[1]+W_same)), 'AC', 0, 'XY'),
                    Sol((max(b[j], L_BB[i][j-1][k-1].time[0]+W_same),
                         max(c[k], L_BB[i][j-1][k-1].time[1]+W_diff)), 'BB', 0, 'XY'),
                    Sol((L_BB[i][j-1][k-1].time[0], max(c[k], max(b[j], L_BB[i]
                                                                  [j-1][k-1].time[1]+W_same)+W_diff)), 'BB', 0, 'YY'),
                    Sol((max(b[j], L_BC[i][j-1][k-1].time[0]+W_same),
                         max(c[k], L_BC[i][j-1][k-1].time[1]+W_same)), 'BC', 0, 'XY'),
                    Sol((L_BC[i][j-1][k-1].time[0], max(c[k], max(b[j], L_BC[i]
                                                                  [j-1][k-1].time[1]+W_diff)+W_diff)), 'BC', 0, 'YY'),
                    key=get_obj
                )
                if j >= 2:
                    L_BB[i][j][k] = min(
                        Sol((max(b[j-1], L_AB[i][j-2][k].time[0]+W_diff),
                             max(b[j], L_AB[i][j-2][k].time[1]+W_same)), 'AB', 0, 'XY'),
                        Sol((max(b[j], L_AB[i][j-2][k].time[0]+W_diff),
                             max(b[j-1], L_AB[i][j-2][k].time[1]+W_same)), 'AB', 0, 'YX'),
                        Sol((max(b[j], max(b[j-1], L_AB[i][j-2][k].time[0]+W_diff) +
                                 W_same), L_AB[i][j-2][k].time[1]), 'AB', 0, 'XX'),
                        Sol((max(b[j-1], L_AC[i][j-2][k].time[0]+W_diff),
                             max(b[j], L_AC[i][j-2][k].time[1]+W_diff)), 'AC', 0, 'XY'),
                        Sol((max(b[j], L_AC[i][j-2][k].time[0]+W_diff),
                             max(b[j-1], L_AC[i][j-2][k].time[1]+W_diff)), 'AC', 0, 'YX'),
                        Sol((max(b[j-1], L_BB[i][j-2][k].time[0]+W_same),
                             max(b[j], L_BB[i][j-2][k].time[1]+W_same)), 'BB', 0, 'XY'),
                        Sol((max(b[j], L_BB[i][j-2][k].time[0]+W_same),
                             max(b[j-1], L_BB[i][j-2][k].time[1]+W_same)), 'BB', 0, 'YX'),
                        Sol((max(b[j], max(b[j-1], L_BB[i][j-2][k].time[0]+W_same) +
                                 W_same), L_BB[i][j-2][k].time[1]), 'BB', 0, 'XX'),
                        Sol((L_BB[i][j-2][k].time[0], max(b[j], max(b[j-1],
                                                                    L_BB[i][j-2][k].time[1]+W_same)+W_same)), 'BB', 0, 'YY'),
                        Sol((max(b[j-1], L_BC[i][j-2][k].time[0]+W_same),
                             max(b[j], L_BC[i][j-2][k].time[1]+W_diff)), 'BC', 0, 'XY'),
                        Sol((max(b[j], L_BC[i][j-2][k].time[0]+W_same),
                             max(b[j-1], L_BC[i][j-2][k].time[1]+W_diff)), 'BC', 0, 'YX'),
                        Sol((L_BC[i][j-2][k].time[0], max(b[j], max(b[j-1],
                                                                    L_BC[i][j-2][k].time[1]+W_diff)+W_same)), 'BC', 0, 'YY'),
                        key=get_obj
                    )
    
    # print_table(L_AB, 'L_AB')
    # print_table(L_AC, 'L_AC')
    # print_table(L_BB, 'L_BB')
    # print_table(L_BC, 'L_BC')

    # Push order to stack
    stack_X = []
    stack_Y = []
    i = alpha
    j = beta
    k = gamma
    # Choose the optimal solution and the table to start backtracking
    opt = min(L_AB[i][j][k], L_AC[i][j][k], L_BB[i]
              [j][k], L_BC[i][j][k], key=get_obj)
    table = ''
    lanes = ''
    print('opt',opt)
    if opt.time == L_AB[i][j][k].time:
        print('AB')
        table = L_AB[i][j][k].table
        lanes = L_AB[i][j][k].lane
        if lanes == 'XY':
            stack_X.append(('A', i, L_AB[i][j][k].time[0]))
            stack_Y.append(('B', j, L_AB[i][j][k].time[1]))
            i -= 1
            j -= 1
        elif lanes == 'XX':
            stack_X.append(('A', i, L_AB[i][j][k].time[0]))
            if table == 'AB':
                stack_X.append(
                    ('B', j, max(b[j], L_AB[i-1][j-1][k].time[0]+W_diff)))
            elif table == 'BB':
                stack_X.append(
                    ('B', j, max(b[j], L_BB[i-1][j-1][k].time[0]+W_same)))
            else:
                print('bug')
            i -= 1
            j -= 1
        elif lanes[0] == 'X':
            stack_X.append(('A', i, L_AB[i][j][k].time[0]))
            i -= 1
        elif lanes[1] == 'Y':
            stack_Y.append(('B', j, L_AB[i][j][k].time[1]))
            j -= 1
    elif opt.time == L_AC[i][j][k].time:
        print('AC')
        table = L_AC[i][j][k].table
        lanes = L_AC[i][j][k].lane
        if lanes == 'XY':
            stack_X.append(('A', i, L_AC[i][j][k].time[0]))
            stack_Y.append(('C', k, L_AC[i][j][k].time[1]))
            i -= 1
            k -= 1
        elif lanes[0] == 'X':
            stack_X.append(('A', i, L_AC[i][j][k].time[0]))
            i -= 1
        elif lanes[1] == 'Y':
            stack_Y.append(('C', k, L_AC[i][j][k].time[1]))
            k -= 1
    elif opt.time == L_BB[i][j][k].time:
        print('BB')
        table = L_BB[i][j][k].table
        lanes = L_BB[i][j][k].lane
        if lanes == 'XY':
            stack_X.append(('B', j-1, L_BB[i][j][k].time[0]))
            stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
            j -= 2
        elif lanes == 'YX':
            stack_Y.append(('B', j-1, L_BB[i][j][k].time[1]))
            stack_X.append(('B', j, L_BB[i][j][k].time[0]))
            j -= 2
        elif lanes == 'XX':
            stack_X.append(('B', j, L_BB[i][j][k].time[0]))
            if table == 'AB':
                stack_X.append(('B', j-1, max(b[j-1], L_AB[i][j-2][k].time[0]+W_diff)))
            elif table == 'BB':
                stack_X.append(('B', j-1, max(b[j-1], L_BB[i][j-2][k].time[0]+W_same)))
            j -= 2
        elif lanes == 'YY':
            stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
            if table == 'BB':
                stack_Y.append(
                    ('B', j-1, max(b[j-1], L_BB[i][j-2][k].time[1]+W_same)))
            elif table == 'BC':
                stack_Y.append(
                    ('B', j-1, max(b[j-1], L_BC[i][j-2][k].time[1]+W_same)))
            else:
                print('bug')
            j -= 2
        elif lanes[0] == 'X':
            stack_X.append(('B', j, L_BB[i][j][k].time[0]))
            j -= 1
        elif lanes[1] == 'Y':
            stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
            j -= 1
    elif opt.time == L_BC[i][j][k].time:
        print('BC')
        table = L_BC[i][j][k].table
        lanes = L_BC[i][j][k].lane
        if lanes == 'XY':
            stack_X.append(('B', j, L_BC[i][j][k].time[0]))
            stack_Y.append(('C', k, L_BC[i][j][k].time[1]))
            j -= 1
            k -= 1
        elif lanes == 'YY':
            stack_Y.append(('C', k, L_BC[i][j][k].time[1]))
            if table == 'BB':
                stack_Y.append(
                    ('B', j, max(b[j], L_BB[i][j-1][k-1].time[1]+W_same)))
            elif table == 'BC':
                stack_Y.append(
                    ('B', j, max(b[j], L_BC[i][j-1][k-1].time[1]+W_diff)))
            else:
                print('bug')
            j -= 1
            k -= 1
        elif lanes[0] == 'X':
            stack_X.append(('B', j, L_BC[i][j][k].time[0]))
            j -= 1
        elif lanes[1] == 'Y':
            stack_Y.append(('C', k, L_BC[i][j][k].time[1]))
            k -= 1

    # Backtracking
    while i > 0 or j > 0 or k > 0:
        print(table, i, j, k)
        if table == 'AB':
            print('AB')
            table = L_AB[i][j][k].table
            lanes = L_AB[i][j][k].lane
            if lanes == 'XY':
                stack_X.append(('A', i, L_AB[i][j][k].time[0]))
                stack_Y.append(('B', j, L_AB[i][j][k].time[1]))
                i -= 1
                j -= 1
            elif lanes == 'XX':
                stack_X.append(('A', i, L_AB[i][j][k].time[0]))
                if table == 'AB':
                    stack_X.append(
                        ('B', j, max(b[j], L_AB[i-1][j-1][k].time[0]+W_diff)))
                elif table == 'BB':
                    stack_X.append(
                        ('B', j, max(b[j], L_BB[i-1][j-1][k].time[0]+W_same)))
                else:
                    print('bug')
                i -= 1
                j -= 1
            elif lanes[0] == 'X':
                stack_X.append(('A', i, L_AB[i][j][k].time[0]))
                i -= 1
            elif lanes[1] == 'Y':
                stack_Y.append(('B', j, L_AB[i][j][k].time[1]))
                j -= 1
        elif table == 'AC':
            print('AC')
            table = L_AC[i][j][k].table
            lanes = L_AC[i][j][k].lane
            if lanes == 'XY':
                stack_X.append(('A', i, L_AC[i][j][k].time[0]))
                stack_Y.append(('C', k, L_AC[i][j][k].time[1]))
                i -= 1
                k -= 1
            elif lanes[0] == 'X':
                stack_X.append(('A', i, L_AC[i][j][k].time[0]))
                i -= 1
            elif lanes[1] == 'Y':
                stack_Y.append(('C', k, L_AC[i][j][k].time[1]))
                k -= 1
        elif table == 'BB':
            print('BB')
            table = L_BB[i][j][k].table
            lanes = L_BB[i][j][k].lane
            if lanes == 'XY':
                stack_X.append(('B', j-1, L_BB[i][j][k].time[0]))
                stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
                j -= 2
            elif lanes == 'YX':
                stack_Y.append(('B', j-1, L_BB[i][j][k].time[1]))
                stack_X.append(('B', j, L_BB[i][j][k].time[0]))
                j -= 2
            elif lanes == 'XX':
                stack_X.append(('B', j, L_BB[i][j][k].time[0]))
                if table == 'AB':
                    stack_X.append(('B', j-1, max(b[j-1], L_AB[i][j-2][k].time[0]+W_diff)))
                elif table == 'BB':
                    stack_X.append(('B', j-1, max(b[j-1], L_BB[i][j-2][k].time[0]+W_same)))
                j -= 2
            elif lanes == 'YY':
                stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
                if table == 'BB':
                    stack_Y.append(
                        ('B', j-1, max(b[j-1], L_BB[i][j-2][k].time[1]+W_same)))
                elif table == 'BC':
                    stack_Y.append(
                        ('B', j-1, max(b[j-1], L_BC[i][j-2][k].time[1]+W_same)))
                else:
                    print('bug')
                j -= 2
            elif lanes[0] == 'X':
                stack_X.append(('B', j, L_BB[i][j][k].time[0]))
                j -= 1
            elif lanes[1] == 'Y':
                stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
                j -= 1
        elif table == 'BC':
            print('BC')
            table = L_BC[i][j][k].table
            lanes = L_BC[i][j][k].lane
            if lanes == 'XY':
                stack_X.append(('B', j, L_BC[i][j][k].time[0]))
                stack_Y.append(('C', k, L_BC[i][j][k].time[1]))
                j -= 1
                k -= 1
            elif lanes == 'YY':
                stack_Y.append(('C', k, L_BC[i][j][k].time[1]))
                if table == 'BB':
                    stack_Y.append(
                        ('B', j, max(b[j], L_BB[i][j-1][k-1].time[1]+W_same)))
                elif table == 'BC':
                    stack_Y.append(
                        ('B', j, max(b[j], L_BC[i][j-1][k-1].time[1]+W_diff)))
                else:
                    print('bug')
                j -= 1
                k -= 1
            elif lanes[0] == 'X':
                stack_X.append(('B', j, L_BC[i][j][k].time[0]))
                j -= 1
            elif lanes[1] == 'Y':
                stack_Y.append(('C', k, L_BC[i][j][k].time[1]))
                k -= 1

    T_last = stack_X[0][2] if stack_X[0][2] >= stack_Y[0][2] else stack_Y[0][2]

    # Delete the redundant element (i==0 or j==0 or k==0)
    while stack_X[-1][1] <= 0:
        stack_X.pop()
    while stack_Y[-1][1] <= 0:
        stack_Y.pop()

    # Output order
    print('lane X:')
    last_X = stack_X[0]
    while len(stack_X) > 0:
        print(stack_X.pop())
    print('-----------------------')
    print('lane Y:')
    last_Y = stack_Y[0]
    while len(stack_Y) > 0:
        print(stack_Y.pop())

    computeTime = time.time()-t0
    return last_X, last_Y, computeTime


# Three-dimensional DP which keeps multiple solutions in each square
def window_allSol_dp(a, b, c, W_same, W_diff, last_X, last_Y, keep):
    t0 = time.time()
    alpha = len(a) - 1  # Number of vehicles in lane A
    beta = len(b) - 1   # Number of vehicles in lane B
    gamma = len(c) - 1  # Number of vehicles in lane C

    # The three-dimensional DP table,
    # the first character after the underscore means which lane that the last vehicle going to lane X is from, and
    # the second character after the underscore means which lane that the last vehicle going to lane X is from.
    L_AB = [[[[Sol((float('inf'), float('inf')), '', 0, '')] for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_AC = [[[[Sol((float('inf'), float('inf')), '', 0, '')] for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_BB = [[[[Sol((float('inf'), float('inf')), '', 0, '')] for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_BC = [[[[Sol((float('inf'), float('inf')), '', 0, '')] for k in range(
        gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]

    # Initialize
    L_AB[0][0][0] = [Sol((last_X[2], last_Y[2]), '', 0, '')]
    L_AC[0][0][0] = [Sol((last_X[2], last_Y[2]), '', 0, '')]
    L_BB[0][0][0] = [Sol((last_X[2], last_Y[2]), '', 0, '')]
    L_BC[0][0][0] = [Sol((last_X[2], last_Y[2]), '', 0, '')]

    last_XY = last_X[0] + last_Y[0]
    T_X = last_X[2]
    T_Y = last_Y[2]
    # print('last_XY', last_XY, T_X, T_Y)
    if last_XY == 'AB':
        L_AB[1][1][0] = [
            Sol((max(a[1], T_X+W_same), max(b[1], T_Y+W_same)), 'AB', 0, 'XY')]
        L_AC[1][0][1] = [
            Sol((max(a[1], T_X+W_same), max(c[1], T_Y+W_diff)), 'AC', 0, 'XY')]
        L_BC[0][1][1] = [
            Sol((max(b[1], T_X+W_diff), max(c[1], T_Y+W_diff)), 'BC', 0, 'XY')]
        L_BB[0][1][0] = [Sol((max(b[1], T_X+W_diff), T_Y), 'BB', 0, 'X0'),
                         Sol((T_X, max(b[1], T_Y+W_same)), 'BB', 0, '0Y')]
        if beta >= 2:
            L_BB[0][2][0] = [Sol((max(b[1], T_X+W_diff), max(b[2], T_Y+W_same)), 'BB', 0, 'XY'),
                             Sol((max(b[2], T_X+W_diff), max(b[1], T_Y+W_same)), 'BB', 0, 'YX')]

        L_AB[1][0][0] = [Sol((max(a[1], T_X+W_same), T_Y), 'AB', 0, 'X0')]
        L_AB[0][1][0] = [Sol((T_X, max(b[1], T_Y+W_same)), 'AB', 0, '0Y')]
        L_AC[1][0][0] = [Sol((max(a[1], T_X+W_same), T_Y), 'AC', 0, 'X0')]
        L_AC[0][0][1] = [Sol((T_X, max(c[1], T_Y+W_diff)), 'AC', 0, '0Y')]
        L_BC[0][1][0] = [Sol((max(b[1], T_X+W_diff), T_Y), 'BC', 0, 'X0')]
        L_BC[0][0][1] = [Sol((T_X, max(c[1], T_Y+W_diff)), 'BC', 0, '0Y')]
    elif last_XY == 'AC':
        L_AB[1][1][0] = [
            Sol((max(a[1], T_X+W_same), max(b[1], T_Y+W_diff)), 'AB', 0, 'XY')]
        L_AC[1][0][1] = [
            Sol((max(a[1], T_X+W_same), max(c[1], T_Y+W_same)), 'AC', 0, 'XY')]
        L_BC[0][1][1] = [
            Sol((max(b[1], T_X+W_diff), max(c[1], T_Y+W_same)), 'BC', 0, 'XY')]
        L_BB[0][1][0] = [Sol((max(b[1], T_X+W_diff), T_Y), 'BB', 0, 'X0'),
                         Sol((T_X, max(b[1], T_Y+W_diff)), 'BB', 0, '0Y')]
        if beta >= 2:
            L_BB[0][2][0] = [Sol((max(b[1], T_X+W_diff), max(b[2], T_Y+W_diff)), 'BB', 0, 'XY'),
                             Sol((max(b[2], T_X+W_diff), max(b[1], T_Y+W_diff)), 'BB', 0, 'YX')]
        L_AB[1][0][0] = [Sol((max(a[1], T_X+W_same), T_Y), 'AB', 0, 'X0')]
        L_AB[0][1][0] = [Sol((T_X, max(b[1], T_Y+W_diff)), 'AB', 0, '0Y')]
        L_AC[1][0][0] = [Sol((max(a[1], T_X+W_same), T_Y), 'AC', 0, 'X0')]
        L_AC[0][0][1] = [Sol((T_X, max(c[1], T_Y+W_same)), 'AC', 0, '0Y')]
        L_BC[0][1][0] = [Sol((max(b[1], T_X+W_diff), T_Y), 'BC', 0, 'X0')]
        L_BC[0][0][1] = [Sol((T_X, max(c[1], T_Y+W_same)), 'BC', 0, '0Y')]
    elif last_XY == 'BB':
        L_AB[1][1][0] = [
            Sol((max(a[1], T_X+W_diff), max(b[1], T_Y+W_same)), 'AB', 0, 'XY')]
        L_AC[1][0][1] = [
            Sol((max(a[1], T_X+W_diff), max(c[1], T_Y+W_diff)), 'AC', 0, 'XY')]
        L_BC[0][1][1] = [
            Sol((max(b[1], T_X+W_same), max(c[1], T_Y+W_diff)), 'BC', 0, 'XY')]
        L_BB[0][1][0] = [Sol((max(b[1], T_X+W_same), T_Y), 'BB', 0, 'X0'),
                         Sol((T_X, max(b[1], T_Y+W_same)), 'BB', 0, '0Y')]
        if beta >= 2:
            L_BB[0][2][0] = [Sol((max(b[1], T_X+W_same), max(b[2], T_Y+W_same)), 'BB', 0, 'XY'),
                             Sol((max(b[2], T_X+W_same), max(b[1], T_Y+W_same)), 'BB', 0, 'YX')]
        L_AB[1][0][0] = [Sol((max(a[1], T_X+W_diff), T_Y), 'AB', 0, 'X0')]
        L_AB[0][1][0] = [Sol((T_X, max(b[1], T_Y+W_same)), 'AB', 0, '0Y')]
        L_AC[1][0][0] = [Sol((max(a[1], T_X+W_diff), T_Y), 'AC', 0, 'X0')]
        L_AC[0][0][1] = [Sol((T_X, max(c[1], T_Y+W_diff)), 'AC', 0, '0Y')]
        L_BC[0][1][0] = [Sol((max(b[1], T_X+W_same), T_Y), 'BC', 0, 'X0')]
        L_BC[0][0][1] = [Sol((T_X, max(c[1], T_Y+W_diff)), 'BC', 0, '0Y')]
    elif last_XY == 'BC':
        L_AB[1][1][0] = [
            Sol((max(a[1], T_X+W_diff), max(b[1], T_Y+W_diff)), 'AB', 0, 'XY')]
        L_AC[1][0][1] = [
            Sol((max(a[1], T_X+W_diff), max(c[1], T_Y+W_same)), 'AC', 0, 'XY')]
        L_BC[0][1][1] = [
            Sol((max(b[1], T_X+W_same), max(c[1], T_Y+W_same)), 'BC', 0, 'XY')]
        L_BB[0][1][0] = [Sol((max(b[1], T_X+W_same), T_Y), 'BB', 0, 'X0'),
                         Sol((T_X, max(b[1], T_Y+W_diff)), 'BB', 0, '0Y')]
        if beta >= 2:
            L_BB[0][2][0] = [Sol((max(b[1], T_X+W_same), max(b[2], T_Y+W_diff)), 'BB', 0, 'XY'),
                             Sol((max(b[2], T_X+W_same), max(b[1], T_Y+W_diff)), 'BB', 0, 'YX')]
        L_AB[1][0][0] = [Sol((max(a[1], T_X+W_diff), T_Y), 'AB', 0, 'X0')]
        L_AB[0][1][0] = [Sol((T_X, max(b[1], T_Y+W_diff)), 'AB', 0, '0Y')]
        L_AC[1][0][0] = [Sol((max(a[1], T_X+W_diff), T_Y), 'AC', 0, 'X0')]
        L_AC[0][0][1] = [Sol((T_X, max(c[1], T_Y+W_same)), 'AC', 0, '0Y')]
        L_BC[0][1][0] = [Sol((max(b[1], T_X+W_same), T_Y), 'BC', 0, 'X0')]
        L_BC[0][0][1] = [Sol((T_X, max(c[1], T_Y+W_same)), 'BC', 0, '0Y')]
    else:
        L_AB[1][1][0] = [Sol((a[1], b[1]), 'AB', 0, 'XY')]
        L_AC[1][0][1] = [Sol((a[1], c[1]), 'AC', 0, 'XY')]
        L_BC[0][1][1] = [Sol((b[1], c[1]), 'BC', 0, 'XY')]
        L_BB[0][1][0] = [Sol((-W_diff, b[1]), 'BB', 0, '0Y'),
                         Sol((b[1], -W_diff), 'BB', 0, 'X0')]
        if beta >= 2:
            L_BB[0][2][0] = [Sol((b[1], b[2]), 'BB', 0, 'XY'),
                             Sol((b[2], b[1]), 'BB', 0, 'YX')]
        L_AB[1][0][0] = [Sol((a[1], -W_diff), 'AB', 0, 'X0')]
        L_AB[0][1][0] = [Sol((-W_diff, b[1]), 'AB', 0, '0Y')]
        L_AC[1][0][0] = [Sol((a[1], -W_diff), 'AC', 0, 'X0')]
        L_AC[0][0][1] = [Sol((-W_diff, c[1]), 'AC', 0, '0Y')]
        L_BC[0][1][0] = [Sol((b[1], -W_diff), 'BC', 0, 'X0')]
        L_BC[0][0][1] = [Sol((-W_diff, c[1]), 'BC', 0, '0Y')]

    for i in range(2, alpha+1):
        L_AB[i][1][0] = [Sol((max(a[i], s.time[0]+W_same), s.time[1]), 'AB', idx, 'XY')
                         for idx, s in enumerate(L_AB[i-1][1][0]) if not float('inf') in s.time]
    for i in range(2, alpha+1):
        L_AC[i][0][1] = [Sol((max(a[i], s.time[0]+W_same), s.time[1]), 'AC', idx, 'XY')
                         for idx, s in enumerate(L_AC[i-1][0][1]) if not float('inf') in s.time]
    for k in range(2, gamma+1):
        L_AC[1][0][k] = [Sol((s.time[0], max(c[k], s.time[1]+W_same)), 'AC', idx, 'XY')
                         for idx, s in enumerate(L_AC[1][0][k-1]) if not float('inf') in s.time]
    for k in range(2, gamma+1):
        L_BC[0][1][k] = [Sol((s.time[0], max(c[k], s.time[1]+W_same)), 'BC', idx, 'XY')
                         for idx, s in enumerate(L_BC[0][1][k-1]) if not float('inf') in s.time]
    if beta >= 3:
        for j in range(3, beta+1):
            L_BB[0][j][0] = [Sol((max(b[j-1], s.time[0]+W_same), max(b[j], s.time[1]+W_same)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[0][j-2][0]) if not float('inf') in s.time] + \
                            [Sol((max(b[j], s.time[0]+W_same), max(b[j-1], s.time[1]+W_same)), 'BB', idx, 'YX') for idx, s in enumerate(L_BB[0][j-2][0]) if not float('inf') in s.time] + \
                            [Sol((max(b[j], max(b[j-1], s.time[0]+W_same)+W_same), s.time[1]), 'BB', idx, 'XX') for idx, s in enumerate(L_BB[0][j-2][0]) if not float('inf') in s.time] + \
                            [Sol((s.time[0], max(b[j], max(b[j-1], s.time[1]+W_same)+W_same)), 'BB', idx, 'YY')
                             for idx, s in enumerate(L_BB[0][j-2][0]) if not float('inf') in s.time]
    for i in range(2, alpha+1):
        L_AB[i][0][0] = [Sol((max(a[i], s.time[0]+W_same), s.time[1]), 'AB', idx, 'X0')
                         for idx, s in enumerate(L_AB[i-1][0][0]) if not float('inf') in s.time]
    for j in range(2, beta+1):
        L_AB[0][j][0] = [Sol((s.time[0], max(b[j], s.time[1]+W_same)), 'AB', idx, '0Y')
                         for idx, s in enumerate(L_AB[0][j-1][0]) if not float('inf') in s.time]
    for i in range(2, alpha+1):
        L_AC[i][0][0] = [Sol((max(a[i], s.time[0]+W_same), s.time[1]), 'AC', idx, 'X0')
                         for idx, s in enumerate(L_AC[i-1][0][0]) if not float('inf') in s.time]
    for k in range(2, gamma+1):
        L_AC[0][0][k] = [Sol((s.time[0], max(c[k], s.time[1]+W_same)), 'AC', idx, '0Y')
                         for idx, s in enumerate(L_AC[0][0][k-1]) if not float('inf') in s.time]
    for j in range(2, beta+1):
        L_BC[0][j][0] = [Sol((max(b[j], s.time[0]+W_same), s.time[1]), 'BC', idx, 'X0')
                         for idx, s in enumerate(L_BC[0][j-1][0]) if not float('inf') in s.time]
    for k in range(2, gamma+1):
        L_BC[0][0][k] = [Sol((s.time[0], max(c[k], s.time[1]+W_same)), 'BC', idx, '0Y')
                         for idx, s in enumerate(L_BC[0][0][k-1]) if not float('inf') in s.time]

    if beta >= 2:
        if last_XY == 'AB':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = [Sol((max(b[1], s.time[0]+W_diff), max(b[2], T_Y+W_same)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time] + \
                                [Sol((max(b[2], s.time[0]+W_diff), max(b[1], T_Y+W_same)), 'AC', idx, 'YX')
                                 for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time]
            for k in range(1, gamma+1):
                L_BB[0][2][k] = [Sol((max(b[1], T_X+W_diff), max(b[2], s.time[1]+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time] + \
                                [Sol((max(b[2], T_X+W_diff), max(b[1], s.time[1]+W_diff)), 'AC', idx, 'YX')
                                 for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time]
        elif last_XY == 'AC':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = [Sol((max(b[1], s.time[0]+W_diff), max(b[2], T_Y+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time] + \
                                [Sol((max(b[2], s.time[0]+W_diff), max(b[1], T_Y+W_diff)), 'AC', idx, 'YX')
                                 for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time]
            for k in range(1, gamma+1):
                L_BB[0][2][k] = [Sol((max(b[1], T_X+W_diff), max(b[2], s.time[1]+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time] + \
                                [Sol((max(b[2], T_X+W_diff), max(b[1], s.time[1]+W_diff)), 'AC', idx, 'YX')
                                 for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time]
        elif last_XY == 'BB':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = [Sol((max(b[1], s.time[0]+W_diff), max(b[2], T_Y+W_same)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time] + \
                                [Sol((max(b[2], s.time[0]+W_diff), max(b[1], T_Y+W_same)), 'AC', idx, 'YX')
                                 for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time]
            for k in range(1, gamma+1):
                L_BB[0][2][k] = [Sol((max(b[1], T_X+W_same), max(b[2], s.time[1]+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time] + \
                                [Sol((max(b[2], T_X+W_same), max(b[1], s.time[1]+W_diff)), 'AC', idx, 'YX')
                                 for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time]
        elif last_XY == 'BC':
            for i in range(1, alpha+1):
                L_BB[i][2][0] = [Sol((max(b[1], s.time[0]+W_diff), max(b[2], T_Y+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time] + \
                                [Sol((max(b[2], s.time[0]+W_diff), max(b[1], T_Y+W_diff)), 'AC', idx, 'YX')
                                 for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time]
            for k in range(1, gamma+1):
                L_BB[0][2][k] = [Sol((max(b[1], T_X+W_same), max(b[2], s.time[1]+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time] + \
                                [Sol((max(b[2], T_X+W_same), max(b[1], s.time[1]+W_diff)), 'AC', idx, 'YX')
                                 for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time]
        else:
            for i in range(1, alpha+1):
                L_BB[i][2][0] = [Sol((max(b[1], s.time[0]+W_diff), b[2]), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time] + \
                                [Sol((max(b[2], s.time[0]+W_diff), b[1]), 'AC', idx, 'YX')
                                 for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time]
            for k in range(1, gamma+1):
                L_BB[0][2][k] = [Sol((b[1], max(b[2], s.time[1]+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time] + \
                                [Sol((b[2], max(b[1], s.time[1]+W_diff)), 'AC', idx, 'YX')
                                 for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time]

    for i in range(1, alpha+1):
        for j in range(2, beta+1):
            L_AB[i][j][0] = [Sol((max(a[i], s.time[0]+W_same), max(b[j], s.time[1]+W_same)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i-1][j-1][0]) if not float('inf') in s.time] + \
                            [Sol((max(a[i], max(b[j], s.time[0]+W_diff)+W_diff), s.time[1]), 'AB', idx, 'XX') for idx, s in enumerate(L_AB[i-1][j-1][0]) if not float('inf') in s.time] + \
                            [Sol((max(a[i], s.time[0]+W_diff), max(b[j], s.time[1]+W_same)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i-1][j-1][0]) if not float('inf') in s.time] + \
                            [Sol((max(a[i], max(b[j], s.time[0]+W_same)+W_diff), s.time[1]), 'BB', idx, 'XX')
                             for idx, s in enumerate(L_BB[i-1][j-1][0]) if not float('inf') in s.time]
    for i in range(2, alpha+1):
        for k in range(2, gamma+1):
            L_AC[i][0][k] = [Sol((max(a[i], s.time[0]+W_same), max(c[k], s.time[1]+W_same)), 'AC', idx, 'XY')
                             for idx, s in enumerate(L_AC[i-1][0][k-1]) if not float('inf') in s.time]
    for j in range(2, beta+1):
        for k in range(1, gamma+1):
            L_BC[0][j][k] = [Sol((max(b[j], s.time[0]+W_same), max(c[k], s.time[1]+W_diff)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[0][j-1][k-1]) if not float('inf') in s.time] + \
                            [Sol((s.time[0], max(c[k], max(b[j], s.time[1]+W_same)+W_diff)), 'BB', idx, 'YY') for idx, s in enumerate(L_BB[0][j-1][k-1]) if not float('inf') in s.time] + \
                            [Sol((max(b[j], s.time[0]+W_same), max(c[k], s.time[1]+W_same)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[0][j-1][k-1]) if not float('inf') in s.time] + \
                            [Sol((s.time[0], max(c[k], max(b[j], s.time[1]+W_diff)+W_diff)), 'BC', idx, 'YY')
                             for idx, s in enumerate(L_BC[0][j-1][k-1]) if not float('inf') in s.time]
    for i in range(1, alpha+1):
        for j in range(3, beta+1):
            L_BB[i][j][0] = [Sol((max(b[j-1], s.time[0]+W_diff), max(b[j], s.time[1]+W_same)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i][j-2][0]) if not float('inf') in s.time] + \
                            [Sol((max(b[j], s.time[0]+W_diff), max(b[j-1], s.time[1]+W_same)), 'AB', idx, 'YX') for idx, s in enumerate(L_AB[i][j-2][0]) if not float('inf') in s.time] + \
                            [Sol((max(b[j-1], s.time[0]+W_same), max(b[j], s.time[1]+W_same)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i][j-2][0]) if not float('inf') in s.time] + \
                            [Sol((max(b[j], s.time[0]+W_same), max(b[j-1], s.time[1]+W_same)), 'BB', idx, 'YX') for idx, s in enumerate(L_BB[i][j-2][0]) if not float('inf') in s.time] + \
                            [Sol((max(b[j], max(b[j-1], s.time[0]+W_same)+W_same), s.time[1]), 'BB', idx, 'XX') for idx, s in enumerate(L_BB[i][j-2][0]) if not float('inf') in s.time] + \
                            [Sol((s.time[0], max(b[j], max(b[j-1], s.time[1]+W_same)+W_same)), 'BB', idx, 'YY')
                             for idx, s in enumerate(L_BB[i][j-2][0]) if not float('inf') in s.time]

    for i in range(1, alpha+1):
        for j in range(1, beta+1):
            for k in range(1, gamma+1):
                L_AB[i][j][k] = [Sol((max(a[i], s.time[0]+W_same), max(b[j], s.time[1]+W_same)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i-1][j-1][k]) if not float('inf') in s.time] + \
                                [Sol((max(a[i], max(b[j], s.time[0]+W_diff)+W_diff), s.time[1]), 'AB', idx, 'XX') for idx, s in enumerate(L_AB[i-1][j-1][k]) if not float('inf') in s.time] + \
                                [Sol((max(a[i], s.time[0]+W_same), max(b[j], s.time[1]+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i-1][j-1][k]) if not float('inf') in s.time] + \
                                [Sol((max(a[i], s.time[0]+W_diff), max(b[j], s.time[1]+W_same)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i-1][j-1][k]) if not float('inf') in s.time] + \
                                [Sol((max(a[i], max(b[j], s.time[0]+W_same)+W_diff), s.time[1]), 'BB', idx, 'XX') for idx, s in enumerate(L_BB[i-1][j-1][k]) if not float('inf') in s.time] + \
                                [Sol((max(a[i], s.time[0]+W_diff), max(b[j], s.time[1]+W_diff)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[i-1][j-1][k]) if not float('inf') in s.time]

                L_AC[i][j][k] = [Sol((max(a[i], s.time[0]+W_same), max(c[k], s.time[1]+W_diff)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i-1][j][k-1]) if not float('inf') in s.time] + \
                                [Sol((max(a[i], s.time[0]+W_same), max(c[k], s.time[1]+W_same)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i-1][j][k-1]) if not float('inf') in s.time] + \
                                [Sol((max(a[i], s.time[0]+W_diff), max(c[k], s.time[1]+W_diff)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i-1][j][k-1]) if not float('inf') in s.time] + \
                                [Sol((max(a[i], s.time[0]+W_diff), max(c[k], s.time[1]+W_same)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[i-1][j][k-1]) if not float('inf') in s.time]

                L_BC[i][j][k] = [Sol((max(b[j], s.time[0]+W_diff), max(c[k], s.time[1]+W_diff)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i][j-1][k-1]) if not float('inf') in s.time] + \
                                [Sol((max(b[j], s.time[0]+W_diff), max(c[k], s.time[1]+W_same)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i][j-1][k-1]) if not float('inf') in s.time] + \
                                [Sol((max(b[j], s.time[0]+W_same), max(c[k], s.time[1]+W_diff)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i][j-1][k-1]) if not float('inf') in s.time] + \
                                [Sol((s.time[0], max(c[k], max(b[j], s.time[1]+W_same)+W_diff)), 'BB', idx, 'YY') for idx, s in enumerate(L_BB[i][j-1][k-1]) if not float('inf') in s.time] + \
                                [Sol((max(b[j], s.time[0]+W_same), max(c[k], s.time[1]+W_same)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[i][j-1][k-1]) if not float('inf') in s.time] + \
                                [Sol((s.time[0], max(c[k], max(b[j], s.time[1]+W_diff)+W_diff)), 'BC', idx, 'YY') for idx, s in enumerate(L_BC[i][j-1][k-1]) if not float('inf') in s.time]

                if j >= 2:
                    L_BB[i][j][k] = [Sol((max(b[j-1], s.time[0]+W_diff), max(b[j], s.time[1]+W_same)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((max(b[j], s.time[0]+W_diff), max(b[j-1], s.time[1]+W_same)), 'AB', idx, 'YX') for idx, s in enumerate(L_AB[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((max(b[j], max(b[j-1], s.time[0]+W_diff)+W_same), s.time[1]), 'AB', idx, 'XX') for idx, s in enumerate(L_AB[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((max(b[j-1], s.time[0]+W_diff), max(b[j], s.time[1]+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((max(b[j], s.time[0]+W_diff), max(b[j-1], s.time[1]+W_diff)), 'AC', idx, 'YX') for idx, s in enumerate(L_AC[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((max(b[j-1], s.time[0]+W_same), max(b[j], s.time[1]+W_same)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((max(b[j], s.time[0]+W_same), max(b[j-1], s.time[1]+W_same)), 'BB', idx, 'YX') for idx, s in enumerate(L_BB[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((max(b[j], max(b[j-1], s.time[0]+W_same)+W_same), s.time[1]), 'BB', idx, 'XX') for idx, s in enumerate(L_BB[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((s.time[0], max(b[j], max(b[j-1], s.time[1]+W_same)+W_same)), 'BB', idx, 'YY') for idx, s in enumerate(L_BB[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((max(b[j-1], s.time[0]+W_same), max(b[j], s.time[1]+W_diff)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((max(b[j], s.time[0]+W_same), max(b[j-1], s.time[1]+W_diff)), 'BC', idx, 'YX') for idx, s in enumerate(L_BC[i][j-2][k]) if not float('inf') in s.time] + \
                                    [Sol((s.time[0], max(b[j], max(b[j-1], s.time[1]+W_same)+W_diff)), 'BC', idx, 'YY') for idx, s in enumerate(L_BC[i][j-2][k]) if not float('inf') in s.time]

    # print_solNum(L_AB, 'L_AB')
    # print_solNum(L_AC, 'L_AC')
    # print_solNum(L_BB, 'L_BB')
    # print_solNum(L_BC, 'L_BC')

    # Push order to stack
    stack_X = []
    stack_Y = []
    i = alpha
    j = beta
    k = gamma
    for s in L_AB[i][j][k]:
        if float('inf') in s.time:
            print('err')
    for s in L_AC[i][j][k]:
        if float('inf') in s.time:
            print('err')
    for s in L_BB[i][j][k]:
        if float('inf') in s.time:
            print('err')
    for s in L_BC[i][j][k]:
        if float('inf') in s.time:
            print('err')
    # Choose the optimal solution and the table to start backtracking
    opt = min(L_AB[i][j][k], key=get_obj)
    min_max = max(opt.time)
    table = 'AB'
    idx = L_AB[i][j][k].index(opt)
    if max(min(L_AC[i][j][k], key=get_obj).time) < min_max:
        opt = min(L_AC[i][j][k], key=get_obj)
        min_max = max(opt.time)
        table = 'AC'
        idx = L_AC[i][j][k].index(opt)
    if max(min(L_BB[i][j][k], key=get_obj).time) < min_max:
        opt = min(L_BB[i][j][k], key=get_obj)
        min_max = max(opt.time)
        table = 'BB'
        idx = L_BB[i][j][k].index(opt)
    if max(min(L_BC[i][j][k], key=get_obj).time) < min_max:
        opt = min(L_BC[i][j][k], key=get_obj)
        min_max = max(opt.time)
        table = 'BC'
        idx = L_BC[i][j][k].index(opt)

    # Backtracking
    while i > 0 or j > 0 or k > 0:
        # print(table, i, j, k, idx)
        if table == 'AB':
            # print('AB')
            lanes = L_AB[i][j][k][idx].lane
            table = L_AB[i][j][k][idx].table
            if lanes == 'XY':
                stack_X.append(('A', i, L_AB[i][j][k][idx].time[0]))
                stack_Y.append(('B', j, L_AB[i][j][k][idx].time[1]))
                idx = L_AB[i][j][k][idx].idx
                i -= 1
                j -= 1
            elif lanes == 'XX':
                stack_X.append(('A', i, L_AB[i][j][k][idx].time[0]))
                stack_X.append(('B', j, 0))
                # if table == 'AB':
                #     stack_X.append(('B', j, max(b[j], L_AB[i-1][j-1][k][idx].time[0]+W_diff)))
                # elif table == 'BB':
                #     stack_X.append(('B', j, max(b[j], L_BB[i-1][j-1][k][idx].time[0]+W_same)))
                # else:
                #     print('bug')
                idx = L_AB[i][j][k][idx].idx
                i -= 1
                j -= 1
            elif lanes[0] == 'X':
                stack_X.append(('A', i, L_AB[i][j][k][idx].time[0]))
                idx = L_AB[i][j][k][idx].idx
                i -= 1
            elif lanes[1] == 'Y':
                stack_Y.append(('B', j, L_AB[i][j][k][idx].time[1]))
                idx = L_AB[i][j][k][idx].idx
                j -= 1
        elif table == 'AC':
            # print('AC')
            lanes = L_AC[i][j][k][idx].lane
            table = L_AC[i][j][k][idx].table
            if lanes == 'XY':
                stack_X.append(('A', i, L_AC[i][j][k][idx].time[0]))
                stack_Y.append(('C', k, L_AC[i][j][k][idx].time[1]))
                idx = L_AC[i][j][k][idx].idx
                i -= 1
                k -= 1
            elif lanes[0] == 'X':
                stack_X.append(('A', i, L_AC[i][j][k][idx].time[0]))
                idx = L_AC[i][j][k][idx].idx
                i -= 1
            elif lanes[1] == 'Y':
                stack_Y.append(('C', k, L_AC[i][j][k][idx].time[1]))
                idx = L_AC[i][j][k][idx].idx
                k -= 1
        elif table == 'BB':
            # print('BB')
            lanes = L_BB[i][j][k][idx].lane
            table = L_BB[i][j][k][idx].table
            if lanes == 'XY':
                stack_X.append(('B', j-1, L_BB[i][j][k][idx].time[0]))
                stack_Y.append(('B', j, L_BB[i][j][k][idx].time[1]))
                j -= 2
            elif lanes == 'YX':
                stack_Y.append(('B', j-1, L_BB[i][j][k][idx].time[1]))
                stack_X.append(('B', j, L_BB[i][j][k][idx].time[0]))
                j -= 2
            elif lanes == 'XX':
                stack_X.append(('B', j, L_BB[i][j][k][idx].time[0]))
                # stack_X.append(('B', j-1, max(b[j-1], L_BB[i][j-2][k][idx].time[0]+W_same)))
                stack_X.append(('B', j-1, 0))
                j -= 2
            elif lanes == 'YY':
                stack_Y.append(('B', j, L_BB[i][j][k][idx].time[1]))
                stack_Y.append(('B', j-1, 0))
                # if table == 'BB':
                #     stack_Y.append(('B', j-1, max(b[j-1], L_BB[i][j-2][k][idx].time[1]+W_same)))
                # elif table == 'BC':
                #     stack_Y.append(('B', j-1, max(b[j-1], L_BC[i][j-2][k][idx].time[1]+W_same)))
                # else:
                #     print('bug')
                j -= 2
            elif lanes[0] == 'X':
                stack_X.append(('B', j, L_BB[i][j][k][idx].time[0]))
                j -= 1
            elif lanes[1] == 'Y':
                stack_Y.append(('B', j, L_BB[i][j][k][idx].time[1]))
                j -= 1
            idx = L_BB[i][j][k][idx].idx
        elif table == 'BC':
            # print('BC')
            lanes = L_BC[i][j][k][idx].lane
            table = L_BC[i][j][k][idx].table
            if lanes == 'XY':
                stack_X.append(('B', j, L_BC[i][j][k][idx].time[0]))
                stack_Y.append(('C', k, L_BC[i][j][k][idx].time[1]))
                idx = L_BC[i][j][k][idx].idx
                j -= 1
                k -= 1
            elif lanes == 'YY':
                stack_Y.append(('C', k, L_BC[i][j][k][idx].time[1]))
                stack_Y.append(('B', j, 0))
                # if table == 'BB':
                #     stack_Y.append(('B', j, max(b[j], L_BB[i][j-1][k-1][idx].time[1]+W_same)))
                # elif table == 'BC':
                #     stack_Y.append(('B', j, max(b[j], L_BC[i][j-1][k-1][idx].time[1]+W_diff)))
                # else:
                #     print('bug')
                j -= 1
                k -= 1
            elif lanes[0] == 'X':
                stack_X.append(('B', j, L_BC[i][j][k][idx].time[0]))
                idx = L_BC[i][j][k][idx].idx
                j -= 1
            elif lanes[1] == 'Y':
                stack_Y.append(('C', k, L_BC[i][j][k][idx].time[1]))
                idx = L_BC[i][j][k][idx].idx
                k -= 1
    # Time to finish scheduling
    computeTime = time.time() - t0

    # The time needed for all vehicles to pass the merging point
    T_last = min_max
    # T_last = stack_X[0][2] if stack_X[0][2] >= stack_Y[0][2] else stack_Y[0][2]

    # Delete the redundant element (i==0 or j==0 or k==0)
    while stack_X[-1][1] <= 0:
        stack_X.pop()
    while stack_Y[-1][1] <= 0:
        stack_Y.pop()

    # Output order
    print('lane X:')
    last_X = stack_X[0]
    # while len(stack_X) > 0:
    #     print(stack_X.pop())
    # print('-----------------------')
    last_Y = stack_Y[0]
    # while len(stack_Y) > 0:
    #     print(stack_Y.pop())

    computeTime = time.time()-t0
    return last_X, last_Y, computeTime


def schedule_single_lane(lane, traffic, W_same, W_diff, prev):
    prevLane = prev[0]
    prevTime = prev[2]
    last = ('', 0, 0)
    if prevLane == '':
        last = (lane, 1, traffic[1])
    elif lane == prevLane:
        last = (lane, 1, max(traffic[1], prevTime+W_same))
    else:
        last = (lane, 1, max(traffic[1], prevTime+W_diff))
    for i in range(2, len(traffic)):
        prevTime = last[2]
        last = (lane, i, max(traffic[i], prevTime+W_same))
    return last


def schedule_by_time_window(a_all, b_all, c_all, W_same, W_diff, cutTime, keep, allSol):
    last_X = ('', 0, 0.0)
    last_Y = ('', 0, 0.0)
    if allSol:
        t0 = time.time()
        while (len(a_all) > 1 and len(b_all) > 1) or (len(b_all) > 1 and (len(c_all) > 1)) or (len(b_all) > 2):
            a, b, c, a_all, b_all, c_all = get_window_by_time(
                a_all, b_all, c_all, cutTime, keep)
            last_X, last_Y, _ = window_allSol_dp(
                a, b, c, W_same, W_diff, last_X, last_Y, keep)
        computeTime = time.time() - t0
        T_last = last_X[2] if last_X[2] >= last_Y[2] else last_Y[2]
        return T_last, computeTime
    else:
        t0 = time.time()
        while (len(a_all) > 1 and len(b_all) > 1) or (len(b_all) > 1 and (len(c_all) > 1)) or (len(b_all) > 2):
            a, b, c, a_all, b_all, c_all = get_window_by_time(
                a_all, b_all, c_all, cutTime, keep)
            last_X, last_Y, _ = window_oneSol_dp(
                a, b, c, W_same, W_diff, last_X, last_Y, keep)
        computeTime = time.time() - t0
        T_last = last_X[2] if last_X[2] >= last_Y[2] else last_Y[2]
        return T_last, computeTime


def schedule_by_num_window(a_all, b_all, c_all, W_same, W_diff, carNum, keep, allSol):
    last_X = ('', 0, 0.0)
    last_Y = ('', 0, 0.0)
    if allSol:
        t0 = time.time()
        # while (len(a_all) > 1 and len(b_all) > 1) or (len(b_all) > 1 and (len(c_all) > 1)) or (len(b_all) > 2):
        while len(a_all) > 1 or len(b_all) > 1 or len(c_all) > 1:
            # a, b, c, a_all, b_all, c_all = get_window_by_num(
            #     a_all, b_all, c_all, carNum, keep)
            a, a_all = get_window_by_num(a_all, carNum, keep)
            b, b_all = get_window_by_num(b_all, carNum, keep)
            c, c_all = get_window_by_num(c_all, carNum, keep)
            if len(a) > 1 and len(b) > 1 and len(c) > 1:
                last_X, last_Y, _ = window_allSol_dp(a, b, c, W_same, W_diff, last_X, last_Y, keep)
            elif len(a) > 1 and len(b) > 1:
                last_X = schedule_single_lane('A', a, W_same, W_diff, last_X)
                last_Y = schedule_single_lane('B', b, W_same, W_diff, last_Y)
            elif len(a) > 1 and len(c) > 1:
                last_X = schedule_single_lane('A', a, W_same, W_diff, last_X)
                last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y)
            elif len(b) > 1 and len(c) > 1:
                last_X = schedule_single_lane('B', b, W_same, W_diff, last_X)
                last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y)
            elif len(a) > 1:
                last_X = schedule_single_lane('A', a, W_same, W_diff, last_X)
            elif len(c) > 1:
                last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y)
            elif len(b) > 1:
                if last_X[2] < last_Y[2]:
                    if last_X[0] == '':
                        last_X = ('B', 1, b[1])
                    elif last_X[0] == 'A':
                        last_X = ('B', 1, max(b[1], last_X[2]+W_diff))
                    else:
                        last_X = ('B', 1, max(b[1], last_X[2]+W_same))
                    if len(b) > 2:
                        if last_Y[0] == '':
                            last_Y = ('B', 2, b[2])
                        elif last_Y[0] == 'C':
                            last_Y = ('B', 2, max(b[2], last_Y[2]+W_diff))
                        else:
                            last_Y = ('B', 2, max(b[2], last_Y[2]+W_same))
                        for i in range(3, len(b), 2):
                            last_X = ('B', i, max(b[i], last_X[2]+W_same))
                            last_Y = ('B', i+1, max(b[i+1], last_Y[2]+W_same))
                else:
                    if last_Y[0] == '':
                        last_Y = ('B', 1, b[1])
                    elif last_Y[0] == 'C':
                        last_Y = ('B', 1, max(b[1], last_Y[2]+W_diff))
                    else:
                        last_Y = ('B', 1, max(b[1], last_Y[2]+W_same))
                    if len(b) > 2:
                        if last_X[0] == '':
                            last_X = ('B', 2, b[2])
                        elif last_X[0] == 'A':
                            last_X = ('B', 2, max(b[2], last_X[2]+W_diff))
                        else:
                            last_X = ('B', 2, max(b[2], last_X[2]+W_same))
                        for i in range(3, len(b), 2):
                            last_Y = ('B', i, max(b[i], last_Y[2]+W_same))
                            last_X = ('B', i+1, max(b[i+1], last_X[2]+W_same))
        
        computeTime = time.time() - t0
        T_last = last_X[2] if last_X[2] >= last_Y[2] else last_Y[2]
        return T_last, computeTime
    else:
        t0 = time.time()
        # while (len(a_all) > 1 and len(b_all) > 1) or (len(b_all) > 1 and (len(c_all) > 1)) or (len(b_all) > 2):
        while len(a_all) > 1 or len(b_all) > 1 or len(c_all) > 1:
            # a, b, c, a_all, b_all, c_all = get_window_by_num(
            #     a_all, b_all, c_all, carNum, keep)
            a, a_all = get_window_by_num(a_all, carNum, keep)
            b, b_all = get_window_by_num(b_all, carNum, keep)
            c, c_all = get_window_by_num(c_all, carNum, keep)
            if len(a) > 1 and len(b) > 1 and len(c) > 1:
                last_X, last_Y, _ = window_oneSol_dp(a, b, c, W_same, W_diff, last_X, last_Y, keep)
            elif len(a) > 1 and len(b) > 1:
                last_X = schedule_single_lane('A', a, W_same, W_diff, last_X)
                last_Y = schedule_single_lane('B', b, W_same, W_diff, last_Y)
            elif len(a) > 1 and len(c) > 1:
                last_X = schedule_single_lane('A', a, W_same, W_diff, last_X)
                last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y)
            elif len(b) > 1 and len(c) > 1:
                last_X = schedule_single_lane('B', b, W_same, W_diff, last_X)
                last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y)
            elif len(a) > 1:
                last_X = schedule_single_lane('A', a, W_same, W_diff, last_X)
            elif len(c) > 1:
                last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y)
            elif len(b) > 1:
                if last_X[2] < last_Y[2]:
                    if last_X[0] == '':
                        last_X = ('B', 1, b[1])
                    elif last_X[0] == 'A':
                        last_X = ('B', 1, max(b[1], last_X[2]+W_diff))
                    else:
                        last_X = ('B', 1, max(b[1], last_X[2]+W_same))
                    if last_Y[0] == '':
                        last_Y = ('B', 2, b[2])
                    elif last_Y[0] == 'C':
                        last_Y = ('B', 2, max(b[2], last_Y[2]+W_diff))
                    else:
                        last_Y = ('B', 2, max(b[2], last_Y[2]+W_same))
                    for i in range(3, len(b), 2):
                        last_X = ('B', i, max(b[i], last_X[2]+W_same))
                        last_Y = ('B', i+1, max(b[i+1], last_Y[2]+W_same))
                else:
                    if last_Y[0] == '':
                        last_Y = ('B', 1, b[1])
                    elif last_Y[0] == 'C':
                        last_Y = ('B', 1, max(b[1], last_Y[2]+W_diff))
                    else:
                        last_Y = ('B', 1, max(b[1], last_Y[2]+W_same))
                    if last_X[0] == '':
                        last_X = ('B', 2, b[2])
                    elif last_X[0] == 'A':
                        last_X = ('B', 2, max(b[2], last_X[2]+W_diff))
                    else:
                        last_X = ('B', 2, max(b[2], last_X[2]+W_same))
                    for i in range(3, len(b), 2):
                        last_Y = ('B', i, max(b[i], last_Y[2]+W_same))
                        last_X = ('B', i+1, max(b[i+1], last_X[2]+W_same))
        
            # print(f'last_X: {last_X}')
            # print(f'last_Y: {last_Y}')
        computeTime = time.time() - t0
        T_last = last_X[2] if last_X[2] >= last_Y[2] else last_Y[2]
        return T_last, computeTime


def main():
    timeStep = 1
    try:
        # the waiting time if two consecutive vehicles are from the same lane
        W_same = float(sys.argv[3])
        # the waiting time if two consecutive vehicles are from different lanes
        W_diff = float(sys.argv[4])
        alpha = int(sys.argv[2])
        beta = int(sys.argv[2])
        gamma = int(sys.argv[2])
        p = float(sys.argv[1])      # lambda for Poisson distribution
        pA = p / 3    # lambda for Poisson distribution in lane A
        pB = p / 3    # lambda for Poisson distribution in lane B
        pC = p / 3    # lambda for Poisson distribution in lane C
    except:
        print('Arguments: lambda, N, W=, W+')
        return

    a_all, b_all, c_all = generate_traffic_v1(
        timeStep, alpha, beta, gamma, p, p, p)
        
    a_all, b_all, c_all = generate_traffic_v2(timeStep, alpha, beta, gamma, p)
    a_all = [0, 73.58, 74.58]
    b_all = [0, 68.28, 69.28, 70.58]
    c_all = [0, 72.28, 73.28, 74.28, 75.28]
    # print(a_all)
    # print(b_all)
    # print(c_all)
    last_X = ('', 0, 0.0)
    last_Y = ('', 0, 0.0)
    # last_X, last_Y, computeTime = window_oneSol_dp(copy.deepcopy(a_all), copy.deepcopy(b_all), copy.deepcopy(c_all), W_same, W_diff, last_X, last_Y, 0)
    # T_last = last_X[2] if last_X[2] >= last_Y[2] else last_Y[2]
    # print((T_last, computeTime))
    print(schedule_by_num_window(copy.deepcopy(a_all), copy.deepcopy(b_all), copy.deepcopy(c_all), W_same, W_diff, 5, 0, False))
    # print(schedule_by_num_window(copy.deepcopy(a_all), copy.deepcopy(b_all), copy.deepcopy(c_all), W_same, W_diff, 10, 0, False))
    # print(schedule_by_num_window(copy.deepcopy(a_all), copy.deepcopy(b_all), copy.deepcopy(c_all), W_same, W_diff, 20, 0, False))
    print('-------------------------------------------All Solutions---------------------------------------------')
    # print(schedule_by_num_window(copy.deepcopy(a_all), copy.deepcopy(b_all), copy.deepcopy(c_all), W_same, W_diff, 5, 0, True))
    # print(schedule_by_num_window(copy.deepcopy(a_all), copy.deepcopy(b_all), copy.deepcopy(c_all), W_same, W_diff, 10, 0, True))
    # print(schedule_by_num_window(copy.deepcopy(a_all), copy.deepcopy(b_all), copy.deepcopy(c_all), W_same, W_diff, 20, 0, True))


if __name__ == '__main__':
    main()
