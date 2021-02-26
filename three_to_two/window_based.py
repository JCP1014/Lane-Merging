import numpy as np
import sys
import time
import random
from collections import namedtuple
Sol = namedtuple("Sol", "time table lane")
Lane = namedtuple("Lane", "traffic num num_tmp")

a_all = [0]
b_all = [0]
c_all = [0]

# v2 
def generate_traffic(timeStep, alpha, beta, gamma, p):    
    # Randomly generate earliest arrival time
    # p = 1. / 10 # a vehicle is generated every 10 seconds in average.
    alpha_tmp = alpha
    beta_tmp = beta
    gamma_tmp = gamma
    num = alpha_tmp + beta_tmp + gamma_tmp
    t = 1.0
    cars = []
    while num> 0:
        if np.random.uniform(0, 1) < p:
            cars.append(round(t, 1))
            num -= 1
        t += timeStep

    # print(a_all)
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
    print(a_all)
    print(b_all)
    print(c_all)

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
    print('a_cut', a_cut)
    print('b_cut', b_cut)
    print('c_cut', c_cut)

def get_obj(sol):
    return max(sol.time)

def multiDim_dp(W_same, W_diff):
    # Set the current window
    windowSize = 3
    a  = a_all[:(windowSize+1)]
    b = b_all[:(windowSize+1)]
    c  = c_all[:(windowSize+1)]
    del a_all[1:(windowSize+1)]
    del b_all[1:(windowSize+1)]
    del c_all[1:(windowSize+1)]

    alpha = windowSize
    beta = windowSize
    gamma = windowSize
    L_AB = [[[ Sol((float('inf'), float('inf')), '', '') for k in range(gamma+1) ] for j in range(beta+1)] for i in range(alpha+1)]
    L_AC = [[[ Sol((float('inf'), float('inf')), '', '') for k in range(gamma+1) ] for j in range(beta+1)] for i in range(alpha+1)]
    L_BB = [[[ Sol((float('inf'), float('inf')), '', '') for k in range(gamma+1) ] for j in range(beta+1)] for i in range(alpha+1)]
    L_BC = [[[ Sol((float('inf'), float('inf')), '', '') for k in range(gamma+1) ] for j in range(beta+1)] for i in range(alpha+1)]

    t0 = time.time()
    # Initialize
    L_AB[0][0][0] = Sol((0.0, 0.0), '', '')
    L_AC[0][0][0] = Sol((0.0, 0.0), '', '')
    L_BB[0][0][0] = Sol((0.0, 0.0), '', '')
    L_BC[0][0][0] = Sol((0.0, 0.0), '', '')

    L_AB[1][1][0] = Sol((a[1], b[1]), 'AB', 'XY')
    L_AC[1][0][1] = Sol((a[1], c[1]), 'AC', 'XY')
    L_BC[0][1][1] = Sol((b[1], c[1]), 'BC', 'XY')
    L_BB[0][1][0] = Sol((b[1], b[1]), 'BB', 'XY')
    if beta >= 2:
        L_BB[0][2][0] = Sol((b[1], b[2]), 'BB', 'XY') if a[1] <= c[1] else Sol((b[2], b[1]), 'BB', 'YX')
    if beta >= 3:
        L_BB[0][3][0] = min(
                            Sol((b[1], max(b[3], b[2]+W_same)), 'BB', 'YY'),
                            Sol((max(b[3], b[1]+W_same), b[2]), 'BB', 'YX'),
                            Sol((max(b[2], b[1]+W_same), b[3]), 'BB', 'XY'),
                            key=get_obj
        )
        # if L_BB[0][3][0][0] > L_BB[0][3][0][1]:
        #     if a[1] < c[1]:
        #         L_BB[0][3][0] = L_BB[0][3][0][::-1]
        # elif L_BB[0][3][0][0] < L_BB[0][3][0][1]:
        #     if c[1] < a[1]:
        #         L_BB[0][3][0] = L_BB[0][3][0][::-1]
            
    for i in range(2, alpha+1):
        L_AB[i][1][0] = Sol((max(a[i], L_AB[i-1][1][0].time[0]+W_same), b[1]), 'AB', 'XY')
    for i in range(2, alpha+1):
        L_AC[i][0][1] = Sol((max(a[i], L_AC[i-1][0][1].time[0]+W_same), c[1]), 'AC', 'XY')
    for k in range(2, gamma+1):
        L_AC[1][0][k] = Sol((a[1], max(c[k], L_AC[1][0][k-1].time[1]+W_same)), 'AC', 'XY')
    for k in range(2, gamma+1):
        L_BC[0][1][k] = Sol((b[1], max(c[k], L_BC[0][1][k-1].time[1]+W_same)), 'BC', 'XY')
    if beta >= 4:
        for j in range(4, beta+1):
            L_BB[0][j][0] = min(
                                Sol((max(b[j-1], L_BB[i][j-2][0].time[0]+W_same), max(b[j], L_BB[i][j-2][0].time[1]+W_same)), 'BB', 'XY'),
                                Sol((max(b[j], L_BB[i][j-2][0].time[0]+W_same), max(b[j-1], L_BB[i][j-2][0].time[1]+W_same)), 'BB', 'YX'),
                                Sol((max(b[j], max(b[j-1], L_BB[i][j-2][0].time[0]+W_same)+W_same), L_BB[i][j-2][0].time[1]), 'BB', 'XX'),
                                Sol((L_BB[i][j-2][0].time[0], max(b[j], max(b[j-1], L_BB[i][j-2][0].time[1]+W_same)+W_same)), 'BB', 'YY'),
                                key=get_obj
            )
            # if L_BB[0][j][0][0] > L_BB[0][j][0][1]:
            #     if a[1] < c[1]:
            #         L_BB[0][j][0] = L_BB[0][j][0][::-1]
            # elif L_BB[0][j][0][0] < L_BB[0][j][0][1]:
            #     if c[1] < a[1]:
            #         L_BB[0][j][0] = L_BB[0][j][0][::-1]


    L_AB[1][0][0] = Sol((a[1], -W_diff), 'AB', 'X0')
    for i in range(2, alpha+1):
        L_AB[i][0][0] = Sol((max(a[i], L_AB[i-1][0][0].time[0]+W_same), -W_diff), 'AB', 'X0')
    L_AC[1][0][0] = Sol((a[1], -W_diff), 'AC', 'X0')
    for i in range(2, alpha+1):
        L_AC[i][0][0] = Sol((max(a[i], L_AC[i-1][0][0].time[0]+W_same), -W_diff), 'AC', 'X0')
    L_AC[0][0][1] = Sol((-W_diff, c[1]), 'AC', '0Y')
    for k in range(2, gamma+1):
        L_AC[0][0][k] = Sol((-W_diff, max(c[k], L_AC[0][0][k-1].time[1]+W_same)), 'AC', '0Y')
    L_BC[0][0][1] = Sol((-W_diff, c[1]), 'BC', '0Y')
    for k in range(2, gamma+1):
        L_BC[0][0][k] = Sol((-W_diff, max(c[k], L_BC[0][0][k-1].time[1]+W_same)), 'BC', '0Y')
    if beta >= 2:
        for i in range(1, alpha+1):
            L_BB[i][2][0] = min(
                                Sol((max(b[1], L_AC[i][0][0].time[0]+W_diff), b[2]), 'AC', 'XY'),
                                Sol((max(b[2], L_AC[i][0][0].time[0]+W_diff), b[1]), 'AC', 'YX'),
                                key=get_obj
            )
        for k in range(1, gamma+1):
            L_BB[0][2][k] = min(
                                Sol((b[1], max(b[2], L_AC[0][0][k].time[1]+W_diff)), 'AC', 'XY'),
                                Sol((b[2], max(b[1], L_AC[0][0][k].time[1]+W_diff)), 'AC', 'YX'),
                                key=get_obj
            )

    
    for i in range(1, alpha+1):
        for j in range(2, beta+1):
            L_AB[i][j][0] = min(
                                Sol((max(a[i], L_AB[i-1][j-1][0].time[0]+W_same), max(b[j], L_AB[i-1][j-1][0].time[1]+W_same)), 'AB', 'XY'),
                                Sol((max(a[i], max(b[j], L_AB[i-1][j-1][0].time[0]+W_diff)+W_diff), L_AB[i-1][j-1][0].time[1]), 'AB', 'XX'),
                                Sol((max(a[i], L_BB[i-1][j-1][0].time[0]+W_diff), max(b[j], L_BB[i-1][j-1][0].time[1]+W_same)), 'BB', 'XY'),
                                Sol((max(a[i], max(b[j], L_BB[i-1][j-1][0].time[0]+W_same)+W_diff), L_BB[i-1][j-1][0].time[1]), 'BB', 'XX'),
                                key=get_obj
            )
    for i in range(2, alpha+1):
        for k in range(2, gamma+1):
            L_AC[i][0][k] = Sol((max(a[i], L_AC[i-1][0][k-1].time[0]+W_same), max(c[k], L_AC[i-1][0][k-1].time[1]+W_same)), 'AC', 'XY')
    for j in range(2, beta+1):
        for k in range(1, gamma+1):
            L_BC[0][j][k] = min(
                                Sol((max(b[j], L_BB[0][j-1][k-1].time[0]+W_same), max(c[k], L_BB[0][j-1][k-1].time[1]+W_diff)), 'BB', 'XY'),
                                Sol((L_BB[0][j-1][k-1].time[0], max(c[k], max(b[j], L_BB[0][j-1][k-1].time[1]+W_same)+W_diff)), 'BB', 'YY'),
                                Sol((max(b[j], L_BC[0][j-1][k-1].time[0]+W_same), max(c[k], L_BC[0][j-1][k-1].time[1]+W_same)), 'BC', 'XY'),
                                Sol((L_BC[0][j-1][k-1].time[0], max(c[k], max(b[j], L_BC[0][j-1][k-1].time[1]+W_diff)+W_diff)), 'BC', 'YY'),
                                key=get_obj
            )
    for i in range(1, alpha+1):
        for j in range(3, beta+1):
            L_BB[i][j][0] = min(
                                Sol((max(b[j-1], L_AB[i][j-2][0].time[0]+W_diff), max(b[j], L_AB[i][j-2][0].time[1]+W_same)), 'AB', 'XY'),
                                Sol((max(b[j], L_AB[i][j-2][0].time[0]+W_diff), max(b[j-1], L_AB[i][j-2][0].time[1]+W_same)), 'AB', 'YX'),
                                Sol((max(b[j-1], L_BB[i][j-2][0].time[0]+W_same), max(b[j], L_BB[i][j-2][0].time[1]+W_same)), 'BB', 'XY'),
                                Sol((max(b[j], L_BB[i][j-2][0].time[0]+W_same), max(b[j-1], L_BB[i][j-2][0].time[1]+W_same)), 'BB', 'YX'),
                                Sol((max(b[j], max(b[j-1], L_BB[i][j-2][0].time[0]+W_same)+W_same), L_BB[i][j-2][0].time[1]), 'BB', 'XX'), 
                                Sol((L_BB[i][j-2][0].time[0], max(b[j], max(b[j-1], L_BB[i][j-2][0].time[1]+W_same)+W_same)), 'BB', 'YY'),
                                key=get_obj
            )

    L_AB[0][1][0] = Sol((-W_diff, b[1]), 'AB', '0Y')
    L_BC[0][1][0] = Sol((b[1], -W_diff), 'BC', 'X0')

    
    for i in range(1, alpha+1):
        for j in range(1, beta+1):
            for k in range(1, gamma+1):
                L_AB[i][j][k] = min(
                                    Sol((max(a[i], L_AB[i-1][j-1][k].time[0]+W_same), max(b[j], L_AB[i-1][j-1][k].time[1]+W_same)), 'AB', 'XY'),
                                    Sol((max(a[i], max(b[j], L_AB[i-1][j-1][k].time[0]+W_diff)+W_diff), L_AB[i-1][j-1][k].time[1]), 'AB', 'XX'),
                                    Sol((max(a[i], L_AC[i-1][j-1][k].time[0]+W_same), max(b[j], L_AC[i-1][j-1][k].time[1]+W_diff)), 'AC', 'XY'),
                                    Sol((max(a[i], L_BB[i-1][j-1][k].time[0]+W_diff), max(b[j], L_BB[i-1][j-1][k].time[1]+W_same)), 'BB', 'XY'),
                                    Sol((max(a[i], max(b[j], L_BB[i-1][j-1][k].time[0]+W_same)+W_diff), L_BB[i-1][j-1][k].time[1]), 'BB', 'XX'),
                                    Sol((max(a[i], L_BC[i-1][j-1][k].time[0]+W_diff), max(b[j], L_BC[i-1][j-1][k].time[1]+W_diff)), 'BC', 'XY'),
                                    key=get_obj
                )
                L_AC[i][j][k] = min(
                                    Sol((max(a[i], L_AB[i-1][j][k-1].time[0]+W_same), max(c[k], L_AB[i-1][j][k-1].time[1]+W_diff)), 'AB', 'XY'),
                                    Sol((max(a[i], L_AC[i-1][j][k-1].time[0]+W_same), max(c[k], L_AC[i-1][j][k-1].time[1]+W_same)), 'AC', 'XY'),
                                    Sol((max(a[i], L_BB[i-1][j][k-1].time[0]+W_diff), max(c[k], L_BB[i-1][j][k-1].time[1]+W_diff)), 'BB', 'XY'),
                                    Sol((max(a[i], L_BC[i-1][j][k-1].time[0]+W_diff), max(c[k], L_BC[i-1][j][k-1].time[1]+W_same)), 'BC', 'XY'),
                                    key=get_obj
                )
                L_BC[i][j][k] = min(
                                    Sol((max(b[j], L_AB[i][j-1][k-1].time[0]+W_diff), max(c[k], L_AB[i][j-1][k-1].time[1]+W_diff)), 'AB', 'XY'),
                                    Sol((max(b[j], L_AC[i][j-1][k-1].time[0]+W_diff), max(c[k], L_AC[i][j-1][k-1].time[1]+W_same)), 'AC', 'XY'),
                                    Sol((max(b[j], L_BB[i][j-1][k-1].time[0]+W_same), max(c[k], L_BB[i][j-1][k-1].time[1]+W_diff)), 'BB', 'XY'),
                                    Sol((L_BB[i][j-1][k-1].time[0], max(c[k], max(b[j], L_BB[i][j-1][k-1].time[1]+W_same)+W_diff)), 'BB', 'YY'),
                                    Sol((max(b[j], L_BC[i][j-1][k-1].time[0]+W_same), max(c[k], L_BC[i][j-1][k-1].time[1]+W_same)), 'BC', 'XY'),
                                    Sol((L_BC[i][j-1][k-1].time[0], max(c[k], max(b[j], L_BC[i][j-1][k-1].time[1]+W_diff)+W_diff)), 'BC', 'YY'),
                                    key=get_obj
                )
                L_BB[i][j][k] = min(
                                    Sol((max(b[j-1], L_AB[i][j-2][k].time[0]+W_diff), max(b[j], L_AB[i][j-2][k].time[1]+W_same)), 'AB', 'XY'),
                                    Sol((max(b[j], L_AB[i][j-2][k].time[0]+W_diff), max(b[j-1], L_AB[i][j-2][k].time[1]+W_same)), 'AB', 'YX'),
                                    Sol((max(b[j-1], L_AC[i][j-2][k].time[0]+W_diff), max(b[j], L_AC[i][j-2][k].time[1]+W_diff)), 'AC', 'XY'),
                                    Sol((max(b[j], L_AC[i][j-2][k].time[0]+W_diff), max(b[j-1], L_AC[i][j-2][k].time[1]+W_diff)), 'AC', 'YX'),
                                    Sol((max(b[j-1], L_BB[i][j-2][k].time[0]+W_same), max(b[j], L_BB[i][j-2][k].time[1]+W_same)), 'BB', 'XY'),
                                    Sol((max(b[j], L_BB[i][j-2][k].time[0]+W_same), max(b[j-1], L_BB[i][j-2][k].time[1]+W_same)), 'BB', 'YX'),
                                    Sol((max(b[j], max(b[j-1], L_BB[i][j-2][k].time[0]+W_same)+W_same), L_BB[i][j-2][k].time[1]), 'BB', 'XX'),
                                    Sol((L_BB[i][j-2][k].time[0], max(b[j], max(b[j-1], L_BB[i][j-2][k].time[1]+W_same)+W_same)), 'BB', 'YY'),
                                    Sol((max(b[j-1], L_BC[i][j-2][k].time[0]+W_same), max(b[j], L_BC[i][j-2][k].time[1]+W_diff)), 'BC', 'XY'),
                                    Sol((max(b[j], L_BC[i][j-2][k].time[0]+W_same), max(b[j-1], L_BC[i][j-2][k].time[1]+W_diff)), 'BC', 'YX'),
                                    Sol((L_BC[i][j-2][k].time[0], max(b[j], max(b[j-1], L_BC[i][j-2][k].time[1]+W_same)+W_diff)), 'BC', 'YY'),
                                    key=get_obj
                )   

    # Push order to stack
    stack_X = []
    stack_Y = []
    i = alpha
    j = beta
    k = gamma
    # Choose the optimal solution and the table to start backtracking
    opt = min(L_AB[i][j][k], L_AC[i][j][k], L_BB[i][j][k], L_BC[i][j][k], key=get_obj)
    table = ''
    lanes = ''
    print(opt)
    if  opt.time == L_AB[i][j][k].time:
        print('AB')
        table = L_AB[i][j][k].table
        lanes = L_AB[i][j][k].lane
        if  lanes == 'XY':
            stack_X.append(('A', i, L_AB[i][j][k].time[0]))
            stack_Y.append(('B', j, L_AB[i][j][k].time[1]))
            i -= 1
            j -= 1
        elif lanes == 'XX':
            stack_X.append(('A', i, L_AB[i][j][k].time[0]))
            stack_X.append(('B', j, L_AB[i-1][j][k].time[0]))
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
        elif lanes == 'YX':
            stack_Y.append(('B', j-1, L_BB[i][j][k].time[1]))
            stack_X.append(('B', j, L_BB[i][j][k].time[0]))
        elif lanes == 'XX':
            stack_X.append(('B', j, L_BB[i][j][k].time[0]))
            stack_X.append(('B', j-1, L_BB[i][j-1][k].time[0]))
        elif lanes == 'YY':
            stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
            stack_Y.append(('B', j-1, L_BB[i][j-1][k].time[1]))
        j -= 2
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
            stack_Y.append(('B', j, L_BC[i][j][k-1].time[1]))
            j -= 1
            k -= 1
        elif lanes[0] == 'X':
            stack_X.append(('B', j, L_BC[i][j][k].time[0]))
            j -= 1
        elif lanes[1] == 'Y':
            stack_Y.append(('C', k, L_BC[i][j][k].time[1]))
            k -= 1

    # Backtracking
    while i>0 or j>0 or k>0:
        print(table, i, j, k)
        if  table == 'AB':
            print('AB')
            table = L_AB[i][j][k].table
            lanes = L_AB[i][j][k].lane
            if  lanes == 'XY':
                stack_X.append(('A', i, L_AB[i][j][k].time[0]))
                stack_Y.append(('B', j, L_AB[i][j][k].time[1]))
                i -= 1
                j -= 1
            elif lanes == 'XX':
                stack_X.append(('A', i, L_AB[i][j][k].time[0]))
                stack_X.append(('B', j, L_AB[i-1][j][k].time[0]))
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
        elif  table == 'BB':
            print('BB')
            table = L_BB[i][j][k].table
            lanes = L_BB[i][j][k].lane
            if lanes == 'XY':
                stack_X.append(('B', j-1, L_BB[i][j][k].time[0]))
                stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
            elif lanes == 'YX':
                stack_Y.append(('B', j-1, L_BB[i][j][k].time[1]))
                stack_X.append(('B', j, L_BB[i][j][k].time[0]))
            elif lanes == 'XX':
                stack_X.append(('B', j, L_BB[i][j][k].time[0]))
                stack_X.append(('B', j-1, L_BB[i][j-1][k].time[0]))
            elif lanes == 'YY':
                stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
                stack_Y.append(('B', j-1, L_BB[i][j-1][k].time[1]))
            j -= 2
        elif  table == 'BC':
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
                stack_Y.append(('B', j, L_BC[i][j][k-1].time[1]))
                j -= 1
                k -= 1
            elif lanes[0] == 'X':
                stack_X.append(('B', j, L_BC[i][j][k].time[0]))
                j -= 1
            elif lanes[1] == 'Y':
                stack_Y.append(('C', k, L_BC[i][j][k].time[1]))
                k -= 1

    computeTime = time.time() - t0
    T_last = stack_X[0][2] if stack_X[0][2] >= stack_Y[0][2] else stack_Y[0][2]
    
    # Delete the redundant element (i==0 or j==0 or k==0)
    while stack_X[-1][1] <= 0:
        stack_X.pop()
    while stack_Y[-1][1] <= 0:
        stack_Y.pop()

    # Output order
    print('lane X:')
    while len(stack_X) > 0:
        print(stack_X.pop())
    print('-----------------------')
    while len(stack_Y) > 0:
        print(stack_Y.pop())

    return T_last, computeTime


def main():
    timeStep = 1
    try:
        W_same = float(sys.argv[3]) # the waiting time if two consecutive vehicles are from the same lane
        W_diff = float(sys.argv[4])  # the waiting time if two consecutive vehicles are from different lanes
        alpha = int(sys.argv[2])
        beta = int(sys.argv[2])
        gamma = int(sys.argv[2])
        p = float(sys.argv[1])
    except:
        print('Arguments: lambda, N, W=, W+')
        return

    generate_traffic(timeStep, alpha, beta, gamma, p)
    a_all = [0, 8.0, 12.0, 38.0, 42.0, 47.0, 72.0, 77.0, 92.0, 109.0, 113.0, 119.0, 121.0, 136.0, 137.0, 138.0, 142.0, 148.0, 149.0, 150.0, 154.0, 157.0, 163.0, 168.0, 175.0, 179.0, 190.0, 198.0, 208.0, 218.0, 232.0, 233.0, 252.0, 254.0, 275.0, 279.0, 295.0, 297.0, 298.0, 304.0, 305.0, 308.0, 317.0, 334.0, 349.0, 355.0, 356.0, 362.0, 363.0, 365.0, 374.0, 383.0, 391.0, 405.0, 426.0, 433.0, 439.0, 470.0, 472.0, 478.0, 481.0, 486.0, 498.0, 502.0, 506.0, 513.0, 530.0, 532.0, 533.0, 537.0, 549.0, 560.0, 595.0, 624.0, 625.0, 628.0, 635.0, 636.0, 637.0, 641.0, 670.0, 671.0, 691.0, 692.0, 700.0, 705.0, 709.0, 713.0, 714.0, 716.0, 725.0, 731.0, 741.0, 751.0, 757.0, 759.0, 770.0, 775.0, 777.0, 779.0, 780.0]
    b_all = [0, 20.0, 22.0, 24.0, 41.0, 43.0, 48.0, 57.0, 58.0, 59.0, 60.0, 67.0, 81.0, 84.0, 86.0, 87.0, 95.0, 99.0, 144.0, 147.0, 153.0, 161.0, 167.0, 171.0, 174.0, 188.0, 195.0, 201.0, 205.0, 207.0, 209.0, 213.0, 216.0, 220.0, 222.0, 226.0, 235.0, 240.0, 248.0, 249.0, 265.0, 269.0, 276.0, 307.0, 321.0, 323.0, 344.0, 368.0, 377.0, 378.0, 385.0, 388.0, 390.0, 409.0, 410.0, 415.0, 444.0, 448.0, 452.0, 453.0, 459.0, 475.0, 497.0, 499.0, 510.0, 523.0, 524.0, 526.0, 551.0, 556.0, 568.0, 599.0, 600.0, 608.0, 609.0, 611.0, 612.0, 613.0, 617.0, 622.0, 639.0, 648.0, 650.0, 652.0, 667.0, 685.0, 689.0, 695.0, 696.0, 697.0, 698.0, 704.0, 718.0, 722.0, 724.0, 736.0, 743.0, 754.0, 760.0, 766.0, 769.0]
    c_all = [0, 1.0, 2.0, 10.0, 16.0, 35.0, 39.0, 49.0, 54.0, 75.0, 78.0, 89.0, 91.0, 97.0, 100.0, 103.0, 104.0, 108.0, 111.0, 115.0, 118.0, 131.0, 134.0, 141.0, 152.0, 164.0, 172.0, 180.0, 182.0, 183.0, 186.0, 191.0, 192.0, 193.0, 199.0, 200.0, 217.0, 221.0, 234.0, 259.0, 280.0, 283.0, 284.0, 286.0, 287.0, 289.0, 301.0, 311.0, 312.0, 318.0, 335.0, 337.0, 352.0, 359.0, 361.0, 370.0, 382.0, 395.0, 413.0, 419.0, 421.0, 424.0, 428.0, 445.0, 450.0, 454.0, 471.0, 473.0, 500.0, 503.0, 509.0, 511.0, 520.0, 521.0, 545.0, 550.0, 552.0, 565.0, 578.0, 582.0, 583.0, 586.0, 589.0, 598.0, 610.0, 619.0, 620.0, 626.0, 627.0, 630.0, 638.0, 642.0, 643.0, 645.0, 660.0, 663.0, 665.0, 669.0, 686.0, 699.0, 711.0]
    split_traffic(a_all, b_all, c_all, p)

    # print('a', a_all)
    # print('b', b_all)
    # print('c', c_all)
    # multiDim_dp(W_same, W_diff)

if __name__ == '__main__':
    main()