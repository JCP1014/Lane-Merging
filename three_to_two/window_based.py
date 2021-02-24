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
    
    # Delete the redundent element (i==0 or j==0 or k==0)
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
    print('a', a_all)
    print('b', b_all)
    print('c', c_all)
    # multiDim_dp(W_same, W_diff)

if __name__ == '__main__':
    main()