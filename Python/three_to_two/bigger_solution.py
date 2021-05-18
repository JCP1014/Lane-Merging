import numpy as np
import random
import time
import sys
import copy
from collections import namedtuple
from three_to_two import oneSol_multiDim_dp

# The tuple of solution stored in each "square" in the table
Sol = namedtuple("Sol", "time table idx lane")    # Each solution has three fields.
                                                    # "time" means the scheduled entering time of the last vehicle
                                                    # "table" means which table the optimal solution is from
                                                    # "idx" means the index of the solution in the source table
                                                    # "lane" means which lane the last vehicle go to

Lane = namedtuple("Lane", "traffic num")    # Each lane has two fields
                                            # "traffic" field means the list of earliest arrival time of vehicles in this lane 
                                            # "num" field means the expected number of vehicles to generate in this lane
# (Version 1)
# Randomly generate earliest arrival time
def generate_traffic_v1(timeStep, alpha, beta, gamma, pA, pB, pC):
    a = [0] # List of earliest arrival time of vehicles in lane A
    b = [0] # List of earliest arrival time of vehicles in lane B
    c = [0] # List of earliest arrival time of vehicles in lane C
    
    alpha_tmp = alpha   # temporary variable for countdown
    beta_tmp = beta
    gamma_tmp = gamma

    # Randomly generate earliest arrival time of vehicles in lane A
    t = 1.0 # time(sec)
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


# (Version 2)
# Randomly generate earliest arrival time
def generate_traffic_v2(timeStep, alpha, beta, gamma, p):   
    a = [0] # List of earliest arrival time of vehicles in lane A
    b = [0] # List of earliest arrival time of vehicles in lane B
    c = [0] # List of earliest arrival time of vehicles in lane C
    num = alpha + beta + gamma
    t = 1.0
    cars = []
    while num> 0:
        if np.random.uniform(0, 1) < p:
            cars.append(round(t, 1))
            num -= 1
        t += timeStep

    # print(a_all)
    A = Lane(a, alpha)
    B = Lane(b, beta)
    C = Lane(c, gamma)
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
    return a, b, c


# Print the content of dp table
def print_table(L, name):
    print(name)
    for k in range(len(L[0][0])):
        print('k =',k)
        for i in range(len(L)):
            for j in range(len(L[0])):
                # print(str(L[i][j][k].time)+'('+L[i][j][k].table+')'+'('+L[i][j][k].lane+')', end=' ')
                print(len(L[i][j][k]), end=' ')
            print('')
        print('')


def get_obj(sol):
    return max(sol.time)
    
# Three-dimensional DP which keeps multiple solutions in each square
def allSol_multiDim_dp(a, b, c, W_same, W_diff):
    alpha = len(a) - 1  # Number of vehicles in lane A
    beta = len(b) - 1   # Number of vehicles in lane B
    gamma = len(c) - 1  # Number of vehicles in lane C
    
    # The three-dimensional DP table,
    # the first character after the underscore means which lane that the last vehicle going to lane X is from, and
    # the second character after the underscore means which lane that the last vehicle going to lane X is from.
    L_AB = [[[ [Sol((float('inf'), float('inf')), '', 0, '')] for k in range(gamma+1) ] for j in range(beta+1)] for i in range(alpha+1)]
    L_AC = [[[ [Sol((float('inf'), float('inf')), '', 0, '')] for k in range(gamma+1) ] for j in range(beta+1)] for i in range(alpha+1)]
    L_BB = [[[ [Sol((float('inf'), float('inf')), '', 0, '')] for k in range(gamma+1) ] for j in range(beta+1)] for i in range(alpha+1)]
    L_BC = [[[ [Sol((float('inf'), float('inf')), '', 0, '')] for k in range(gamma+1) ] for j in range(beta+1)] for i in range(alpha+1)]

    t0 = time.time()
    # Initialize
    L_AB[0][0][0] = [Sol((0.0, 0.0), '', 0, '')]
    L_AC[0][0][0] = [Sol((0.0, 0.0), '', 0, '')]
    L_BB[0][0][0] = [Sol((0.0, 0.0), '', 0, '')]
    L_BC[0][0][0] = [Sol((0.0, 0.0), '', 0, '')]
    L_AB[1][1][0] = [Sol((a[1], b[1]), 'AB', 0, 'XY')]
    L_AC[1][0][1] = [Sol((a[1], c[1]), 'AC', 0, 'XY')]
    L_BC[0][1][1] = [Sol((b[1], c[1]), 'BC', 0, 'XY')]
    L_BB[0][1][0] = [Sol((-W_diff, b[1]), 'BB', 0, '0Y'),
                    Sol((b[1], -W_diff), 'BB', 0, 'X0')]
    if beta >= 2:
        L_BB[0][2][0] = [Sol((b[1], b[2]), 'BB', 0, 'XY'),
                        Sol((b[2], b[1]), 'BB', 0, 'YX')]
    for i in range(2, alpha+1):
        L_AB[i][1][0] = [ Sol((max(a[i], s.time[0]+W_same), b[1]), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i-1][1][0]) if not float('inf') in s.time]
    for i in range(2, alpha+1):
        L_AC[i][0][1] = [ Sol((max(a[i], s.time[0]+W_same), c[1]), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i-1][0][1]) if not float('inf') in s.time]
    for k in range(2, gamma+1):
        L_AC[1][0][k] = [ Sol((a[1], max(c[k], s.time[1]+W_same)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[1][0][k-1]) if not float('inf') in s.time]
    for k in range(2, gamma+1):
        L_BC[0][1][k] = [ Sol((b[1], max(c[k], s.time[1]+W_same)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[0][1][k-1]) if not float('inf') in s.time]
    if beta >= 3:
        for j in range(3, beta+1):
            L_BB[0][j][0] = [ Sol((max(b[j-1], s.time[0]+W_same), max(b[j], s.time[1]+W_same)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[0][j-2][0]) if not float('inf') in s.time ] + \
                            [ Sol((max(b[j], s.time[0]+W_same), max(b[j-1], s.time[1]+W_same)), 'BB', idx, 'YX') for idx, s in enumerate(L_BB[0][j-2][0]) if not float('inf') in s.time ] + \
                            [ Sol((max(b[j], max(b[j-1], s.time[0]+W_same)+W_same), s.time[1]), 'BB', idx, 'XX') for idx, s in enumerate(L_BB[0][j-2][0]) if not float('inf') in s.time ] + \
                            [ Sol((s.time[0], max(b[j], max(b[j-1], s.time[1]+W_same)+W_same)), 'BB', idx, 'YY') for idx, s in enumerate(L_BB[0][j-2][0]) if not float('inf') in s.time ]
    L_AB[1][0][0] = [Sol((a[1], -W_diff), 'AB', 0, 'X0')]
    for i in range(2, alpha+1):
        L_AB[i][0][0] = [ Sol((max(a[i], s.time[0]+W_same), -W_diff), 'AB', idx, 'X0') for idx, s in enumerate(L_AB[i-1][0][0]) if not float('inf') in s.time ]
    L_AB[0][1][0] = [Sol((-W_diff, b[1]), 'AB', 0, '0Y')]
    for j in range(2, beta+1):
        L_AB[0][j][0] = [ Sol((-W_diff, max(b[j], s.time[1]+W_same)), 'AB', idx, '0Y') for idx, s in enumerate(L_AB[0][j-1][0]) if not float('inf') in s.time ]
    L_AC[1][0][0] = [Sol((a[1], -W_diff), 'AC', 0, 'X0')]
    for i in range(2, alpha+1):
        L_AC[i][0][0] = [ Sol((max(a[i], s.time[0]+W_same), -W_diff), 'AC', idx, 'X0') for idx, s in enumerate(L_AC[i-1][0][0]) if not float('inf') in s.time ]
    L_AC[0][0][1] = [Sol((-W_diff, c[1]), 'AC', 0, '0Y')]
    for k in range(2, gamma+1):
        L_AC[0][0][k] = [ Sol((-W_diff, max(c[k], s.time[1]+W_same)), 'AC', idx, '0Y') for idx, s in enumerate(L_AC[0][0][k-1]) if not float('inf') in s.time ]
    L_BC[0][1][0] = [Sol((b[1], -W_diff), 'BC', 0, 'X0')]
    for j in range(2, beta+1):
        L_BC[0][j][0] = [ Sol((max(b[j], s.time[0]+W_same), -W_diff), 'BC', idx, 'X0') for idx, s in enumerate(L_BC[0][j-1][0]) if not float('inf') in s.time ]
    L_BC[0][0][1] = [Sol((-W_diff, c[1]), 'BC', 0, '0Y')]
    for k in range(2, gamma+1):
        L_BC[0][0][k] = [ Sol((-W_diff, max(c[k], s.time[1]+W_same)), 'BC', idx, '0Y') for idx, s in enumerate(L_BC[0][0][k-1]) if not float('inf') in s.time ]
    if beta >= 2:
        for i in range(1, alpha+1):
            L_BB[i][2][0] = [ Sol((max(b[1], s.time[0]+W_diff), b[2]), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time ] + \
                            [ Sol((max(b[2], s.time[0]+W_diff), b[1]), 'AC', idx, 'YX') for idx, s in enumerate(L_AC[i][0][0]) if not float('inf') in s.time ]
        for k in range(1, gamma+1):
            L_BB[0][2][k] = [ Sol((b[1], max(b[2], s.time[1]+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time ] + \
                            [ Sol((b[2], max(b[1], s.time[1]+W_diff)), 'AC', idx, 'YX') for idx, s in enumerate(L_AC[0][0][k]) if not float('inf') in s.time ]
    for i in range(1, alpha+1):
        for j in range(2, beta+1):
            L_AB[i][j][0] = [ Sol((max(a[i], s.time[0]+W_same), max(b[j], s.time[1]+W_same)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i-1][j-1][0]) if not float('inf') in s.time ] + \
                            [ Sol((max(a[i], max(b[j], s.time[0]+W_diff)+W_diff), s.time[1]), 'AB', idx, 'XX') for idx, s in enumerate(L_AB[i-1][j-1][0]) if not float('inf') in s.time ] + \
                            [ Sol((max(a[i], s.time[0]+W_diff), max(b[j], s.time[1]+W_same)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i-1][j-1][0]) if not float('inf') in s.time ] + \
                            [ Sol((max(a[i], max(b[j], s.time[0]+W_same)+W_diff), s.time[1]), 'BB', idx, 'XX') for idx, s in enumerate(L_BB[i-1][j-1][0]) if not float('inf') in s.time ]
    for i in range(2, alpha+1):
        for k in range(2, gamma+1):
            L_AC[i][0][k] = [ Sol((max(a[i], s.time[0]+W_same), max(c[k], s.time[1]+W_same)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i-1][0][k-1]) if not float('inf') in s.time ]
    for j in range(2, beta+1):
        for k in range(1, gamma+1):
            L_BC[0][j][k] = [ Sol((max(b[j], s.time[0]+W_same), max(c[k], s.time[1]+W_diff)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[0][j-1][k-1]) if not float('inf') in s.time ] + \
                            [ Sol((s.time[0], max(c[k], max(b[j], s.time[1]+W_same)+W_diff)), 'BB', idx, 'YY') for idx, s in enumerate(L_BB[0][j-1][k-1]) if not float('inf') in s.time ] + \
                            [ Sol((max(b[j], s.time[0]+W_same), max(c[k], s.time[1]+W_same)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[0][j-1][k-1]) if not float('inf') in s.time ] + \
                            [ Sol((s.time[0], max(c[k], max(b[j], s.time[1]+W_diff)+W_diff)), 'BC', idx, 'YY') for idx, s in enumerate(L_BC[0][j-1][k-1]) if not float('inf') in s.time ]
    for i in range(1, alpha+1):
        for j in range(3, beta+1):
            L_BB[i][j][0] = [ Sol((max(b[j-1], s.time[0]+W_diff), max(b[j], s.time[1]+W_same)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i][j-2][0]) if not float('inf') in s.time ] + \
                            [ Sol((max(b[j], s.time[0]+W_diff), max(b[j-1], s.time[1]+W_same)), 'AB', idx, 'YX') for idx, s in enumerate(L_AB[i][j-2][0]) if not float('inf') in s.time ] + \
                            [ Sol((max(b[j-1], s.time[0]+W_same), max(b[j], s.time[1]+W_same)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i][j-2][0]) if not float('inf') in s.time ] + \
                            [ Sol((max(b[j], s.time[0]+W_same), max(b[j-1], s.time[1]+W_same)), 'BB', idx, 'YX') for idx, s in enumerate(L_BB[i][j-2][0]) if not float('inf') in s.time ] + \
                            [ Sol((max(b[j], max(b[j-1], s.time[0]+W_same)+W_same), s.time[1]), 'BB', idx, 'XX') for idx, s in enumerate(L_BB[i][j-2][0]) if not float('inf') in s.time ] + \
                            [ Sol((s.time[0], max(b[j], max(b[j-1], s.time[1]+W_same)+W_same)), 'BB', idx, 'YY') for idx, s in enumerate(L_BB[i][j-2][0]) if not float('inf') in s.time ]
    
    # Compute tables
    for i in range(1, alpha+1):
        for j in range(1, beta+1):
            for k in range(1, gamma+1):
                L_AB[i][j][k] = [ Sol((max(a[i], s.time[0]+W_same), max(b[j], s.time[1]+W_same)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i-1][j-1][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(a[i], max(b[j], s.time[0]+W_diff)+W_diff), s.time[1]), 'AB', idx, 'XX') for idx, s in enumerate(L_AB[i-1][j-1][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(a[i], s.time[0]+W_same), max(b[j], s.time[1]+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i-1][j-1][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(a[i], s.time[0]+W_diff), max(b[j], s.time[1]+W_same)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i-1][j-1][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(a[i], max(b[j], s.time[0]+W_same)+W_diff), s.time[1]), 'BB', idx, 'XX') for idx, s in enumerate(L_BB[i-1][j-1][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(a[i], s.time[0]+W_diff), max(b[j], s.time[1]+W_diff)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[i-1][j-1][k]) if not float('inf') in s.time ]

                L_AC[i][j][k] = [ Sol((max(a[i], s.time[0]+W_same), max(c[k], s.time[1]+W_diff)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i-1][j][k-1]) if not float('inf') in s.time ] + \
                                [ Sol((max(a[i], s.time[0]+W_same), max(c[k], s.time[1]+W_same)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i-1][j][k-1]) if not float('inf') in s.time ] + \
                                [ Sol((max(a[i], s.time[0]+W_diff), max(c[k], s.time[1]+W_diff)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i-1][j][k-1]) if not float('inf') in s.time ] + \
                                [ Sol((max(a[i], s.time[0]+W_diff), max(c[k], s.time[1]+W_same)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[i-1][j][k-1]) if not float('inf') in s.time ]

                L_BC[i][j][k] = [ Sol((max(b[j], s.time[0]+W_diff), max(c[k], s.time[1]+W_diff)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i][j-1][k-1]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j], s.time[0]+W_diff), max(c[k], s.time[1]+W_same)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i][j-1][k-1]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j], s.time[0]+W_same), max(c[k], s.time[1]+W_diff)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i][j-1][k-1]) if not float('inf') in s.time ] + \
                                [ Sol((s.time[0], max(c[k], max(b[j], s.time[1]+W_same)+W_diff)), 'BB', idx, 'YY') for idx, s in enumerate(L_BB[i][j-1][k-1]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j], s.time[0]+W_same), max(c[k], s.time[1]+W_same)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[i][j-1][k-1]) if not float('inf') in s.time ] + \
                                [ Sol((s.time[0], max(c[k], max(b[j], s.time[1]+W_diff)+W_diff)), 'BC', idx, 'YY') for idx, s in enumerate(L_BC[i][j-1][k-1]) if not float('inf') in s.time ]

                L_BB[i][j][k] = [ Sol((max(b[j-1], s.time[0]+W_diff), max(b[j], s.time[1]+W_same)), 'AB', idx, 'XY') for idx, s in enumerate(L_AB[i][j-2][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j], s.time[0]+W_diff), max(b[j-1], s.time[1]+W_same)), 'AB', idx, 'YX') for idx, s in enumerate(L_AB[i][j-2][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j-1], s.time[0]+W_diff), max(b[j], s.time[1]+W_diff)), 'AC', idx, 'XY') for idx, s in enumerate(L_AC[i][j-2][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j], s.time[0]+W_diff), max(b[j-1], s.time[1]+W_diff)), 'AC', idx, 'YX') for idx, s in enumerate(L_AC[i][j-2][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j-1], s.time[0]+W_same), max(b[j], s.time[1]+W_same)), 'BB', idx, 'XY') for idx, s in enumerate(L_BB[i][j-2][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j], s.time[0]+W_same), max(b[j-1], s.time[1]+W_same)), 'BB', idx, 'YX') for idx, s in enumerate(L_BB[i][j-2][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j], max(b[j-1], s.time[0]+W_same)+W_same), s.time[1]), 'BB', idx, 'XX') for idx, s in enumerate(L_BB[i][j-2][k]) if not float('inf') in s.time ] + \
                                [ Sol((s.time[0], max(b[j], max(b[j-1], s.time[1]+W_same)+W_same)), 'BB', idx, 'YY') for idx, s in enumerate(L_BB[i][j-2][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j-1], s.time[0]+W_same), max(b[j], s.time[1]+W_diff)), 'BC', idx, 'XY') for idx, s in enumerate(L_BC[i][j-2][k]) if not float('inf') in s.time ] + \
                                [ Sol((max(b[j], s.time[0]+W_same), max(b[j-1], s.time[1]+W_diff)), 'BC', idx, 'YX') for idx, s in enumerate(L_BC[i][j-2][k]) if not float('inf') in s.time ] + \
                                [ Sol((s.time[0], max(b[j], max(b[j-1], s.time[1]+W_same)+W_diff)), 'BC', idx, 'YY') for idx, s in enumerate(L_BC[i][j-2][k]) if not float('inf') in s.time ]

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
    while i>0 or j>0 or k>0:
        print(table, i, j, k, idx)
        if  table == 'AB':
            # print('AB')
            lanes = L_AB[i][j][k][idx].lane
            table = L_AB[i][j][k][idx].table
            if  lanes == 'XY':
                stack_X.append(('A', i, L_AB[i][j][k][idx].time[0]))
                stack_Y.append(('B', j, L_AB[i][j][k][idx].time[1]))
                idx = L_AB[i][j][k][idx].idx
                i -= 1
                j -= 1
            elif lanes == 'XX':
                stack_X.append(('A', i, L_AB[i][j][k][idx].time[0]))
                if table == 'AB':
                    stack_X.append(('B', j, max(b[j], L_AB[i-1][j-1][k][idx].time[0]+W_diff)))
                elif table == 'BB':
                    stack_X.append(('B', j, max(b[j], L_BB[i-1][j-1][k][idx].time[0]+W_same)))
                else:
                    print('bug')
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
        elif  table == 'BB':
            # print('BB')
            lanes = L_BB[i][j][k][idx].lane
            table = L_BB[i][j][k][idx].table
            if lanes == 'XY':
                stack_X.append(('B', j-1, L_BB[i][j][k][idx].time[0]))
                stack_Y.append(('B', j, L_BB[i][j][k][idx].time[1]))
            elif lanes == 'YX':
                stack_Y.append(('B', j-1, L_BB[i][j][k][idx].time[1]))
                stack_X.append(('B', j, L_BB[i][j][k][idx].time[0]))
            elif lanes == 'XX':
                stack_X.append(('B', j, L_BB[i][j][k][idx].time[0]))
                stack_X.append(('B', j-1, max(b[j-1], L_BB[i][j-2][k][idx].time[0]+W_same)))
            elif lanes == 'YY':
                stack_Y.append(('B', j, L_BB[i][j][k][idx].time[1]))
                if table == 'BB':
                    stack_Y.append(('B', j-1, max(b[j-1], L_BB[i][j-2][k][idx].time[1]+W_same)))
                elif table == 'BC':
                    stack_Y.append(('B', j-1, max(b[j-1], L_BC[i][j-2][k][idx].time[1]+W_same)))
                else:
                    print('bug')
            idx = L_BB[i][j][k][idx].idx
            j -= 2
        elif  table == 'BC':
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
                if table == 'BB':
                    stack_Y.append(('B', j, max(b[j], L_BB[i][j-1][k-1][idx].time[1]+W_same)))
                elif table == 'BC':
                    stack_Y.append(('B', j, max(b[j], L_BC[i][j-1][k-1][idx].time[1]+W_diff)))
                else:
                    print('bug')
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
    while len(stack_X) > 0:
        print(stack_X.pop())
    print('-----------------------')
    while len(stack_Y) > 0:
        print(stack_Y.pop())

    return T_last, computeTime
    

# First-Come-First-Go
def fcfg(timeStep, a, b, c, W_same, W_diff):  # v5
    a = a[1:]
    b = b[1:]
    c = c[1:]
    t = 0   # time
    B_prevTo = 'Y'  # Which lane the previous vehicle in lane B go to
    X_lastT = -W_diff   
    Y_lastT = -W_diff
    X_lastFrom = ''
    Y_lastFrom = ''
        
    while len(a) > 0 or len(b) > 0 or len(c) > 0:
        t = round(t+timeStep, 2)
        if len(a) > 0 and len(b) > 0 and len(c) > 0:
            if a[0] == t and b[0] == t and c[0] == t:
                if len(a) > 1 and len(b) > 1 and len(c) > 1: 
                    if b[1] >= a[1] and b[1] >= c[1]:
                        if X_lastFrom == 'A':
                            X_lastT = max(a.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(a.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'A'
                        if Y_lastFrom == 'C':
                            Y_lastT = max(c.pop(0), Y_lastT+W_same)
                        else:
                            Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'C'
                        if X_lastT <= t and Y_lastT <= t:
                            if a[0] >= c[0]:
                                X_lastT = max(b.pop(0), X_lastT+W_diff)
                                X_lastFrom = 'B'
                            else:
                                Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                                Y_lastFrom = 'B'
                        else:
                            if X_lastT < Y_lastT:
                                X_lastT = max(b.pop(0), X_lastT+W_diff)
                                X_lastFrom = 'B'
                            elif Y_lastT < X_lastT:
                                Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                                Y_lastFrom = 'B'
                            else:
                                if a[0] >= c[0]:
                                    X_lastT = max(b.pop(0), X_lastT+W_diff)
                                    X_lastFrom = 'B'
                                else:
                                    Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                                    Y_lastFrom = 'B'
                    elif a[1] >= b[1] and a[1] >= c[1]:
                        if X_lastFrom == 'B':
                            X_lastT = max(b.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'B'
                        if Y_lastFrom == 'C':
                            Y_lastT = max(c.pop(0), Y_lastT+W_same)
                        else:
                            Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'C'
                        X_lastT = max(a.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'A'
                    elif c[1] >= a[1] and c[1] >= b[1]:
                        if X_lastFrom == 'A':
                            X_lastT = max(a.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(a.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'A'
                        if Y_lastFrom == 'B':
                            Y_lastT = max(b.pop(0), Y_lastT+W_same)
                        else:
                            Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'B'
                        Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'C'
                elif len(b) < 2:
                    if X_lastFrom == 'A':
                        X_lastT = max(a.pop(0), X_lastT+W_same)
                    else:
                        X_lastT = max(a.pop(0), X_lastT+W_diff)
                    X_lastFrom = 'A'
                    if Y_lastFrom == 'C':
                        Y_lastT = max(c.pop(0), Y_lastT+W_same)
                    else:
                        Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                    Y_lastFrom = 'C'
                    if X_lastT <= t and Y_lastT <= t:
                        if len(a) < 1:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                            X_lastFrom = 'B'
                        elif len(c) < 1:
                            Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                            Y_lastFrom = 'B'
                        else:
                            if a[0] >= c[0]:
                                X_lastT = max(b.pop(0), X_lastT+W_diff)
                                X_lastFrom = 'B'
                            else:
                                Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                                Y_lastFrom = 'B'
                    else:
                        if X_lastT < Y_lastT:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                            X_lastFrom = 'B'
                        elif Y_lastT < X_lastT:
                            Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                            Y_lastFrom = 'B'
                        else:
                            if len(a) < 1:
                                X_lastT = max(b.pop(0), X_lastT+W_diff)
                                X_lastFrom = 'B'
                            elif len(c) < 1:
                                Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                                Y_lastFrom = 'B'
                            else:
                                if a[0] >= c[0]:
                                    X_lastT = max(b.pop(0), X_lastT+W_diff)
                                    X_lastFrom = 'B'
                                else:
                                    Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                                    Y_lastFrom = 'B'
                elif len(a) < 2:
                    if X_lastFrom == 'B':
                        X_lastT = max(b.pop(0), X_lastT+W_same)
                    else:
                        X_lastT = max(b.pop(0), X_lastT+W_diff)
                    X_lastFrom = 'B'
                    if Y_lastFrom == 'C':
                        Y_lastT = max(c.pop(0), Y_lastT+W_same)
                    else:
                        Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                    Y_lastFrom = 'C'
                    X_lastT = max(a.pop(0), Y_lastT+W_diff)
                    X_lastFrom = 'A'
                elif len(c) < 2:
                    if X_lastFrom == 'A':
                        X_lastT = max(a.pop(0), X_lastT+W_same)
                    else:
                        X_lastT = max(a.pop(0), X_lastT+W_diff)
                    X_lastFrom = 'A'
                    if Y_lastFrom == 'B':
                        Y_lastT = max(b.pop(0), Y_lastT+W_same)
                    else:
                        Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                    Y_lastFrom = 'B'
                    Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                    Y_lastFrom = 'C'
            elif a[0] == t and c[0] == t:
                if X_lastFrom == 'A':
                    X_lastT = max(a.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(a.pop(0), X_lastT+W_diff)
                X_lastFrom = 'A'
                if Y_lastFrom == 'C':
                    Y_lastT = max(c.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'C'                
            elif a[0] == t and b[0] == t:
                if X_lastFrom == 'A':
                    X_lastT = max(a.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(a.pop(0), X_lastT+W_diff)
                X_lastFrom = 'A'
                if Y_lastFrom == 'B':
                    Y_lastT = max(b.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'B'
            elif b[0] == t and c[0] == t:
                if X_lastFrom == 'B':
                    X_lastT = max(b.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(b.pop(0), X_lastT+W_diff)
                X_lastFrom = 'B'
                if Y_lastFrom == 'C':
                    Y_lastT = max(c.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'C'  
            elif b[0] == t:
                if X_lastT <= t and Y_lastT <= t:
                    if c[0] > a[0]:
                        if Y_lastFrom == 'B':
                            Y_lastT = max(b.pop(0), Y_lastT+W_same)
                        else:
                            Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'B'
                        # print('Y:', Y_lastFrom, Y_lastT)   
                    else:
                        if X_lastFrom == 'B':
                            X_lastT = max(b.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'B'
                        # print('X:', X_lastFrom, X_lastT)
                else:
                    if X_lastT <= t and Y_lastT <= t:
                        if c[0] > a[0]:
                            if Y_lastFrom == 'B':
                                Y_lastT = max(b.pop(0), Y_lastT+W_same)
                            else:
                                Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                            Y_lastFrom = 'B'
                            # print('Y:', Y_lastFrom, Y_lastT)   
                        else:
                            if X_lastFrom == 'B':
                                X_lastT = max(b.pop(0), X_lastT+W_same)
                            else:
                                X_lastT = max(b.pop(0), X_lastT+W_diff)
                            X_lastFrom = 'B'
                            # print('X:', X_lastFrom, X_lastT)
                    else:
                        if X_lastT < Y_lastT:
                            if X_lastFrom == 'B':
                                X_lastT = max(b.pop(0), X_lastT+W_same)
                            else:
                                X_lastT = max(b.pop(0), X_lastT+W_diff)
                            X_lastFrom = 'B'
                        elif Y_lastT < X_lastT:
                            if Y_lastFrom == 'B':
                                Y_lastT = max(b.pop(0), Y_lastT+W_same)
                            else:
                                Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                            Y_lastFrom = 'B'
                        else:
                            if c[0] > a[0]:
                                if Y_lastFrom == 'B':
                                    Y_lastT = max(b.pop(0), Y_lastT+W_same)
                                else:
                                    Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                                Y_lastFrom = 'B'
                                # print('Y:', Y_lastFrom, Y_lastT)   
                            else:
                                if X_lastFrom == 'B':
                                    X_lastT = max(b.pop(0), X_lastT+W_same)
                                else:
                                    X_lastT = max(b.pop(0), X_lastT+W_diff)
                                X_lastFrom = 'B'
                                # print('X:', X_lastFrom, X_lastT)
            elif a[0] == t:
                if X_lastFrom == 'A':
                    X_lastT = max(a.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(a.pop(0), X_lastT+W_diff)
                X_lastFrom = 'A'
                # print('X:', X_lastFrom, X_lastT)
            elif c[0] == t:
                if Y_lastFrom == 'C':
                    Y_lastT = max(c.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'C'
                # print('Y:', Y_lastFrom, Y_lastT)  
        elif len(a) > 0 and len(c) > 0: 
            if a[0] == t and c[0] == t:
                if X_lastFrom == 'A':
                    X_lastT = max(a.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(a.pop(0), X_lastT+W_diff)
                X_lastFrom = 'A'
                if Y_lastFrom == 'C':
                    Y_lastT = max(c.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'C'       
            elif a[0] == t:
                if X_lastFrom == 'A':
                    X_lastT = max(a.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(a.pop(0), X_lastT+W_diff)
                X_lastFrom = 'A'
                # print('X:', X_lastFrom, X_lastT)  
            elif c[0] == t:
                if Y_lastFrom == 'C':
                    Y_lastT = max(c.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'C' 
                # print('Y:', Y_lastFrom, Y_lastT)
        elif len(a) > 0 and len(b) > 0:
            if a[0] == t and b[0] == t:
                if X_lastFrom == 'A':
                    X_lastT = max(a.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(a.pop(0), X_lastT+W_diff)
                X_lastFrom = 'A'
                if Y_lastFrom == 'B':
                    Y_lastT = max(b.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'B'
            elif a[0] == t:
                if X_lastFrom == 'A':
                    X_lastT = max(a.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(a.pop(0), X_lastT+W_diff)
                X_lastFrom = 'A'
                # print('X:', X_lastFrom, X_lastT)
            elif b[0] == t:
                if X_lastT <= t and Y_lastT <= t:
                    if Y_lastFrom == 'B':
                        Y_lastT = max(b.pop(0), Y_lastT+W_same)
                    else:
                        Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                    Y_lastFrom = 'B'
                    # print('Y:', Y_lastFrom, Y_lastT)
                else:
                    if X_lastT < Y_lastT:
                        if X_lastFrom == 'B':
                            X_lastT = max(b.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'B'
                        # print('X:', X_lastFrom, X_lastT)
                    elif Y_lastT < X_lastT:
                        if Y_lastFrom == 'B':
                            Y_lastT = max(b.pop(0), Y_lastT+W_same)
                        else:
                            Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'B'
                        # print('Y:', Y_lastFrom, Y_lastT)
                    else:
                        if Y_lastFrom == 'B':
                            Y_lastT = max(b.pop(0), Y_lastT+W_same)
                        else:
                            Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'B'
                        # print('Y:', Y_lastFrom, Y_lastT)
        elif len(b) > 0 and len(c) > 0:
            if b[0] == t and c[0] == t:
                if X_lastFrom == 'B':
                    X_lastT = max(b.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(b.pop(0), X_lastT+W_diff)
                X_lastFrom = 'B'
                if Y_lastFrom == 'C':
                    Y_lastT = max(c.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'C'
            elif c[0] == t:
                if Y_lastFrom == 'C':
                    Y_lastT = max(c.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'C'
                # print('Y:', Y_lastFrom, Y_lastT)
            elif b[0] == t:
                if X_lastT <= t and Y_lastT <= t:
                    if X_lastFrom == 'B':
                        X_lastT = max(b.pop(0), X_lastT+W_same)
                    else:
                        X_lastT = max(b.pop(0), X_lastT+W_diff)
                    X_lastFrom = 'B'
                    # print('X:', X_lastFrom, X_lastT)
                else:
                    if X_lastT < Y_lastT:
                        if X_lastFrom == 'B':
                            X_lastT = max(b.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'B'
                        # print('X:', X_lastFrom, X_lastT)
                    elif Y_lastT < X_lastT:
                        if Y_lastFrom == 'B':
                            Y_lastT = max(b.pop(0), Y_lastT+W_same)
                        else:
                            Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'B'
                        # print('Y:', Y_lastFrom, Y_lastT)
                    else:
                        if X_lastFrom == 'B':
                            X_lastT = max(b.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'B'
                        # print('X:', X_lastFrom, X_lastT)
        elif len(a) > 0:
            if X_lastFrom == 'A':
                X_lastT = max(a.pop(0), X_lastT+W_same)
            else:
                X_lastT = max(a.pop(0), X_lastT+W_diff)
            X_lastFrom = 'A'
            # print('X:', X_lastFrom, X_lastT)
        elif len(c) > 0:
            if Y_lastFrom == 'C':
                Y_lastT = max(c.pop(0), Y_lastT+W_same)
            else:
                Y_lastT = max(c.pop(0), Y_lastT+W_diff)
            Y_lastFrom = 'C'
            # print('Y:', Y_lastFrom, Y_lastT)
        elif len(b) > 0:
            if B_prevTo == 'Y':
                if X_lastFrom == 'B':
                    X_lastT = max(b.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(b.pop(0), X_lastT+W_diff)
                X_lastFrom = 'B'
                # print('X:', X_lastFrom, X_lastT)
                B_prevTo = 'X'
            else:
                if Y_lastFrom == 'B':
                    Y_lastT = max(b.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'B'
                # print('Y:', Y_lastFrom, Y_lastT)
                B_prevTo = 'Y'

    return max(X_lastT, Y_lastT)


# Command line arguments: lambda, N, W=, W+
def main():
    timeStep = 1    # The precision of time (in second)
    try:
        W_same = float(sys.argv[3]) # the waiting time if two consecutive vehicles are from the same lane
        W_diff = float(sys.argv[4])  # the waiting time if two consecutive vehicles are from different lanes
        alpha = int(sys.argv[2])    # Number of vehicles in lane A
        beta = int(sys.argv[2])     # Number of vehicles in lane B
        gamma = int(sys.argv[2])    # Number of vehicles in lane C
        p = float(sys.argv[1])      # lambda for Poisson distribution
        pA = p / 3     # lambda for Poisson distribution in lane A
        pB = p / 3     # lambda for Poisson distribution in lane B
        pC = p / 3     # lambda for Poisson distribution in lane C
    except:
        print('Arguments: lambda, N, W=, W+')
        return
    a, b, c = generate_traffic_v1(timeStep, alpha, beta, gamma, pA, pB, pC)
    # a, b, c = generate_traffic_v2(timeStep, alpha, beta, gamma, p)
    # a = [0, 1.0, 2.0, 4.0, 5.0, 7.0]
    # b = [0, 5.0, 9.0, 13.0, 14.0, 19.0]
    # c = [0, 1.0, 2.0, 3.0, 6.0, 15.0]
    print(a)
    print(b)
    print(c)

    # Use the previous two-dimentional approach 
    # ret = twoDim_dp(copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff)
    # print('dp:', ret[0], ret[1])
   
    # Use the three-dimensional approach which only keeps min-max in each square
    ret = oneSol_multiDim_dp(copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff)
    print('dp:', ret[0], ret[1])

    # Use the three-dimensional approach which keeps all solution in each square
    ret = allSol_multiDim_dp(copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff)
    print('dp:', ret[0], ret[1])

    # Use FCFG approach
    print('fcfg:', fcfg(timeStep, copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff))



if __name__ == '__main__':
    main()