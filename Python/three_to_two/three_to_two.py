import numpy as np
import random
import time
import sys
import copy
from collections import namedtuple

# The tuple of solution stored in each "square" in the table
Sol = namedtuple("Sol", "time table lane")  # Each solution has three fields.
                                            # "time" means the scheduled entering time of the last vehicle
                                            # "table" means which table the optimal solution is from
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
    
    alpha_tmp = alpha   # temp variable for countdown
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
                print(str(L[i][j][k].time)+'('+L[i][j][k].table+')'+'('+L[i][j][k].lane+')', end=' ')
            print('')
        print('')


# Previous approach, not optimal
def twoDim_dp(a, b, c, W_same, W_diff):
    alpha = len(a) - 1
    beta = len(b) - 1
    gamma = len(c) - 1
    Lx = np.zeros((alpha+1, beta+1, 2))  # dp table
    Ly = np.zeros((gamma+1, beta+1, 2))  # dp table

    t0 = time.time()
    # Initialize
    Lx[0][0][0] = 0
    Lx[0][0][1] = 0
    Lx[1][0][0] = a[1]
    # Lx[0][1][1] = b[1]
    Ly[0][0][0] = 0
    Ly[0][0][1] = 0
    Ly[1][0][0] = c[1]
    # Ly[0][1][1] = b[1]

    for i in range(2, alpha+1):
        Lx[i][0][0] = max(a[i], Lx[i-1][0][0]+W_same)
    for i in range(1, alpha+1):
        Lx[i][0][1] = Lx[i][0][0] + W_diff
    for i in range(2, gamma+1):
        Ly[i][0][0] = max(c[i], Ly[i-1][0][0]+W_same)
    for i in range(1, gamma+1):
        Ly[i][0][1] = Ly[i][0][0] + W_diff

    beta_sum = 1
    beta_1 = 1
    beta_2 = 1
    while beta_sum <= beta:
        if beta_1 == 1:
            Lx[0][beta_1][1] = b[beta_sum]
        else:
            Lx[0][beta_1][1] = max(b[beta_sum], Lx[0][beta_1-1][1]+W_same)
        Lx[0][beta_1][0] = Lx[0][beta_1][1] + W_diff
        if beta_2 == 1:
            Ly[0][beta_2][1] = b[beta_sum]
        else:
            Ly[0][beta_2][1] = max(b[beta_sum], Ly[0][beta_2-1][1]+W_same)
        Ly[0][beta_2][0] = Ly[0][beta_2][1] + W_diff  

        for i in range(1, alpha+1):
            Lx[i][beta_1][0] = min( max(a[i], Lx[i-1][beta_1][0]+W_same), max(a[i], Lx[i-1][beta_1][1]+W_diff) )
            Lx[i][beta_1][1] = min( max(b[beta_sum], Lx[i][beta_1-1][0]+W_diff), max(b[beta_sum], Lx[i][beta_1-1][1]+W_same) )
        for i in range(1, gamma+1):
            Ly[i][beta_2][0] = min( max(c[i], Ly[i-1][beta_2][0]+W_same), max(c[i], Ly[i-1][beta_2][1]+W_diff) )
            Ly[i][beta_2][1] = min( max(b[beta_sum], Ly[i][beta_2-1][0]+W_diff), max(b[beta_sum], Ly[i][beta_2-1][1]+W_same) )

        min_1 = min(Lx[alpha][beta_1][0], Lx[alpha][beta_1][1])
        min_2 = min(Ly[gamma][beta_2][0], Ly[gamma][beta_2][1])
        if min_1 <= min_2:
            beta_1 += 1
        else:
            beta_2 += 1
        beta_sum += 1
    computeTime = time.time() - t0

    beta_1 -= 1
    beta_2 -= 1

    # Choose optimal solution
    order_stack = []
    i = alpha
    j = beta_1
    while i>0 or j>0:
        if Lx[i][j][1] < Lx[i][j][0]:
            order_stack.append(('B', j, Lx[i][j][1]))
            j -= 1
        else:
            order_stack.append(('A', i, Lx[i][j][0]))
            i -= 1
    # Output order
    print('Lane 1:', order_stack)
    # while len(order_stack) > 0:
    #     print(order_stack.pop())
    T_last = order_stack[0][2]

    order_stack = []
    i = gamma
    j = beta_2
    while i>0 or j>0:
        if Ly[i][j][1] < Ly[i][j][0]:
            order_stack.append(('B', j, Ly[i][j][1]))
            j -= 1
        else:
            order_stack.append(('C', i, Ly[i][j][0]))
            i -= 1
    # Output order
    print('Lane 2:', order_stack)
    # while len(order_stack) > 0:
    #     print(order_stack.pop())
    if order_stack[0][2] > T_last:
        T_last = order_stack[0][2]

    return T_last, computeTime
      

# Get the objective value of each solution
def get_obj(sol):
    return max(sol.time)


# Three-dimensional DP 
def oneSol_multiDim_dp(a, b, c, W_same, W_diff):
    alpha = len(a) - 1  # Number of vehicles in lane A
    beta = len(b) - 1   # Number of vehicles in lane B
    gamma = len(c) - 1  # Number of vehicles in lane C
    
    # The three-dimensional DP table,
    # the first character after the underscore means which lane that the last vehicle going to lane X is from, and
    # the second character after the underscore means which lane that the last vehicle going to lane X is from.
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
    L_BB[0][1][0] = Sol((-W_diff, b[1]), 'BB', '0Y') if a[1] <= c[1] else Sol((b[1], -W_diff), 'BB', 'X0')
    if beta >= 2:
        L_BB[0][2][0] = Sol((b[1], b[2]), 'BB', 'XY') if a[1] <= c[1] else Sol((b[2], b[1]), 'BB', 'YX')
    # if beta >= 3:
    #     L_BB[0][3][0] = min(
    #                         Sol((b[1], max(b[3], b[2]+W_same)), 'BB', 'YY'),
    #                         Sol((max(b[3], b[1]+W_same), b[2]), 'BB', 'YX'),
    #                         Sol((max(b[2], b[1]+W_same), b[3]), 'BB', 'XY'),
    #                         key=get_obj
    #     )
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
    if beta >= 3:
        for j in range(3, beta+1):
            L_BB[0][j][0] = min(
                                Sol((max(b[j-1], L_BB[0][j-2][0].time[0]+W_same), max(b[j], L_BB[0][j-2][0].time[1]+W_same)), 'BB', 'XY'),
                                Sol((max(b[j], L_BB[0][j-2][0].time[0]+W_same), max(b[j-1], L_BB[0][j-2][0].time[1]+W_same)), 'BB', 'YX'),
                                Sol((max(b[j], max(b[j-1], L_BB[0][j-2][0].time[0]+W_same)+W_same), L_BB[0][j-2][0].time[1]), 'BB', 'XX'),
                                Sol((L_BB[0][j-2][0].time[0], max(b[j], max(b[j-1], L_BB[0][j-2][0].time[1]+W_same)+W_same)), 'BB', 'YY'),
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
    L_AB[0][1][0] = Sol((-W_diff, b[1]), 'AB', '0Y')
    for j in range(2, beta+1):
        L_AB[0][j][0] = Sol((-W_diff, max(b[j], L_AB[0][j-1][0].time[1]+W_same)), 'AB', '0Y')
    L_AC[1][0][0] = Sol((a[1], -W_diff), 'AC', 'X0')
    for i in range(2, alpha+1):
        L_AC[i][0][0] = Sol((max(a[i], L_AC[i-1][0][0].time[0]+W_same), -W_diff), 'AC', 'X0')
    L_AC[0][0][1] = Sol((-W_diff, c[1]), 'AC', '0Y')
    for k in range(2, gamma+1):
        L_AC[0][0][k] = Sol((-W_diff, max(c[k], L_AC[0][0][k-1].time[1]+W_same)), 'AC', '0Y')
    L_BC[0][1][0] = Sol((b[1], -W_diff), 'BC', 'X0')
    for j in range(2, beta+1):
        L_BC[0][j][0] = Sol((max(b[j], L_BC[0][j-1][0].time[0]+W_same), -W_diff), 'BC', 'X0')
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
    
    # Compute tables
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
                if j>=2:
                    L_BB[i][j][k] = min(
                                        Sol((max(b[j-1], L_AB[i][j-2][k].time[0]+W_diff), max(b[j], L_AB[i][j-2][k].time[1]+W_same)), 'AB', 'XY'),
                                        Sol((max(b[j], L_AB[i][j-2][k].time[0]+W_diff), max(b[j-1], L_AB[i][j-2][k].time[1]+W_same)), 'AB', 'YX'),
                                        Sol((max(b[j], max(b[j-1], L_AB[i][j-2][k].time[0]+W_diff)+W_same), L_AB[i][j-2][k].time[1]), 'AB', 'XX'),
                                        Sol((max(b[j-1], L_AC[i][j-2][k].time[0]+W_diff), max(b[j], L_AC[i][j-2][k].time[1]+W_diff)), 'AC', 'XY'),
                                        Sol((max(b[j], L_AC[i][j-2][k].time[0]+W_diff), max(b[j-1], L_AC[i][j-2][k].time[1]+W_diff)), 'AC', 'YX'),
                                        Sol((max(b[j-1], L_BB[i][j-2][k].time[0]+W_same), max(b[j], L_BB[i][j-2][k].time[1]+W_same)), 'BB', 'XY'),
                                        Sol((max(b[j], L_BB[i][j-2][k].time[0]+W_same), max(b[j-1], L_BB[i][j-2][k].time[1]+W_same)), 'BB', 'YX'),
                                        Sol((max(b[j], max(b[j-1], L_BB[i][j-2][k].time[0]+W_same)+W_same), L_BB[i][j-2][k].time[1]), 'BB', 'XX'),
                                        Sol((L_BB[i][j-2][k].time[0], max(b[j], max(b[j-1], L_BB[i][j-2][k].time[1]+W_same)+W_same)), 'BB', 'YY'),
                                        Sol((max(b[j-1], L_BC[i][j-2][k].time[0]+W_same), max(b[j], L_BC[i][j-2][k].time[1]+W_diff)), 'BC', 'XY'),
                                        Sol((max(b[j], L_BC[i][j-2][k].time[0]+W_same), max(b[j-1], L_BC[i][j-2][k].time[1]+W_diff)), 'BC', 'YX'),
                                        Sol((L_BC[i][j-2][k].time[0], max(b[j], max(b[j-1], L_BC[i][j-2][k].time[1]+W_diff)+W_same)), 'BC', 'YY'),
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
    opt = min(L_AB[i][j][k], L_AC[i][j][k], L_BB[i][j][k], L_BC[i][j][k], key=get_obj)
    table = ''
    lanes = ''
    # print(opt)
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
            if table == 'AB':
                stack_X.append(('B', j, max(b[j], L_AB[i-1][j-1][k].time[0]+W_diff)))
            elif table == 'BB':
                stack_X.append(('B', j, max(b[j], L_BB[i-1][j-1][k].time[0]+W_same)))
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
            stack_X.append(('B', j-1, max(b[j-1], L_BB[i][j-2][k].time[0]+W_same)))
            j -= 2
        elif lanes == 'YY':
            stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
            if table == 'BB':
                stack_Y.append(('B', j-1, max(b[j-1], L_BB[i][j-2][k].time[1]+W_same)))
            elif table == 'BC':
                stack_Y.append(('B', j-1, max(b[j-1], L_BC[i][j-2][k].time[1]+W_same)))
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
                stack_Y.append(('B', j, max(b[j], L_BB[i][j-1][k-1].time[1]+W_same)))
            elif table == 'BC':
                stack_Y.append(('B', j, max(b[j], L_BC[i][j-1][k-1].time[1]+W_diff)))
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
    while i>0 or j>0 or k>0:
        print(table, i, j, k)
        if  table == 'AB':
            # print('AB')
            table = L_AB[i][j][k].table
            lanes = L_AB[i][j][k].lane
            if  lanes == 'XY':
                stack_X.append(('A', i, L_AB[i][j][k].time[0]))
                stack_Y.append(('B', j, L_AB[i][j][k].time[1]))
                i -= 1
                j -= 1
            elif lanes == 'XX':
                stack_X.append(('A', i, L_AB[i][j][k].time[0]))
                if table == 'AB':
                    stack_X.append(('B', j, max(b[j], L_AB[i-1][j-1][k].time[0]+W_diff)))
                elif table == 'BB':
                    stack_X.append(('B', j, max(b[j], L_BB[i-1][j-1][k].time[0]+W_same)))
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
            # print('AC')
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
            # print('BB')
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
                stack_X.append(('B', j-1, max(b[j-1], L_BB[i][j-2][k].time[0]+W_same)))
                j -= 2
            elif lanes == 'YY':
                stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
                if table == 'BB':
                    stack_Y.append(('B', j-1, max(b[j-1], L_BB[i][j-2][k].time[1]+W_same)))
                elif table == 'BC':
                    stack_Y.append(('B', j-1, max(b[j-1], L_BC[i][j-2][k].time[1]+W_same)))
                else:
                    print('bug')
                j -= 2
            elif lanes[0] == 'X':
                stack_X.append(('B', j, L_BB[i][j][k].time[0]))
                j -= 1
            elif lanes[1] == 'Y':
                stack_Y.append(('B', j, L_BB[i][j][k].time[1]))
                j -= 1
        elif  table == 'BC':
            # print('BC')
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
                    stack_Y.append(('B', j, max(b[j], L_BB[i][j-1][k-1].time[1]+W_same)))
                elif table == 'BC':
                    stack_Y.append(('B', j, max(b[j], L_BC[i][j-1][k-1].time[1]+W_diff)))
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

    # Time to finish scheduling
    computeTime = time.time() - t0

    # The time needed for all vehicles to pass the merging point
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
    

# First-Come-First-Go
def fcfg_v1(timeStep, a, b, c, W_same, W_diff):
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
    # a = [0, 3.0, 4.0, 7.0, 10.0, 13.0]
    # b = [0, 1.0, 7.0, 8.0, 12.0, 13.0]
    # c = [0, 7.0, 10.0, 11.0, 14.0, 15.0]
    a = [0, 409, 410]
    b = [0, 462, 463]
    c = [0, 433, 526]
    # print(a)
    # print(b)
    # print(c)

    # Use the previous two-dimentional approach 
    ret = twoDim_dp(copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff)
    print('dp:', ret[0], ret[1])
   
    # Use the three-dimensional approach
    ret = oneSol_multiDim_dp(copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff)
    print('dp:', ret[0], ret[1])

    # Use fcfg approach
    print('fcfg:', fcfg_v1(timeStep, copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff))



if __name__ == '__main__':
    main()