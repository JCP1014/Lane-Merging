import numpy as np
import time
import sys
import copy

def generate_traffic(alpha, beta, gamma, pA, pB, pC):
    a = [0]
    b = [0]
    c = [0]
    
    # Randomly generate earliest arrival time
    # pA = 1. / 10 # a vehicle is generated every 10 seconds in average.
    # pB = 1. / 10
    # pC = 1. / 10
    alpha_tmp = alpha
    beta_tmp = beta
    gamma_tmp = gamma
    t = 1.0
    while alpha_tmp > 0:
        if np.random.uniform(0, 1) < pA:
            t = round(t, 1)
            a.append(t)
            alpha_tmp -= 1
        t += 0.1
    t = 1.0
    while beta_tmp > 0:
        if np.random.uniform(0, 1) < pB:
            t = round(t, 1)
            b.append(t)
            beta_tmp -= 1
        t += 0.1
    t = 1.0
    while gamma_tmp > 0:
        if np.random.uniform(0, 1) < pC:
            t = round(t, 1)
            c.append(t)
            gamma_tmp -= 1
        t += 0.1

    return a, b, c


def print_table(L):
    for k in range(L.shape[2]):
        for i in range(L.shape[0]):
            for j in range(L.shape[1]):
                print(L[i][j][k], end=' ')
            print('')
        print('')


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
        Lx[0][beta_1][1] = max(b[beta_sum], Lx[0][beta_1-1][1]+W_same)
        Lx[0][beta_1][0] = Lx[0][beta_1][1] + W_diff
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
    # print('Lane 1:', order_stack[0])
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
    # print('Lane 2:', order_stack[0])
    # while len(order_stack) > 0:
    #     print(order_stack.pop())
    if order_stack[0][2] < T_last:
        T_last = order_stack[0][2]

    return T_last, computeTime


# def multiDim_dp(a, b, c, W_same, W_diff):
#     alpha = len(a) - 1
#     beta = len(b) - 1
#     gamma = len(c) - 1
#     L_AB = np.zeros((alpha+1, beta+1, gamma+1))
#     L_AC = np.zeros((alpha+1, beta+1, gamma+1))
#     L_BB = np.zeros((alpha+1, beta+1, gamma+1))
#     L_BC = np.zeros((alpha+1, beta+1, gamma+1))

#     t0 = time.time()
#     # Initialize
#     L_AB[0][0][0] = (0, 0)
#     L_AC[0][0][0] = (0, 0)
#     L_BB[0][0][0] = (0, 0)
#     L_BC[0][0][0] = (0, 0)

#     L_AB[1][1][0] = (a[1], b[1])
#     L_AC[1][0][1] = (a[1], c[1])
#     L_BC[0][1][1] = (b[1], c[1])

#     for i in range(2, alpha+1):
#         for j in range(2, beta+1):
#             L_AB[i][j][0] = ( max(a[i], L_AB[i-1][j][0]+W_same), max(b[i], L_AB[i][j][0]+W_same)
#     for i in range(1, alpha+1):
#         L1[i][0][1] = L1[i][0][0] + W_diff
#     for i in range(2, gamma+1):
#         L2[i][0][0] = max(c[i], L2[i-1][0][0]+W_same)
#     for i in range(1, gamma+1):
#         L2[i][0][1] = L2[i][0][0] + W_diff

#     beta_sum = 1
#     beta_1 = 1
#     beta_2 = 1
#     while beta_sum <= beta:
#         L1[0][beta_1][1] = max(b[beta_sum], L1[0][beta_1-1][1]+W_same)
#         L1[0][beta_1][0] = L1[0][beta_1][1] + W_diff
#         L2[0][beta_2][1] = max(b[beta_sum], L2[0][beta_2-1][1]+W_same)
#         L2[0][beta_2][0] = L2[0][beta_2][1] + W_diff    
#         for i in range(1, alpha+1):
#             L1[i][beta_1][0] = min( max(a[i], L1[i-1][beta_1][0]+W_same), max(a[i], L1[i-1][beta_1][1]+W_diff) )
#             L1[i][beta_1][1] = min( max(b[beta_sum], L1[i][beta_1-1][0]+W_diff), max(b[beta_sum], L1[i][beta_1-1][1]+W_same) )
#         for i in range(1, gamma+1):
#             L2[i][beta_2][0] = min( max(c[i], L2[i-1][beta_2][0]+W_same), max(c[i], L2[i-1][beta_2][1]+W_diff) )
#             L2[i][beta_2][1] = min( max(b[beta_sum], L2[i][beta_2-1][0]+W_diff), max(b[beta_sum], L2[i][beta_2-1][1]+W_same) )

#         min_1 = min(L1[alpha][beta_1][0], L1[alpha][beta_1][1])
#         min_2 = min(L2[gamma][beta_2][0], L2[gamma][beta_2][1])
#         if min_1 <= min_2:
#             beta_1 += 1
#         else:
#             beta_2 += 1
#         beta_sum += 1
#     computeTime = time.time() - t0

#     beta_1 -= 1
#     beta_2 -= 1
#     # print_table(L1)
#     # print_table(L2)

#     # Choose optimal solution
#     order_stack = []
#     i = alpha
#     j = beta_1
#     while i>0 or j>0:
#         if L1[i][j][1] < L1[i][j][0]:
#             order_stack.append(('B', j, L1[i][j][1]))
#             j -= 1
#         else:
#             order_stack.append(('A', i, L1[i][j][0]))
#             i -= 1
#     # Output order
#     # print('Lane 1:', order_stack[0])
#     # while len(order_stack) > 0:
#     #     print(order_stack.pop())
#     T_last = order_stack[0][2]

#     order_stack = []
#     i = gamma
#     j = beta_2
#     while i>0 or j>0:
#         if L2[i][j][1] < L2[i][j][0]:
#             order_stack.append(('B', j, L2[i][j][1]))
#             j -= 1
#         else:
#             order_stack.append(('C', i, L2[i][j][0]))
#             i -= 1
#     # Output order
#     # print('Lane 2:', order_stack[0])
#     # while len(order_stack) > 0:
#     #     print(order_stack.pop())
#     if order_stack[0][2] < T_last:
#         T_last = order_stack[0][2]

#     return T_last, computeTime


def fcfg(a, b, c, W_same, W_diff):  # v2
    a = a[1:]
    b = b[1:]
    c = c[1:]
    t = 0   # time
    qX = [] # Vehicles go to lane X
    qY = [] # Vehicles go to lane Y
    prevTo = 'Y'

    while len(a) > 0 or len(b) > 0 or len(c) > 0:
        t += 0.1
        if len(a) > 0 and len(b) > 0 and len(c) > 0:
            if a[0] <= t and b[0] <= t and c[0] <= t:
                if b[1] >= a[1] and b[1] >= c[1]:
                    qX.append( ('A', a.pop(0)) )
                    qY.append( ('C', c.pop(0)) )
                elif c[1] >= a[1] and c[1] >= b[1]:
                    qX.append( ('A', a.pop(0)) )
                    qY.append( ('B', b.pop(0)) )
                elif a[1] >= b[1] and a[1] >= c[1]:
                    qX.append( ('B', b.pop(0)) )
                    qY.append( ('C', c.pop(0)) )
            elif a[0] <= t and c[0] <= t:
                qX.append( ('A', a.pop(0)) )
                qY.append( ('C', c.pop(0)) )
            elif a[0] <= t and b[0] <= t:
                qX.append( ('A', a.pop(0)) )
                qY.append( ('B', b.pop(0)) )
            elif b[0] <= t and c[0] <= t:
                qX.append( ('B', b.pop(0)) )
                qY.append( ('C', c.pop(0)) )
            elif b[0] <= t:
                if c[0] > a[0]:
                    qY.append( ('B', b.pop(0)) )
                else:
                    qX.append( ('B', b.pop(0)) )  
            elif a[0] <= t:
                qX.append( ('A', a.pop(0)) )
            elif c[0] <= t:
                qY.append( ('C', c.pop(0)) )
        elif len(a) > 0 and len(c) > 0: 
            if a[0] <= t and c[0] <= t:
                qX.append( ('A', a.pop(0)) )
                qY.append( ('C', c.pop(0)) )
            elif a[0] <= t:
                qX.append( ('A', a.pop(0)) )
            elif c[0] <= t:
                qY.append( ('C', c.pop(0)) )
        elif len(a) > 0 and len(b) > 0:
            if a[0] <= t and b[0] <= t:
                qX.append( ('A', a.pop(0)) )
                qY.append( ('B', b.pop(0)) )
            elif a[0] <= t:
                qX.append( ('A', a.pop(0)) )
            elif b[0] <= t:
                qY.append( ('B', b.pop(0)) )
        elif len(b) > 0 and len(c) > 0:
            if b[0] <= t and c[0] <= t:
                qX.append( ('B', b.pop(0)) )
                qY.append( ('C', c.pop(0)) )
            elif c[0] <= t:
                qY.append( ('C', c.pop(0)) )
            elif b[0] <= t:
                qX.append( ('B', b.pop(0)) )  
        elif len(a) > 0:
            qX.append( ('A', a.pop(0)) )
        elif len(c) > 0:
            qY.append( ('C', c.pop(0)) )
        elif len(b) > 0:
            if prevTo == 'Y':
                qX.append( ('B', b.pop(0)) )
                prevTo = 'X'
            else:
                qY.append( ('B', b.pop(0)) )
                prevTo = 'Y'
    
    (prevFrom, T_lastX) = qX.pop(0)
    for (lane, arrival) in qX:
        if lane == prevFrom:
            if arrival - T_lastX < W_same:
                T_lastX = T_lastX + W_same
            else:
                T_lastX = arrival
        else:
            if arrival - T_lastX < W_diff:
                T_lastX = T_lastX + W_diff
            else:
                T_lastX = arrival
    
    (prevFrom, T_lastY) = qY.pop(0)
    for (lane, arrival) in qY:
        if lane == prevFrom:
            if arrival - T_lastY < W_same:
                T_lastY = T_lastY + W_same
            else:
                T_lastY = arrival
        else:
            if arrival - T_lastY < W_diff:
                T_lastY = T_lastY + W_diff
            else:
                T_lastY = arrival

    return T_lastX if T_lastX >= T_lastY else T_lastY


# command line arguments: lambda, N, W=, W+
def main():
    W_same = float(sys.argv[3]) # the waiting time if two consecutive vehicles are from the same lane
    W_diff = float(sys.argv[4])  # the waiting time if two consecutive vehicles are from different lanes
    alpha = int(sys.argv[2])
    beta = int(sys.argv[2])
    gamma = int(sys.argv[2])
    pA = float(sys.argv[1])
    pB = float(sys.argv[1])
    pC = float(sys.argv[1])
    a, b, c = generate_traffic(alpha, beta, gamma, pA, pB, pC)
    # print(a)
    # print(b)
    # print(c)

    ret = twoDim_dp(copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff)
    print('dp:', ret[0], ret[1])
    print('fcfg:', fcfg(copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff))



if __name__ == '__main__':
    main()