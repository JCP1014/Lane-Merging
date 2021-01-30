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
            a.append(round(t, 1))
            alpha_tmp -= 1
        t += 0.1
    t = 1.0
    while beta_tmp > 0:
        if np.random.uniform(0, 1) < pB:
            b.append(round(t, 1))
            beta_tmp -= 1
        t += 0.1
    t = 1.0
    while gamma_tmp > 0:
        if np.random.uniform(0, 1) < pC:
            c.append(round(t, 1))
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
        

def multiDim_dp(a, b, c, W_same, W_diff):
    alpha = len(a) - 1
    beta = len(b) - 1
    gamma = len(c) - 1
    L_AB = np.zeros((alpha+1, beta+1, gamma+1))
    L_AC = np.zeros((alpha+1, beta+1, gamma+1))
    L_BB = np.zeros((alpha+1, beta+1, gamma+1))
    L_BC = np.zeros((alpha+1, beta+1, gamma+1))

    t0 = time.time()
    # Initialize
    L_AB[0][0][0] = (0, 0)
    L_AC[0][0][0] = (0, 0)
    L_BB[0][0][0] = (0, 0)
    L_BC[0][0][0] = (0, 0)

    L_AB[1][1][0] = (a[1], b[1])
    L_AC[1][0][1] = (a[1], c[1])
    L_BC[0][1][1] = (b[1], c[1])
    if a[1] <= c[1]:
        L_BB[0][2][0] = (b[1], b[2])
    else:
        L_BB[0][2][0] = (b[2], b[1])

    for i in range(2, alpha+1):
        L_AB[i][1][0] = ( max(a[i], L_AB[i-1][1][0]+W_same), b[1] )
    for j in range(2, beta+1):
        L_AB[1][j][0] = ( a[1], max(b[j], L_AB[1][j-1][0]+W_same) )
    for i in range(2, alpha+1):
        L_AC[i][0][1] = ( max(a[i], L_AC[i-1][0][1]+W_same), c[1] )
    for k in range(2, gamma+1):
        L_AC[1][0][k] = ( a[1], max(c[k], L_AC[1][0][k-1]+W_same) )
    for j in range(2, beta+1):
        L_BC[0][j][1] = ( max(b[j], L_BC[0][j-1][1]+W_same), c[1] )
    for k in range(2, gamma+1):
        L_BC[0][1][k] = ( b[1], max(c[k], L_BC[0][1][k-1]+W_same) )

    
    for i in range(1, alpha+1):
        for j in range(1, beta+1):
            for k in range(1, gamma+1):
                L_AB[i][j][k] = min(
                                    (max(a[i], L_AB[i-1][j-1][k][0]+W_same), max(b[j], L_AB[i-1][j-1][k][1]+W_same)),
                                    (max(a[i], max(b[j], L_AB[i-1][j-1][k][0]+W_diff)+W_diff), L_AB[i-1][j-1][k][1]),
                                    (max(a[i], L_AC[i-1][j-1][k][0]+W_same), max(b[j], L_AC[i-1][j-1][k][1]+W_diff)),
                                    (max(a[i], L_BB[i-1][j-1][k][0]+W_diff), max(b[j], L_BB[i-1][j-1][k][1]+W_same)),
                                    (max(a[i], max(b[j], L_BB[i-1][j-1][k][0]+W_same)+W_diff), L_BB[i-1][j-1][k][1]),
                                    (max(a[i], L_BC[i-1][j-1][k][0]+W_diff), max(b[j], L_BC[i-1][j-1][k][1]+W_diff)),
                                    key=max
                )
                L_AC[i][j][k] = min(
                                    (max(a[i], L_AB[i-1][j][k-1][0]+W_same), max(c[k], L_AB[i-1][j][k-1][1]+W_diff)),
                                    (max(a[i], L_AC[i-1][j][k-1][0]+W_same), max(c[k], L_AC[i-1][j][k-1][1]+W_same)),
                                    (max(a[i], L_BB[i-1][j][k-1][0]+W_diff), max(c[k], L_BB[i-1][j][k-1][1]+W_diff)),
                                    (max(a[i], L_BC[i-1][j][k-1][0]+W_diff), max(c[k], L_BC[i-1][j][k-1][1]+W_same)),
                                    key=max
                )
                L_BC[i][j][k] = min(
                                    (max(b[j], L_AB[i][j-1][k-1][0]+W_diff), max(c[k], L_AB[i][j-1][k-1][1]+W_diff)),
                                    (max(b[j], L_AC[i][j-1][k-1][0]+W_diff), max(c[k], L_AC[i][j-1][k-1][1]+W_same)),
                                    (max(b[j], L_BB[i][j-1][k-1][0]+W_same), max(c[k], L_BB[i][j-1][k-1][1]+W_diff)),
                                    (L_BB[i][j-1][k-1][0], max(c[k], max(b[j], L_BB[i][j-1][k-1][1]+W_same)+W_diff)),
                                    (max(b[j], L_BC[i][j-1][k-1][0]+W_same), max(c[k], L_BC[i][j-1][k-1][1]+W_same)),
                                    (L_BC[i][j-1][k-1][0], max(c[k], max(b[j], L_BC[i][j-1][k-1][1]+W_diff)+W_diff)),
                                    key=max
                )
                L_BB[i][j][k] = min(
                                    (max(b[j-1], L_AB[i][j-2][k][0]+W_diff), max(b[j], L_AB[i][j-2][k][1]+W_same)),
                                    (max(b[j], L_AB[i][j-2][k][0]+W_diff), max(b[j-1], L_AB[i][j-2][k][1]+W_same)),
                                    (max(b[j-1], L_AC[i][j-2][k][0]+W_diff), max(b[j], L_AC[i][j-2][k][1]+W_diff)),
                                    (max(b[j], L_AC[i][j-2][k][0]+W_diff), max(b[j-1], L_AC[i][j-2][k][1]+W_diff)),
                                    (max(b[j-1], L_BB[i][j-2][k][0]+W_same), max(b[j], L_BB[i][j-2][k][1]+W_same)),
                                    (max(b[j], L_BB[i][j-2][k][0]+W_same), max(b[j-1], L_BB[i][j-2][k][1]+W_same)),
                                    (max(b[j], max(b[j-1], L_BB[i][j-2][k][0]+W_same)+W_same), L_BB[i][j-2][k][1]),
                                    (L_BB[i][j-2][k][0], max(b[j], max(b[j-1], L_BB[i][j-2][k][1]+W_same)+W_same)),
                                    (max(b[j-1], L_BC[i][j-2][k][0]+W_same), max(b[j], L_BC[i][j-2][k][1]+W_diff)),
                                    (max(b[j], L_BC[i][j-2][k][0]+W_same), max(b[j-1], L_BC[i][j-2][k][1]+W_diff)),
                                    (L_BC[i][j-2][k][0], max(b[j], max(b[j-1], L_BC[i][j-2][k][1]+W_same)+W_diff)),
                                    key=max
                )


def fcfg(a, b, c, W_same, W_diff):  # v4
    a = a[1:]
    b = b[1:]
    c = c[1:]
    t = 0   # time
    B_prevTo = 'Y'
    X_lastT = 0
    Y_lastT = 0
    X_lastFrom = ''
    Y_lastFrom = ''

    # Decide first vehicles of lane X and lane Y
    # to initialize X_lastT, Y_lastT, X_lastFrom, and Y_lastFrom
    minT = min(a[0], b[0], c[0])
    if a[0] == minT and b[0] == minT and c[0] == minT:
        if max(a[1], b[1], c[1]) == b[1]:
            X_lastT = a.pop(0)
            X_lastFrom = 'A'
            Y_lastT = c.pop(0)
            Y_lastFrom = 'C'
        elif c[1] >= a[1]:
            X_lastT = a.pop(0)
            X_lastFrom = 'A'
            Y_lastT = b.pop(0)
            Y_lastFrom = 'B'
        else:
            X_lastT = b.pop(0)
            X_lastFrom = 'B'
            Y_lastT = c.pop(0)
            Y_lastFrom = 'C'
    elif a[0] == minT and c[0] == minT:
        X_lastT = a.pop(0)
        X_lastFrom = 'A'
        Y_lastT = c.pop(0)
        Y_lastFrom = 'C'
    elif a[0] == minT and b[0] == minT:
        X_lastT = a.pop(0)
        X_lastFrom = 'A'
        Y_lastT = b.pop(0)
        Y_lastFrom = 'B'
    elif b[0] == minT and c[0] == minT:
        X_lastT = b.pop(0)
        X_lastFrom = 'B'
        Y_lastT = c.pop(0)
        Y_lastFrom = 'C'
    elif a[0] == minT:
        X_lastT = a.pop(0)
        X_lastFrom = 'A'
        if b[0] == c[0]:
            if b[1] >= c[1]:
                Y_lastT = c.pop(0)
                Y_lastFrom = 'C'
            else:
                Y_lastT = b.pop(0)
                Y_lastFrom = 'B'
        elif c[0] < b[0]:
            Y_lastT = c.pop(0)
            Y_lastFrom = 'C'
        else:
            Y_lastT = b.pop(0)
            Y_lastFrom = 'B'
    elif c[0] == minT:
        Y_lastT = c.pop(0)
        Y_lastFrom = 'C'
        if b[0] == a[0]:
            if b[1] >= a[1]:
                X_lastT = a.pop(0)
                X_lastFrom = 'A'
            else:
                X_lastT = b.pop(0)
                X_lastFrom = 'B'
        elif a[0] < b[0]:
            X_lastT = a.pop(0)
            X_lastFrom = 'A'
        else:
            X_lastT = b.pop(0)
            X_lastFrom = 'B'
    elif b[0] == minT:
        if a[0] >= c[0]:
            X_lastT = b.pop(0)
            X_lastFrom = 'B'
            if b[0] == c[0]:
                if b[1] >= c[1]:
                    Y_lastT = c.pop(0)
                    Y_lastFrom = 'C'
                else:
                    Y_lastT = b.pop(0)
                    Y_lastFrom = 'B'
            elif c[0] < b[0]:
                Y_lastT = c.pop(0)
                Y_lastFrom = 'C'
            else:
                Y_lastT = b.pop(0)
                Y_lastFrom = 'B'
        else:
            Y_lastT = b.pop(0)
            Y_lastFrom = 'B'
            if b[0] == a[0]:
                if b[1] >= a[1]:
                    X_lastT = a.pop(0)
                    X_lastFrom = 'A'
                else:
                    X_lastT = b.pop(0)
                    X_lastFrom = 'B'
            elif a[0] < b[0]:
                X_lastT = a.pop(0)
                X_lastFrom = 'A'
            else:
                X_lastT = b.pop(0)
                X_lastFrom = 'B'
    print('X:', X_lastFrom, X_lastT)
    print('Y:', Y_lastFrom, Y_lastT)

        
    while len(a) > 0 or len(b) > 0 or len(c) > 0:
        t = round(t+0.1, 2)
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
                        print('Y:', Y_lastFrom, Y_lastT)   
                    else:
                        if X_lastFrom == 'B':
                            X_lastT = max(b.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'B'
                        print('X:', X_lastFrom, X_lastT)
                else:
                    if X_lastT <= t and Y_lastT <= t:
                        if c[0] > a[0]:
                            if Y_lastFrom == 'B':
                                Y_lastT = max(b.pop(0), Y_lastT+W_same)
                            else:
                                Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                            Y_lastFrom = 'B'
                            print('Y:', Y_lastFrom, Y_lastT)   
                        else:
                            if X_lastFrom == 'B':
                                X_lastT = max(b.pop(0), X_lastT+W_same)
                            else:
                                X_lastT = max(b.pop(0), X_lastT+W_diff)
                            X_lastFrom = 'B'
                            print('X:', X_lastFrom, X_lastT)
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
                                print('Y:', Y_lastFrom, Y_lastT)   
                            else:
                                if X_lastFrom == 'B':
                                    X_lastT = max(b.pop(0), X_lastT+W_same)
                                else:
                                    X_lastT = max(b.pop(0), X_lastT+W_diff)
                                X_lastFrom = 'B'
                                print('X:', X_lastFrom, X_lastT)
            elif a[0] == t:
                if X_lastFrom == 'A':
                    X_lastT = max(a.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(a.pop(0), X_lastT+W_diff)
                X_lastFrom = 'A'
                print('X:', X_lastFrom, X_lastT)
            elif c[0] == t:
                if Y_lastFrom == 'C':
                    Y_lastT = max(c.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'C'
                print('Y:', Y_lastFrom, Y_lastT)  
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
                print('X:', X_lastFrom, X_lastT)  
            elif c[0] == t:
                if Y_lastFrom == 'C':
                    Y_lastT = max(c.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(c.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'C' 
                print('Y:', Y_lastFrom, Y_lastT)
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
                print('X:', X_lastFrom, X_lastT)
            elif b[0] == t:
                if X_lastT <= t and Y_lastT <= t:
                    if Y_lastFrom == 'B':
                        Y_lastT = max(b.pop(0), Y_lastT+W_same)
                    else:
                        Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                    Y_lastFrom = 'B'
                    print('Y:', Y_lastFrom, Y_lastT)
                else:
                    if X_lastT < Y_lastT:
                        if X_lastFrom == 'B':
                            X_lastT = max(b.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'B'
                        print('X:', X_lastFrom, X_lastT)
                    elif Y_lastT < X_lastT:
                        if Y_lastFrom == 'B':
                            Y_lastT = max(b.pop(0), Y_lastT+W_same)
                        else:
                            Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'B'
                        print('Y:', Y_lastFrom, Y_lastT)
                    else:
                        if Y_lastFrom == 'B':
                            Y_lastT = max(b.pop(0), Y_lastT+W_same)
                        else:
                            Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'B'
                        print('Y:', Y_lastFrom, Y_lastT)
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
                print('Y:', Y_lastFrom, Y_lastT)
            elif b[0] == t:
                if X_lastT <= t and Y_lastT <= t:
                    if X_lastFrom == 'B':
                        X_lastT = max(b.pop(0), X_lastT+W_same)
                    else:
                        X_lastT = max(b.pop(0), X_lastT+W_diff)
                    X_lastFrom = 'B'
                    print('X:', X_lastFrom, X_lastT)
                else:
                    if X_lastT < Y_lastT:
                        if X_lastFrom == 'B':
                            X_lastT = max(b.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'B'
                        print('X:', X_lastFrom, X_lastT)
                    elif Y_lastT < X_lastT:
                        if Y_lastFrom == 'B':
                            Y_lastT = max(b.pop(0), Y_lastT+W_same)
                        else:
                            Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                        Y_lastFrom = 'B'
                        print('Y:', Y_lastFrom, Y_lastT)
                    else:
                        if X_lastFrom == 'B':
                            X_lastT = max(b.pop(0), X_lastT+W_same)
                        else:
                            X_lastT = max(b.pop(0), X_lastT+W_diff)
                        X_lastFrom = 'B'
                        print('X:', X_lastFrom, X_lastT)
        elif len(a) > 0:
            if X_lastFrom == 'A':
                X_lastT = max(a.pop(0), X_lastT+W_same)
            else:
                X_lastT = max(a.pop(0), X_lastT+W_diff)
            X_lastFrom = 'A'
            print('X:', X_lastFrom, X_lastT)
        elif len(c) > 0:
            if Y_lastFrom == 'C':
                Y_lastT = max(c.pop(0), Y_lastT+W_same)
            else:
                Y_lastT = max(c.pop(0), Y_lastT+W_diff)
            Y_lastFrom = 'C'
            print('Y:', Y_lastFrom, Y_lastT)
        elif len(b) > 0:
            if B_prevTo == 'Y':
                if X_lastFrom == 'B':
                    X_lastT = max(b.pop(0), X_lastT+W_same)
                else:
                    X_lastT = max(b.pop(0), X_lastT+W_diff)
                X_lastFrom = 'B'
                print('X:', X_lastFrom, X_lastT)
                B_prevTo = 'X'
            else:
                if Y_lastFrom == 'B':
                    Y_lastT = max(b.pop(0), Y_lastT+W_same)
                else:
                    Y_lastT = max(b.pop(0), Y_lastT+W_diff)
                Y_lastFrom = 'B'
                print('Y:', Y_lastFrom, Y_lastT)
                B_prevTo = 'Y'

    return max(X_lastT, Y_lastT)


# Command line arguments: lambda, N, W=, W+
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
    a = [0, 7.9, 10.3, 12.4]
    b = [0, 4.2, 7.1, 11.8]
    c = [0, 7.8, 9.5, 11.0]
    print(a)
    print(b)
    print(c)

    ret = twoDim_dp(copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff)
    print('dp:', ret[0], ret[1])
    print('fcfg:', fcfg(copy.deepcopy(a), copy.deepcopy(b), copy.deepcopy(c), W_same, W_diff))



if __name__ == '__main__':
    main()