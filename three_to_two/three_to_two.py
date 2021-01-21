import numpy as np
import time
import sys

def generate_traffic(alpha, beta, gamma, pA, pB, pC):
    a = np.array([0])
    b = np.array([0])
    c = np.array([0])
    
    # Randomly generate earliest arrival time
    # pA = 1. / 10 # a vehicle is generated every 10 seconds in average.
    # pB = 1. / 10
    # pC = 1. / 10
    alpha_tmp = alpha
    beta_tmp = beta
    gamma_tmp = gamma
    t = 1
    while alpha_tmp > 0:
        if np.random.uniform(0, 1) < pA:
            a = np.append(a, t)
            alpha_tmp -= 1
        t += 0.1
    t = 1
    while beta_tmp > 0:
        if np.random.uniform(0, 1) < pB:
            b = np.append(b, t)
            beta_tmp -= 1
        t += 0.1
    t = 1
    while gamma_tmp > 0:
        if np.random.uniform(0, 1) < pC:
            c = np.append(c, t)
            gamma_tmp -= 1
        t += 0.1

    return a, b, c

def dp_schedule(a, b, c, W_same, W_diff):
    alpha = len(a) - 1
    beta = len(b) - 1
    gamma = len(c) - 1
    L1 = np.zeros((alpha+1, beta+1, 2))  # dp table
    L2 = np.zeros((gamma+1, beta+1, 2))  # dp table

    t0 = time.time()
    # Initialize
    L1[0][0][0] = 0
    L1[0][0][1] = 0
    L1[1][0][0] = a[1]
    L1[0][1][1] = b[1]
    L2[0][0][0] = 0
    L2[0][0][1] = 0
    L2[1][0][0] = c[1]
    L2[0][1][1] = b[1]
    cnt = 8
    for i in range(2, alpha+1):
        L1[i][0][0] = max(a[i], L1[i-1][0][0]+W_same)
        cnt += 1
    for i in range(1, alpha+1):
        L1[i][0][1] = L1[i][0][0] + W_diff
        cnt += 1
    for i in range(2, gamma+1):
        L2[i][0][0] = max(c[i], L2[i-1][0][0]+W_same)
        cnt += 1
    for i in range(1, gamma+1):
        L2[i][0][1] = L2[i][0][0] + W_diff
        cnt += 1

    beta_sum = 1
    beta_1 = 1
    beta_2 = 1
    while beta_sum <= beta:
        L1[0][beta_1][1] = max(b[beta_sum], L1[0][beta_1-1][1]+W_same)
        L1[0][beta_1][0] = L1[0][beta_1][1] + W_diff
        L2[0][beta_2][1] = max(b[beta_sum], L2[0][beta_2-1][1]+W_same)
        L2[0][beta_2][0] = L2[0][beta_2][1] + W_diff    
        cnt += 4
        for i in range(1, alpha+1):
            L1[i][beta_1][0] = min( max(a[i], L1[i-1][beta_1][0]+W_same), max(a[i], L1[i-1][beta_1][1]+W_diff) )
            L1[i][beta_1][1] = min( max(b[beta_sum], L1[i][beta_1-1][0]+W_diff), max(b[beta_sum], L1[i][beta_1-1][1]+W_same) )
            cnt += 2
        for i in range(1, gamma+1):
            L2[i][beta_2][0] = min( max(c[i], L2[i-1][beta_2][0]+W_same), max(c[i], L2[i-1][beta_2][1]+W_diff) )
            L2[i][beta_2][1] = min( max(b[beta_sum], L2[i][beta_2-1][0]+W_diff), max(b[beta_sum], L2[i][beta_2-1][1]+W_same) )
            cnt += 2

        min_1 = min(L1[alpha][beta_1][0], L1[alpha][beta_1][1])
        min_2 = min(L2[gamma][beta_2][0], L2[gamma][beta_2][1])
        if min_1 <= min_2:
            beta_1 += 1
        else:
            beta_2 += 1
        beta_sum += 1
    computeTime = time.time() - t0

    beta_1 -= 1
    beta_2 -= 1
    # print(cnt)
    # print_table(L1)
    # print_table(L2)

    # Choose optimal solution
    order_stack = []
    i = alpha
    j = beta_1
    while i>0 or j>0:
        if L1[i][j][1] < L1[i][j][0]:
            order_stack.append(('B', j, L1[i][j][1]))
            j -= 1
        else:
            order_stack.append(('A', i, L1[i][j][0]))
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
        if L2[i][j][1] < L2[i][j][0]:
            order_stack.append(('B', j, L2[i][j][1]))
            j -= 1
        else:
            order_stack.append(('C', i, L2[i][j][0]))
            i -= 1
    # Output order
    # print('Lane 2:', order_stack[0])
    # while len(order_stack) > 0:
    #     print(order_stack.pop())
    if order_stack[0][2] < T_last:
        T_last = order_stack[0][2]

    return T_last, computeTime

def print_table(L):
    for k in range(L.shape[2]):
        for i in range(L.shape[0]):
            for j in range(L.shape[1]):
                print(L[i][j][k], end=' ')
            print('')
        print('')

def fafg_schedule(a, b, c, W_same, W_diff):
    a = a[1:]
    b = b[1:]
    c = c[1:]
    alpha = len(a)
    beta = len(b)
    gamma = len(c)
    t = 0   # time
    nextA = 1
    nextB1 = 1
    nextB2 = 1
    nextC = 1
    while len(a) > 0 or len(b) > 0 or len(c) > 0:
        t += 0.1
        if len(a) > 0 and len(b) > 0 and len(c) > 0:
            # print('a', a[0], 'b', b[0], 'c', c[0])
            if (a[0] <= t and nextA <= t) and ((b[0] <= t and nextB1 <= t) or (b[0] <= t and nextB2 <= t)) and (c[0] <= t and nextC <= t):
                if a[0] <= b[0]:
                    # print(a[0], t)
                    a = a[1:]
                    nextA = t + W_same
                    nextB1 = t + W_diff
                elif b[0] <= t and nextB1 <= t:
                    # print(b[0], t)
                    b = b[1:]
                    nextB1 = t + W_same
                    nextA = t + W_diff
                try:
                    if c[0] <= b[0]:
                        # print(c[0], t)
                        c = c[1:]
                        nextC = t + W_same
                        nextB2 = t + W_diff
                    elif b[0] <= t and nextB2 <= t:
                        # print(b[0], t)
                        b = b[1:]
                        nextB2 = t + W_same
                        nextC = t + W_diff
                except Exception as e:
                    # print(e)
                    # print(c[0], t)
                    c = c[1:]
                    nextC = t + W_same
                    nextB2 = t + W_diff
            elif (a[0] <= t and nextA <= t) and (b[0] <= t and nextB2 <= t):
                if a[0] <= b[0]:
                    # print(a[0], t)
                    a = a[1:]
                    nextA = t + W_same
                    nextB1 = t + W_diff
                if b[0] <= c[0]:
                    # print(b[0], t)
                    b = b[1:]
                    nextB2 = t + W_same
                    nextC = t + W_diff
            elif (a[0] <= t and nextA <= t) and (c[0] <= t and nextC <= t):
                if a[0] <= b[0]:
                    # print(a[0], t)
                    a = a[1:]
                    nextA = t + W_same
                    nextB1 = t + W_diff
                if c[0] <= b[0]:
                    # print(c[0], t)
                    c = c[1:]
                    nextC = t + W_same
                    nextB2 = t + W_diff
            elif (b[0] <= t and nextB1 <= t) and (c[0] <= t and nextC <= t):
                if b[0] <= a[0]:
                    # print(b[0], t)
                    b = b[1:]
                    nextA = t + W_diff
                    nextB1 = t + W_same
                    # print(c[0], t)
                    c = c[1:]
                    nextC = t + W_same
                    nextB2 = t + W_diff
            elif a[0] <= t and nextA <= t:
                if a[0] < b[0]:
                    # print(a[0], t)
                    a = a[1:]
                    nextA = t + W_same
                    nextB1 = t + W_diff
            elif c[0] <= t and nextC <= t:
                if c[0] <= b[0]:
                    # print(c[0], t)
                    c = c[1:]
                    nextC = t + W_same
                    nextB2 = t + W_diff
            elif b[0] <= t and nextB1 <= t:
                if b[0] <= a[0]:
                    # print(b[0], t)
                    b = b[1:]
                    nextB1 = t + W_same
                    nextA = t + W_diff
            elif b[0] <= t and nextB2 <= t:
                if b[0] <= c[0]:
                    # print(b[0], t)
                    b = b[1:]
                    nextB2 = t + W_same
                    nextC = t + W_diff
        elif len(a) > 0 and len(b) > 0:
            # print('a', a[0], 'b', b[0])
            if (a[0] <= t and nextA <= t) and (b[0] <= t and nextB2 <= t):
                if a[0] <= b[0]:
                    # print(a[0], t)
                    a = a[1:]
                    nextA = t + W_same
                    nextB1 = t + W_diff
                # print(b[0], t)
                b = b[1:]
                nextB2 = t + W_same
            elif a[0] <= t and nextA <= t:
                if a[0] <= b[0]:
                    # print(a[0], t)
                    a = a[1:]
                    nextA = t + W_same
                    nextB1 = t + W_diff
            elif b[0] <= t and nextB1 <= t:
                if b[0] <= a[0]:
                    # print(b[0], t)
                    b = b[1:]
                    nextB1 = t + W_same
                    nextA = t + W_diff
            elif b[0] <= t and nextB2 <= t:
                # print(b[0], t)
                b = b[1:]
                nextB2 = t + W_same
        elif len(c) > 0 and len(b) > 0:
            # print('b', b[0], 'c', c[0])
            if (b[0] <= t and nextB1 <= t) and (c[0] <= t and nextC <= t):
                # print(b[0], t)
                b = b[1:]
                nextB1 = t + W_same
                # print(c[0], t)
                c = c[1:]
                nextC = t + W_same
                nextB2 = t + W_diff
            elif c[0] <= t and nextC <= t:
                if c[0] <= b[0]:
                    # print(c[0], t)
                    c = c[1:]
                    nextC = t + W_same
                    nextB2 = t + W_diff
            elif b[0] <= t and nextB1 <= t:
                # print(b[0], t)
                b = b[1:]
                nextB1 = t + W_same
            elif b[0] <= t and nextB2 <= t:
                if b[0] < c[0]:
                    # print(b[0], t)
                    b = b[1:]
                    nextB2 = t + W_same
                    nextC = t + W_diff
        elif len(a) > 0 and len(c) > 0:
            # print('a', a[0], 'c', c[0])
            if (a[0] <= t and nextA <= t) and (c[0] <= t and nextC <= t):
                # print(a[0], t)
                a = a[1:]
                nextA = t + W_same
                nextB1 = t + W_diff
                # print(c[0], t)
                c = c[1:]
                nextC = t + W_same
                nextB2 = t + W_diff
            elif a[0] <= t and nextA <= t:
                # print(a[0], t)
                a = a[1:]
                nextA = t + W_same
                nextB1 = t + W_diff
            elif c[0] <= t and nextC <= t:
                # print(c[0], t)
                c = c[1:]
                nextC = t + W_same
                nextB2 = t + W_diff
        elif len(a) > 0:
            # print('a', a[0])
            if a[0] <= t and nextA <= t:
                # print(a[0], t)
                a = a[1:]
                nextA = t + W_same
        elif len(b) > 0:
            # print('b', b[0])
            if b[0] <= t and nextB1 <= t:
                # print(b[0], t)
                b = b[1:]
                nextB1 = t + W_same
            elif b[0] <= t and nextB2 <= t:
                # print(b[0], t)
                b = b[1:]
                nextB2 = t + W_same
        elif len(c) > 0:
            # print('c', c[0])
            if c[0] <= t and nextC <= t:
                # print(c[0], t)
                c = c[1:]
                nextC = t + W_same
    
    # print(t)
    return t
    
# arguments: lambda, N, W=, W+
def main():
    W_same = float(sys.argv[3]) # the waiting time if two consecutive vehicles are from the same lane
    W_diff = float(sys.argv[4])  # the waiting time if two consecutive vehicles are from different lanes
    # alpha = np.random.randint(1,11)
    # beta = np.random.randint(1,11)
    # gamma = np.random.randint(1,11)
    alpha = int(sys.argv[2])
    beta = int(sys.argv[2])
    gamma = int(sys.argv[2])
    # print(alpha)
    # print(beta)
    # print(gamma)
    pA = float(sys.argv[1])
    pB = float(sys.argv[1])
    pC = float(sys.argv[1])
    a, b, c = generate_traffic(alpha, beta, gamma, pA, pB, pC)
    
    # print(a)
    # print(b)
    # print(c)

    ret = dp_schedule(a, b, c, W_same, W_diff)
    print('dp:', ret[0], ret[1])
    print('fcfg:', fafg_schedule(a, b, c, W_same, W_diff))


if __name__ == '__main__':
    main()