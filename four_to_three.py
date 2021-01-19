import numpy as np
import time

def print_table(L):
    for k in range(L.shape[2]):
        for i in range(L.shape[0]):
            for j in range(L.shape[1]):
                print(L[i][j][k], end=' ')
            print('')
        print('')

def schedule(alpha, beta, gamma, a, b, c):
    W_same = 1  # the waiting time if two consecutive vehicles are from the same lane
    W_diff = 3  # the waiting time if two consecutive vehicles are from different lanes
    L1 = np.zeros((alpha+1, beta+1, 2))  # dp table
    L2 = np.zeros((gamma+1, beta+1, 2))  # dp table

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
    beta_1 -= 1
    beta_2 -= 1
    print(cnt)
    print_table(L1)
    print_table(L2)

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
    print('Lane 1:')
    while len(order_stack) > 0:
        print(order_stack.pop())

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
    print('Lane 2:')
    while len(order_stack) > 0:
        print(order_stack.pop())

    

def main():
    alpha = np.random.randint(1,11)
    beta = np.random.randint(1,11)
    gamma = np.random.randint(1,11)
    print(alpha)
    print(beta)
    print(gamma)
    a = np.array([0])
    b = np.array([0])
    c = np.array([0])
    
    # Randomly generate earliest arrival time
    pA = 1. / 3 # a vehicle is generated every 6 seconds in average.
    pB = 1. / 3
    pC = 1. / 3
    alpha_tmp = alpha
    beta_tmp = beta
    gamma_tmp = gamma
    t = 1
    while alpha_tmp > 0:
        if np.random.uniform(0, 1) < pA:
            a = np.append(a, t)
            alpha_tmp -= 1
        t += 1
    t = 1
    while beta_tmp > 0:
        if np.random.uniform(0, 1) < pB:
            b = np.append(b, t)
            beta_tmp -= 1
        t += 1
    t = 1
    while gamma_tmp > 0:
        if np.random.uniform(0, 1) < pC:
            c = np.append(c, t)
            gamma_tmp -= 1
        t += 1
    
    print(a)
    print(b)
    print(c)

    schedule(alpha, beta, gamma, a, b, c)


if __name__ == '__main__':
    main()