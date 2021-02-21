import numpy as np
import time
from collections import namedtuple
Sol = namedtuple("Sol", "time last")

def print_table(L):
    for i in range(len(L)):
        for j in range(len(L[0])):
            print(L[i][j], end=' ')
        print('')

def schedule(alpha, beta, a, b):
    W_same = 1  # the waiting time if two consecutive vehicles are from the same lane
    W_diff = 3  # the waiting time if two consecutive vehicles are from different lanes
    # L = np.zeros((alpha+1, beta+1, 2))  # dp table
    L_A = [[ Sol(float('inf'), '') for j in range(beta+1)] for i in range(alpha+1)]
    L_B = [[ Sol(float('inf'),'') for j in range(beta+1)] for i in range(alpha+1)]
    

    t0 = time.time()
    # Initialize
    # L_A[0][0][0] = Sol(0.0, '')
    # L_B[0][0][0] = Sol(0.0, '')

    # Initialize
    L_A[0][0] = Sol(0, '')
    L_B[0][0] = Sol(0, '')
    L_A[1][0] = Sol(a[1], '')
    L_B[0][1] = Sol(b[1], '')
    for i in range(2, alpha+1):
        L_A[i][0] = Sol(max(a[i], L_A[i-1][0].time+W_same), 'A')
    for j in range(2, beta+1):
        L_B[0][j] = Sol(max(b[j], L_B[0][j-1].time+W_same), 'B')
    for j in range(1, beta+1):
        L_A[0][j] = Sol(L_B[0][j].time + W_diff, '')
    for i in range(1, alpha+1):
        L_B[i][0] = Sol(L_A[i][0].time + W_diff, '')
    
    # Compute table
    for i in range(1, alpha+1):
        for j in range(1, beta+1):
            # L(i, j, A)
            if max(a[i], L_A[i-1][j].time+W_same) <= max(a[i], L_B[i-1][j].time+W_diff):
                L_A[i][j] = Sol(max(a[i], L_A[i-1][j].time+W_same), 'A')
            else:
                L_A[i][j] = Sol(max(a[i], L_B[i-1][j].time+W_diff), 'B')
            
            # L(i, j, B)
            if max(b[j], L_A[i][j-1].time+W_diff) <= max(b[j], L_B[i][j-1].time+W_same):
                L_B[i][j] = Sol(max(b[j], L_A[i][j-1].time+W_diff), 'A')
            else:
                L_B[i][j] = Sol(max(b[j], L_B[i][j-1].time+W_same), 'B')

            # L[i][j][0] = min( max(a[i], L[i-1][j][0]+W_same), max(a[i], L[i-1][j][1]+W_diff) )
            # L[i][j][1] = min( max(b[j], L[i][j-1][0]+W_diff), max(b[j], L[i][j-1][1]+W_same) )

    print_table(L_A)
    print_table(L_B)

    # Choose optimal solution
    order_stack = []
    lastFrom = ''
    i = alpha
    j = beta
    if L_A[i][j].time <= L_B[i][j].time:
        order_stack.append(('A', L_A[i][j].time))
        lastFrom = L_A[i][j].last
        i -= 1
    else:
        order_stack.append(('B', L_B[i][j].time))
        lastFrom = L_B[i][j].last
        j -= 1
    while i>0 or j>0:
        if lastFrom == 'A':
            order_stack.append(('A', L_A[i][j].time))
            lastFrom = L_A[i][j].last
            i -= 1
        else:
            order_stack.append(('B', L_B[i][j].time))
            lastFrom = L_B[i][j].last
            j -= 1

    # Output order
    while len(order_stack) > 0:
        print(order_stack.pop())
    

def main():
    alpha = np.random.randint(1,11)
    beta = np.random.randint(1,11)
    print(alpha)
    print(beta)
    a = np.array([0])
    b = np.array([0])
    
    # Randomly generate earliest arrival time
    pA = 1. / 3 # a vehicle is generated every 6 seconds in average.
    pB = 1. / 3
    alpha_tmp = alpha
    beta_tmp = beta
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
    
    # s = np.random.poisson(20, alpha+beta)
    # index = 0
    # for i in range(alpha):
    #     a = np.append(a, s[index])
    #     index += 1
    # for i in range(beta):
    #     b = np.append(b, s[index])
    #     index += 1
    # a = np.sort(a)
    # b = np.sort(b)

    alpha = 2
    beta = 2
    # a = np.array([0, 7.8, 9.5, 11.0])
    # b = np.array([0, 4.2, 7.1, 11.8])
    a = np.array([0, 1, 3])
    b = np.array([0, 2, 4])
    print(a)
    print(b)

    schedule(alpha, beta, a, b)


if __name__ == '__main__':
    main()