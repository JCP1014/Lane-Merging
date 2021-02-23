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
    L_A = [[ Sol(float('inf'), '') for j in range(beta+1)] for i in range(alpha+1)]
    L_B = [[ Sol(float('inf'),'') for j in range(beta+1)] for i in range(alpha+1)]
    
    t0 = time.time()

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
            # Compute L(i, j, A)
            val_A = max(a[i], L_A[i-1][j].time+W_same)
            val_B = max(a[i], L_B[i-1][j].time+W_diff)
            if val_A <= val_B:
                L_A[i][j] = Sol(val_A, 'A')
            else:
                L_A[i][j] = Sol(val_B, 'B')
            # Compute L(i, j, B)
            val_A = max(b[j], L_A[i][j-1].time+W_diff)
            val_B = max(b[j], L_B[i][j-1].time+W_same)
            if val_A <= val_B:
                L_B[i][j] = Sol(val_A, 'A')
            else:
                L_B[i][j] = Sol(val_B, 'B')

    # print_table(L_A)
    # print_table(L_B)

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
    # Backtracking
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
    
    # alpha = 3
    # beta = 3
    # a = np.array([0, 7.8, 9.5, 11.0])
    # b = np.array([0, 4.2, 7.1, 11.8])
    alpha = 2
    beta = 2
    a = np.array([0, 1, 3])
    b = np.array([0, 2, 4])
    print(a)
    print(b)

    schedule(alpha, beta, a, b)


if __name__ == '__main__':
    main()