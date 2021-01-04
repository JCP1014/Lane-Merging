import numpy as np

def print_table(L):
    for k in range(L.shape[2]):
        for i in range(L.shape[0]):
            for j in range(L.shape[1]):
                print(L[i][j][k], end=' ')
            print('')
        print('')

def schedule(alpha, beta, a, b):
    W_same = 1  # the waiting time if two consecutive vehicles are from the same lane
    W_diff = 3  # the waiting time if two consecutive vehicles are from different lanes
    L = np.zeros((alpha+1, beta+1, 2))  # dp table

    # Initialize
    L[0][0][0] = 0
    L[0][0][1] = 0
    L[1][0][0] = a[1]
    L[0][1][1] = b[1]
    for i in range(2, alpha+1):
        L[i][0][0] = max(a[i], L[i-1][0][0]+W_same)
    for j in range(2, beta+1):
        L[0][j][1] = max(b[j], L[0][j-1][1]+W_same)
    for j in range(1, beta+1):
        L[0][j][0] = L[0][j][1] + W_diff
    for i in range(1, alpha+1):
        L[i][0][1] = L[i][0][0] + W_diff
    
    # Compute table
    for i in range(1, alpha+1):
        for j in range(1, beta+1):
            L[i][j][0] = min( max(a[i], L[i-1][j][0]+W_same), max(a[i], L[i-1][j][1]+W_diff) )
            L[i][j][1] = min( max(b[j], L[i][j-1][0]+W_diff), max(b[j], L[i][j-1][1]+W_same) )

    # Choose optimal solution
    order_stack = []
    i = alpha
    j = beta
    while i>0 or j>0:
        if L[i][j][1] < L[i][j][0]:
            order_stack.append(('B', j, L[i][j][1]))
            j -= 1
        else:
            order_stack.append(('A', i, L[i][j][0]))
            i -= 1

    # Output order
    while len(order_stack) > 0:
        print(order_stack.pop())
    

def main():
    alpha = 2
    beta = 2
    a = np.array([0, 4, 11])
    b = np.array([0, 7, 9])
    schedule(alpha, beta, a, b)

    # W_same = 1
    # W_diff = 3
    # alpha = 2
    # beta = 2
    # a = np.array([0])
    # b = np.array([0])
    # L = np.zeros((alpha+1, beta+1, 2))
    
    # # Randomly generate earliest arrival time
    # s = np.random.poisson(10, 10)
    # index = 0
    # for i in range(alpha):
    #     a = np.append(a, s[index])
    #     index += 1
    # for i in range(beta):
    #     b = np.append(b, s[index])
    #     index += 1
    # a = np.sort(a)
    # b = np.sort(b)
    # a = np.array([0, 1, 3])
    # b = np.array([0, 2, 4])
    # print(a)
    # print(b)

    # # Initialize
    # L[0][0][0] = 0
    # L[0][0][1] = 0
    # L[1][0][0] = a[1]
    # L[0][1][1] = b[1]
    # print_table(L)

    # for i in range(2, alpha+1):
    #     L[i][0][0] = max(a[i], L[i-1][0][0]+W_same)
    # for j in range(2, beta+1):
    #     L[0][j][1] = max(b[j], L[0][j-1][1]+W_same)
    # for j in range(1, beta+1):
    #     L[0][j][0] = L[0][j][1] + W_diff
    # for i in range(1, alpha+1):
    #     L[i][0][1] = L[i][0][0] + W_diff
    
    # for i in range(1, alpha+1):
    #     for j in range(1, beta+1):
    #         L[i][j][0] = min( max(a[i], L[i-1][j][0]+W_same), max(a[i], L[i-1][j][1]+W_diff) )
    #         L[i][j][1] = min( max(b[j], L[i][j-1][0]+W_diff), max(b[j], L[i][j-1][1]+W_same) )

    # print_table(L)

    # order_stack = []
    # i = alpha
    # j = beta
    # while i>0 or j>0:
    #     if L[i][j][1] < L[i][j][0]:
    #         order_stack.append(('B', j, L[i][j][1]))
    #         j -= 1
    #     else:
    #         order_stack.append(('A', i, L[i][j][0]))
    #         i -= 1

    # while len(order_stack) > 0:
    #     print(order_stack.pop())


if __name__ == '__main__':
    main()