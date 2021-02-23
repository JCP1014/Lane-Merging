def multiDim_dp(a, b, c, W_same, W_diff):
    alpha = len(a) - 1
    beta = len(b) - 1
    gamma = len(c) - 1
    # L_AB = np.zeros((alpha+1, beta+1, gamma+1), dtype=(float,2))
    # L_AC = np.zeros((alpha+1, beta+1, gamma+1), dtype=(float,2))
    # L_BB = np.zeros((alpha+1, beta+1, gamma+1), dtype=(float,2))
    # L_BC = np.zeros((alpha+1, beta+1, gamma+1), dtype=(float,2))
    L_AB = [[[(0.0, 0.0) for k in range(gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_AC = [[[(0.0, 0.0) for k in range(gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_BB = [[[(0.0, 0.0) for k in range(gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]
    L_BC = [[[(0.0, 0.0) for k in range(gamma+1)] for j in range(beta+1)] for i in range(alpha+1)]

    t0 = time.time()
    # Initialize
    L_AB[0][0][0] = (0.0, 0.0)
    L_AC[0][0][0] = (0.0, 0.0)
    L_BB[0][0][0] = (0.0, 0.0)
    L_BC[0][0][0] = (0.0, 0.0)

    L_AB[1][1][0] = (a[1], b[1])
    L_AC[1][0][1] = (a[1], c[1])
    L_BC[0][1][1] = (b[1], c[1])
    L_BB[0][2][0] = (b[1], b[2]) if a[1] <= c[1] else (b[2], b[1])
    L_BB[0][1][0] = L_BB[0][2][0]
    L_BB[0][3][0] = min(
                        (max(b[3], b[1]+W_same), b[2]),
                        (b[1], max(b[3], b[2]+W_same)),
                        (max(b[2], b[1]+W_same), b[3]),
                        key=max
    )
    if L_BB[0][3][0][0] > L_BB[0][3][0][1]:
        if a[1] < c[1]:
            L_BB[0][3][0] = L_BB[0][3][0][::-1]
    elif L_BB[0][3][0][0] < L_BB[0][3][0][1]:
        if c[1] < a[1]:
            L_BB[0][3][0] = L_BB[0][3][0][::-1]

    
    
    print_table(L_AB, 'L_AB')
    print_table(L_AC, 'L_AC')
    print_table(L_BB, 'L_BB')
    print_table(L_BC, 'L_BC')
    print('------------------------1')
            
    for i in range(2, alpha+1):
        L_AB[i][1][0] = ( max(a[i], L_AB[i-1][1][0][0]+W_same), b[1] )
    for j in range(2, beta+1):
        L_AB[1][j][0] = ( a[1], max(b[j], L_AB[1][j-1][0][1]+W_same) )
    for i in range(2, alpha+1):
        L_AC[i][0][1] = ( max(a[i], L_AC[i-1][0][1][0]+W_same), c[1] )
    for k in range(2, gamma+1):
        L_AC[1][0][k] = ( a[1], max(c[k], L_AC[1][0][k-1][1]+W_same) )
    for j in range(2, beta+1):
        L_BC[0][j][1] = ( max(b[j], L_BC[0][j-1][1][0]+W_same), c[1] )
    for k in range(2, gamma+1):
        L_BC[0][1][k] = ( b[1], max(c[k], L_BC[0][1][k-1][1]+W_same) )
    for j in range(4, beta+1):
        L_BB[0][j][0] = min(
                            (max(b[j-1], L_BB[i][j-2][k][0]+W_same), max(b[j], L_BB[i][j-2][k][1]+W_same)),
                            (max(b[j], L_BB[i][j-2][k][0]+W_same), max(b[j-1], L_BB[i][j-2][k][1]+W_same)),
                            (max(b[j], max(b[j-1], L_BB[i][j-2][k][0]+W_same)+W_same), L_BB[i][j-2][k][1]),
                            (L_BB[i][j-2][k][0], max(b[j], max(b[j-1], L_BB[i][j-2][k][1]+W_same)+W_same)),
                            key=max
        )


    L_AB[1][0][0] = (a[1], 0)
    for i in range(2, alpha+1):
        L_AB[i][0][0] = (max(a[i], L_AB[i-1][0][0][0]+W_same), 0)
    L_AC[1][0][0] = (a[1], 0)
    for i in range(2, alpha+1):
        L_AC[i][0][0] = (max(a[i], L_AC[i-1][0][0][0]+W_same), 0)
    L_AC[0][0][1] = (0, c[1])
    for k in range(2, gamma+1):
        L_AC[0][0][k] = (0, max(c[k], L_AC[0][0][k-1][1]+W_same))
    L_BC[0][0][1] = (0, c[1])
    for k in range(2, gamma+1):
        L_BC[0][0][k] = (0, max(c[k], L_BC[0][0][k-1][1]+W_same))
    
    L_AB[0][1][0] = (0, b[1])
    for k in range(1, beta+1):
        L_AB[0][1][k] =  



    for i in range(2, alpha+1):
        for k in range(2, gamma+1):
            L_AC[i][0][k] = (max(a[i], L_AC[i-1][0][k-1][0]+W_same), max(c[k], L_AC[i-1][0][k-1][1]+W_same))
    for i in range(0, alpha+1):
        for k in range(1, gamma+1):
            L_AB[i][0][k] = (L_AC[i][0][k][0], L_AC[i][0][k][1]+W_diff)
    for i in range(1, alpha+1):
        for k in range(1, gamma+1):
            L_BC[i][0][k] = (L_AC[i][0][k][0]+W_diff, L_AC[i][0][k][1])
    for i in range(1, alpha+1):
        for k in range(1, gamma+1):
            L_BB[i][0][k] = (L_AC[i][0][k][0]+W_diff, L_AC[i][0][k][1]+W_diff)
    for k in range(1, gamma+1):
        L_BB[0][0][k] = min(
                            (b[1], max(b[2], L_AC[0][0][k][1]+W_diff)),
                            (b[2], max(b[1], L_AC[0][0][k][1]+W_diff)),
                            key=max
        )
    

    # L_AB[1][1][1] = (a[1], max(b[1], c[1]+W_diff))
    # for k in range(2, gamma+1):
    #     L_AB[1][1][k] = (a[1], max(b[1], L_))

    print_table(L_AB, 'L_AB')
    print_table(L_AC, 'L_AC')
    print_table(L_BB, 'L_BB')
    print_table(L_BC, 'L_BC')
    print('------------------------2')
    
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
    print_table(L_AB, 'L_AB')
    print_table(L_AC, 'L_AC')
    print_table(L_BB, 'L_BB')
    print_table(L_BC, 'L_BC')