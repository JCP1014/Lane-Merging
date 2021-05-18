def fcfg_v1(a, b, c, W_same, W_diff):
    a = a[1:]
    b = b[1:]
    c = c[1:]
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
                if a[0] <= b[0] and c[0] <= b[0]:
                    # print(a[0], t)
                    a = a[1:]
                    nextA = t + W_same
                    nextB1 = t + W_diff
                    # print(c[0], t)
                    c = c[1:]
                    nextC = t + W_same
                    nextB2 = t + W_diff
                elif c[0] <= b[0] and nextB1 <= t:
                    # print(c[0], t)
                    c = c[1:]
                    nextC = t + W_same
                    nextB2 = t + W_diff
                    # print(b[0], t)
                    b = b[1:]
                    nextB1 = t + W_same
                    nextA = t + W_diff
                elif a[0] <= b[0] and nextB2 <= t:
                    # print(a[0], t)
                    a = a[1:]
                    nextA = t + W_same
                    nextB1 = t + W_diff
                    # print(b[0], t)
                    b = b[1:]
                    nextB2 = t + W_same
                    nextC = t + W_diff
                elif c[0] <= b[0]:
                    # print(c[0], t)
                    c = c[1:]
                    nextC = t + W_same
                    nextB2 = t + W_diff
                elif a[0] <= b[0]:
                    # print(a[0], t)
                    a = a[1:]
                    nextA = t + W_same
                    nextB1 = t + W_diff
                elif nextB1 <= t:
                    # print(b[0], t)
                    b = b[1:]
                    nextB2 = t + W_same
                    nextC = t + W_diff
                elif nextB2 <= t:
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