# Should be revised: the part where decides first vehicles of lane X and lane Y
def fcfg(timeStep, a, b, c, W_same, W_diff):  # v4
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