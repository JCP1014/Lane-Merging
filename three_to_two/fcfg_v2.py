def fcfg(a, b, c, W_same, W_diff):  # v2
    a = a[1:]
    b = b[1:]
    c = c[1:]
    t = 0   # time
    qX = [] # Vehicles go to laqne X
    qY = [] # Vehicles go to lane Y
    prevTo = 'Y'

    while len(a) > 0 or len(b) > 0 or len(c) > 0:
        t += 0.1
        t = round(t, 2)
        if len(a) > 0 and len(b) > 0 and len(c) > 0:
            if a[0] <= t and b[0] <= t and c[0] <= t:
                if len(a) > 1 and len(b) > 1 and len(c) > 1: 
                    if b[1] >= a[1] and b[1] >= c[1]:
                        qX.append( ('A', a.pop(0)) )
                        qY.append( ('C', c.pop(0)) )
                        if a[0] >= c[0]:
                            qX.append( ('B', b.pop(0)) )
                        else:
                            qY.append( ('B', b.pop(0)) )
                    elif a[1] >= b[1] and a[1] >= c[1]:
                        qX.append( ('B', b.pop(0)) )
                        qY.append( ('C', c.pop(0)) )
                        qX.append( ('A', a.pop(0)) )
                    elif c[1] >= a[1] and c[1] >= b[1]:
                        qX.append( ('A', a.pop(0)) )
                        qY.append( ('B', b.pop(0)) )
                        qY.append( ('C', c.pop(0)) )
                elif len(b) < 2:
                    qX.append( ('A', a.pop(0)) )
                    qY.append( ('C', c.pop(0)) )
                    if len(a) < 1:
                        qX.append( ('B', b.pop(0)) )
                    elif len(c) < 1:
                        qY.append( ('B', b.pop(0)) )
                    else:
                        if a[0] >= c[0]:
                            qX.append( ('B', b.pop(0)) )
                        else:
                            qY.append( ('B', b.pop(0)) )
                elif len(a) < 2:
                    qX.append( ('B', b.pop(0)) )
                    qY.append( ('C', c.pop(0)) )
                    qX.append( ('A', a.pop(0)) )
                elif len(c) < 2:
                    qX.append( ('A', a.pop(0)) )
                    qY.append( ('B', b.pop(0)) )
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
    print(prevFrom, T_lastX)
    for (lane, arrival) in qX:
        print(lane, arrival)
        if lane == prevFrom:
            T_lastX = max(arrival, T_lastX + W_same)
        else:
            T_lastX = max(arrival, T_lastX + W_diff)
        prevFrom = lane

    print('---')
    (prevFrom, T_lastY) = qY.pop(0)
    print(prevFrom, T_lastY)
    for (lane, arrival) in qY:
        print(lane, arrival)
        if lane == prevFrom:
            T_lastY = max(arrival, T_lastY + W_same)
        else:
            T_lastY = max(arrival, T_lastY + W_diff)
        prevFrom = lane

    return max(T_lastX, T_lastY)