from collections import namedtuple
Event = namedtuple("Event", "lane time")
def fcfg(a, b, c, W_same, W_diff):  # v3
    a = a[1:]
    b = b[1:]
    c = c[1:]
    t = 0   # time
    qX = [] # Vehicles go to lane X
    qY = [] # Vehicles go to lane Y
    B_prevTo = 'Y'
    lenX = 0
    lenY = 0
    waiting = 0

    while len(a) > 0 or len(b) > 0 or len(c) > 0:
        t = round(t+0.1, 2)
        if len(a) > 0 and len(b) > 0 and len(c) > 0:
            if a[0] <= t and b[0] <= t and c[0] <= t:
                if len(a) > 1 and len(b) > 1 and len(c) > 1: 
                    if b[1] >= a[1] and b[1] >= c[1]:
                        qX.append( Event('A', a.pop(0)) )
                        qY.append( Event('C', c.pop(0)) )
                        waiting = Event('B', b.pop(0))
                        # if a[0] >= c[0]:
                        #     qX.append( Event('B', b.pop(0)) )
                        # else:
                        #     qY.append( Event('B', b.pop(0)) )
                    elif a[1] >= b[1] and a[1] >= c[1]:
                        qX.append( Event('B', b.pop(0)) )
                        qY.append( Event('C', c.pop(0)) )
                        qX.append( Event('A', a.pop(0)) )
                    elif c[1] >= a[1] and c[1] >= b[1]:
                        qX.append( Event('A', a.pop(0)) )
                        qY.append( Event('B', b.pop(0)) )
                        qY.append( Event('C', c.pop(0)) )
                elif len(b) < 2:
                    qX.append( Event('A', a.pop(0)) )
                    qY.append( Event('C', c.pop(0)) )
                    if len(a) < 1:
                        qX.append( Event('B', b.pop(0)) )
                    elif len(c) < 1:
                        qY.append( Event('B', b.pop(0)) )
                    else:
                        if a[0] >= c[0]:
                            qX.append( Event('B', b.pop(0)) )
                        else:
                            qY.append( Event('B', b.pop(0)) )
                elif len(a) < 2:
                    qX.append( Event('B', b.pop(0)) )
                    qY.append( Event('C', c.pop(0)) )
                    qX.append( Event('A', a.pop(0)) )
                elif len(c) < 2:
                    qX.append( Event('A', a.pop(0)) )
                    qY.append( Event('B', b.pop(0)) )
                    qY.append( Event('C', c.pop(0)) )
            elif a[0] <= t and c[0] <= t:
                qX.append( Event('A', a.pop(0)) )
                qY.append( Event('C', c.pop(0)) )
            elif a[0] <= t and b[0] <= t:
                qX.append( Event('A', a.pop(0)) )
                qY.append( Event('B', b.pop(0)) )
            elif b[0] <= t and c[0] <= t:
                qX.append( Event('B', b.pop(0)) )
                qY.append( Event('C', c.pop(0)) )
            elif b[0] <= t:
                if c[0] > a[0]:
                    qY.append( Event('B', b.pop(0)) )
                else:
                    qX.append( Event('B', b.pop(0)) )  
            elif a[0] <= t:
                qX.append( Event('A', a.pop(0)) )
            elif c[0] <= t:
                qY.append( Event('C', c.pop(0)) )
        elif len(a) > 0 and len(c) > 0: 
            if a[0] <= t and c[0] <= t:
                qX.append( Event('A', a.pop(0)) )
                qY.append( Event('C', c.pop(0)) )
            elif a[0] <= t:
                qX.append( Event('A', a.pop(0)) )
            elif c[0] <= t:
                qY.append( Event('C', c.pop(0)) )
        elif len(a) > 0 and len(b) > 0:
            if a[0] <= t and b[0] <= t:
                qX.append( Event('A', a.pop(0)) )
                qY.append( Event('B', b.pop(0)) )
            elif a[0] <= t:
                qX.append( Event('A', a.pop(0)) )
            elif b[0] <= t:
                qY.append( Event('B', b.pop(0)) )
        elif len(b) > 0 and len(c) > 0:
            if b[0] <= t and c[0] <= t:
                qX.append( Event('B', b.pop(0)) )
                qY.append( Event('C', c.pop(0)) )
            elif c[0] <= t:
                qY.append( Event('C', c.pop(0)) )
            elif b[0] <= t:
                qX.append( Event('B', b.pop(0)) )  
        elif len(a) > 0:
            qX.append( Event('A', a.pop(0)) )
        elif len(c) > 0:
            qY.append( Event('C', c.pop(0)) )
        elif len(b) > 0:
            if B_prevTo == 'Y':
                qX.append( Event('B', b.pop(0)) )
                B_prevTo = 'X'
            else:
                qY.append( Event('B', b.pop(0)) )
                B_prevTo = 'Y'

        if lenX > 0:
            for i in range(lenX, len(qX)):
                if qX[i].lane == qX[i-1].lane:
                    qX[i].time = max(qX[i].time, qX[i-1].time+W_same)
                else:
                    qX[i].time = max(qX[i].time, qX[i-1].time+W_diff)
        else:
            for i in range(1, len(qX)):
                if qX[i].lane == qX[i-1].lane:
                    qX[i].time = max(qX[i].time, qX[i-1].time+W_same)
                else:
                    qX[i].time = max(qX[i].time, qX[i-1].time+W_diff)
        lenX = len(qX)

        if lenY > 0:
            for i in range(lenY, len(qY)):
                if qY[i].lane == qY[i-1].lane:
                    qY[i].time = max(qY[i].time, qY[i-1].time+W_same)
                else:
                    qY[i].time = max(qY[i].time, qY[i-1].time+W_diff)
        else:
            for i in range(1, len(qY)):
                if qY[i].lane == qY[i-1].lane:
                    qY[i].time = max(qY[i].time, qY[i-1].time+W_same)
                else:
                    qY[i].time = max(qY[i].time, qY[i-1].time+W_diff)
        lenY = len(qY)
    
    # (prevFrom, T_lastX) = qX.pop(0)
    # print(prevFrom, T_lastX)
    # for (lane, arrival) in qX:
    #     print(lane, arrival)
    #     if lane == prevFrom:
    #         T_lastX = max(arrival, T_lastX + W_same)
    #     else:
    #         T_lastX = max(arrival, T_lastX + W_diff)
    #     prevFrom = lane

    # print('---')
    # (prevFrom, T_lastY) = qY.pop(0)
    # print(prevFrom, T_lastY)
    # for (lane, arrival) in qY:
    #     print(lane, arrival)
    #     if lane == prevFrom:
    #         T_lastY = max(arrival, T_lastY + W_same)
    #     else:
    #         T_lastY = max(arrival, T_lastY + W_diff)
    #     prevFrom = lane

    return max(T_lastX, T_lastY)