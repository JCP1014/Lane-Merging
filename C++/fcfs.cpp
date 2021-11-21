#include "fcfs.h"

tuple<char, int, float> schedule_single_lane(char lane, vector<float> traffic, float W_same, float W_diff, tuple<char, int, float> prev)
{
    char prevLane = get<0>(prev);
    float prevTime = get<2>(prev);
    tuple<char, int, float> last = make_tuple('0', 0, 0.0);
    if (prevLane == '0')
    {
        last = make_tuple(lane, 1, traffic[1]);
    }
    else if (lane == prevLane)
    {
        last = make_tuple(lane, 1, max(traffic[1], prevTime + W_same));
    }
    else
    {
        last = make_tuple(lane, 1, max(traffic[1], prevTime + W_diff));
    }
    for (int i = 2; i < traffic.size(); ++i)
    {
        prevTime = get<2>(last);
        last = make_tuple(lane, i, max(traffic[i], prevTime + W_same));
    }
    return last;
}

tuple<float, double> first_come_first_serve_v1(float timeStep, vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff)
{
    auto t0 = chrono::high_resolution_clock::now();
    vector<float> a(a_all.begin() + 1, a_all.end());
    vector<float> b(b_all.begin() + 1, b_all.end());
    vector<float> c(c_all.begin() + 1, c_all.end());
    float t = 0;         // time
    char B_prevTo = 'Y'; // Which lane the previous vehicle in lane B go to
    float X_lastT = -W_diff;
    float Y_lastT = -W_diff;
    char X_lastFrom = '0';
    char Y_lastFrom = '0';

    while (a.size() > 0 || b.size() > 0 || c.size() > 0)
    {
        t = t + timeStep;
        if (a.size() > 0 && b.size() > 0 && c.size() > 0)
        {
            if (a[0] == t && b[0] == t && c[0] == t)
            {
                if (a.size() > 1 && b.size() > 1 && c.size() > 1)
                {
                    if (b[1] >= a[1] && b[1] >= c[1])
                    {
                        if (X_lastFrom == 'A')
                            X_lastT = max(a[0], X_lastT + W_same);
                        else
                            X_lastT = max(a[0], X_lastT + W_diff);
                        a.erase(a.begin());
                        X_lastFrom = 'A';

                        if (Y_lastFrom == 'C')
                            Y_lastT = max(c[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(c[0], Y_lastT + W_diff);
                        c.erase(c.begin());
                        Y_lastFrom = 'C';

                        if (X_lastT <= t && Y_lastT <= t)
                        {
                            if (a[0] >= c[0])
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                        }
                        else
                        {
                            if (X_lastT < Y_lastT)
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else if (Y_lastT < X_lastT)
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                            else
                            {
                                if (a[0] >= c[0])
                                {
                                    X_lastT = max(b[0], X_lastT + W_diff);
                                    b.erase(b.begin());
                                    X_lastFrom = 'B';
                                }
                                else
                                {
                                    Y_lastT = max(b[0], Y_lastT + W_diff);
                                    b.erase(b.begin());
                                    Y_lastFrom = 'B';
                                }
                            }
                        }
                    }
                    else if (a[1] >= b[1] && a[1] >= c[1])
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';

                        if (Y_lastFrom == 'C')
                            Y_lastT = max(c[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(c[0], Y_lastT + W_diff);
                        c.erase(c.begin());
                        Y_lastFrom = 'C';

                        X_lastT = max(a[0], X_lastT + W_diff);
                        a.erase(a.begin());
                        X_lastFrom = 'A';
                    }
                    else if (c[1] >= a[1] && c[1] >= b[1])
                    {
                        if (X_lastFrom == 'A')
                            X_lastT = max(a[0], X_lastT + W_same);
                        else
                            X_lastT = max(a[0], X_lastT + W_diff);
                        a.erase(a.begin());
                        X_lastFrom = 'A';

                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';

                        Y_lastT = max(c[0], Y_lastT + W_diff);
                        c.erase(c.begin());
                        Y_lastFrom = 'C';
                    }
                }
                else if (b.size() < 2)
                {
                    if (X_lastFrom == 'A')
                        X_lastT = max(a[0], X_lastT + W_same);
                    else
                        X_lastT = max(a[0], X_lastT + W_diff);
                    a.erase(a.begin());
                    X_lastFrom = 'A';

                    if (Y_lastFrom == 'C')
                        Y_lastT = max(c[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(c[0], Y_lastT + W_diff);
                    c.erase(c.begin());
                    Y_lastFrom = 'C';

                    if (X_lastT <= t && Y_lastT <= t)
                    {
                        if (a.size() < 1)
                        {
                            X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                        else if (c.size() < 1)
                        {
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                            b.erase(b.begin());
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (a[0] >= c[0])
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                        }
                    }
                    else
                    {
                        if (X_lastT < Y_lastT)
                        {
                            X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                        else if (Y_lastT < X_lastT)
                        {
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (a.size() < 1)
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else if (c.size() < 1)
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                            else
                            {
                                if (a[0] >= c[0])
                                {
                                    X_lastT = max(b[0], X_lastT + W_diff);
                                    b.erase(b.begin());
                                    X_lastFrom = 'B';
                                }
                                else
                                {
                                    Y_lastT = max(b[0], Y_lastT + W_diff);
                                    b.erase(b.begin());
                                    Y_lastFrom = 'B';
                                }
                            }
                        }
                    }
                }
                else if (a.size() < 2)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    b.erase(b.begin());
                    X_lastFrom = 'B';

                    if (Y_lastFrom == 'C')
                        Y_lastT = max(c[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(c[0], Y_lastT + W_diff);
                    c.erase(c.begin());
                    Y_lastFrom = 'C';

                    X_lastT = max(a[0], Y_lastT + W_diff);
                    a.erase(a.begin());
                    X_lastFrom = 'A';
                }
                else if (c.size() < 2)
                {
                    if (X_lastFrom == 'A')
                        X_lastT = max(a[0], X_lastT + W_same);
                    else
                        X_lastT = max(a[0], X_lastT + W_diff);
                    a.erase(a.begin());
                    X_lastFrom = 'A';

                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    Y_lastFrom = 'B';

                    Y_lastT = max(c[0], Y_lastT + W_diff);
                    c.erase(c.begin());
                    Y_lastFrom = 'C';
                }
            }
            else if (a[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (a[0] == t && b[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'B')
                    Y_lastT = max(b[0], Y_lastT + W_same);
                else
                    Y_lastT = max(b[0], Y_lastT + W_diff);
                b.erase(b.begin());
                Y_lastFrom = 'B';
            }
            else if (b[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'B')
                    X_lastT = max(b[0], X_lastT + W_same);
                else
                    X_lastT = max(b[0], X_lastT + W_diff);
                b.erase(b.begin());
                X_lastFrom = 'B';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (b[0] == t)
            {
                if (X_lastT <= t && Y_lastT <= t)
                {
                    if (c[0] > a[0])
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                }
                else
                {
                    if (X_lastT <= t && Y_lastT <= t)
                    {
                        if (c[0] > a[0])
                        {
                            if (Y_lastFrom == 'B')
                                Y_lastT = max(b[0], Y_lastT + W_same);
                            else
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                            b.erase(b.begin());
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (X_lastFrom == 'B')
                                X_lastT = max(b[0], X_lastT + W_same);
                            else
                                X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                    }
                    else
                    {
                        if (X_lastT < Y_lastT)
                        {
                            if (X_lastFrom == 'B')
                                X_lastT = max(b[0], X_lastT + W_same);
                            else
                                X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                        else if (Y_lastT < X_lastT)
                        {
                            if (Y_lastFrom == 'B')
                                Y_lastT = max(b[0], Y_lastT + W_same);
                            else
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                            b.erase(b.begin());
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (c[0] > a[0])
                            {
                                if (Y_lastFrom == 'B')
                                    Y_lastT = max(b[0], Y_lastT + W_same);
                                else
                                    Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                            else
                            {
                                if (X_lastFrom == 'B')
                                    X_lastT = max(b[0], X_lastT + W_same);
                                else
                                    X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                        }
                    }
                }
            }
            else if (a[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';
            }
            else if (c[0] == t)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
        }
        else if (a.size() > 0 && c.size() > 0)
        {
            if (a[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (a[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';
            }
            else if (c[0] == t)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
        }
        else if (a.size() > 0 && b.size() > 0)
        {
            if (a[0] == t && b[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'B')
                    Y_lastT = max(b[0], Y_lastT + W_same);
                else
                    Y_lastT = max(b[0], Y_lastT + W_diff);
                b.erase(b.begin());
                Y_lastFrom = 'B';
            }
            else if (a[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';
            }
            else if (b[0] == t)
            {
                if (X_lastT <= t && Y_lastT <= t)
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    b.erase(b.begin());
                    Y_lastFrom = 'B';
                }
                else
                {
                    if (X_lastT < Y_lastT)
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                    else if (Y_lastT < X_lastT)
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                    else
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                }
            }
        }
        else if (b.size() > 0 && c.size() > 0)
        {
            if (b[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'B')
                    X_lastFrom = max(b[0], X_lastT + W_same);
                else
                    X_lastT = max(b[0], X_lastT + W_diff);
                b.erase(b.begin());
                X_lastFrom = 'B';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (c[0] == t)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (b[0] == t)
            {
                if (X_lastT <= t && Y_lastT <= t)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    b.erase(b.begin());
                    X_lastFrom = 'B';
                }
                else
                {
                    if (X_lastT < Y_lastT)
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                    else if (Y_lastT < X_lastT)
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                }
            }
        }
        else if (a.size() > 0)
        {
            if (X_lastFrom == 'A')
                X_lastT = max(a[0], X_lastT + W_same);
            else
                X_lastT = max(a[0], X_lastT + W_diff);
            a.erase(a.begin());
            X_lastFrom = 'A';
        }
        else if (c.size() > 0)
        {
            if (Y_lastFrom == 'C')
                Y_lastT = max(c[0], Y_lastT + W_same);
            else
                Y_lastT = max(c[0], Y_lastT + W_diff);
            c.erase(c.begin());
            Y_lastFrom = 'C';
        }
        else if (b.size() > 0)
        {
            if (B_prevTo == 'Y')
            {
                if (X_lastFrom == 'B')
                    X_lastT = max(b[0], X_lastT + W_same);
                else
                    X_lastT = max(b[0], X_lastT + W_diff);
                b.erase(b.begin());
                X_lastFrom = 'B';
                B_prevTo = 'X';
            }
            else
            {
                if (Y_lastFrom == 'B')
                    Y_lastT = max(b[0], Y_lastT + W_same);
                else
                    Y_lastT = max(b[0], Y_lastT + W_diff);
                b.erase(b.begin());
                Y_lastFrom = 'B';
                B_prevTo = 'Y';
            }
        }
    }
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    float T_last = max(X_lastT, Y_lastT);
    cout << "fcfs result: " << T_last << " " << totalComputeTime << endl;
    return make_tuple(T_last, totalComputeTime);
}

tuple<float, double> first_come_first_serve_v2(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff)
{
    auto t0 = chrono::high_resolution_clock::now();
    vector<float> a(a_all.begin() + 1, a_all.end());
    vector<float> b(b_all.begin() + 1, b_all.end());
    vector<float> c(c_all.begin() + 1, c_all.end());
    float X_lastT = -W_diff;
    float Y_lastT = -W_diff;
    char X_lastFrom = '0';
    char Y_lastFrom = '0';
    float first = 0;
    default_random_engine generator(time(NULL));
    uniform_int_distribution<int> distribution(0, 1);
    int tmp;

    while (a.size() > 0 || b.size() > 0 || c.size() > 0)
    {
        if (a.size() > 0 && b.size() > 0 && c.size() > 0)
        {
            first = min({a[0], b[0], c[0]});
            if (first == a[0])
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                X_lastFrom = 'A';
                a.erase(a.begin());
            }
            if (first == c[0])
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                Y_lastFrom = 'C';
                c.erase(c.begin());
            }
            if (first == b[0])
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    X_lastFrom = 'B';
                    b.erase(b.begin());
                }
                else if (Y_lastT < X_lastT)
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    Y_lastFrom = 'B';
                    b.erase(b.begin());
                }
                else
                {
                    if (a[0] > c[0])
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        X_lastFrom = 'B';
                        b.erase(b.begin());
                    }
                    else if (c[0] > a[0])
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        Y_lastFrom = 'B';
                        b.erase(b.begin());
                    }
                    else
                    {
                        if (distribution(generator) == 0)
                        {
                            if (X_lastFrom == 'B')
                                X_lastT = max(b[0], X_lastT + W_same);
                            else
                                X_lastT = max(b[0], X_lastT + W_diff);
                            X_lastFrom = 'B';
                            b.erase(b.begin());
                        }
                        else
                        {
                            if (Y_lastFrom == 'B')
                                Y_lastT = max(b[0], Y_lastT + W_same);
                            else
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                            Y_lastFrom = 'B';
                            b.erase(b.begin());
                        }
                    }
                }
            }
        }
        else if (a.size() > 1 && b.size() > 1)
        {
            tie(X_lastFrom, tmp, X_lastT) = schedule_single_lane('A', a, W_same, W_diff, make_tuple(X_lastFrom, 0, X_lastT));
            tie(Y_lastFrom, tmp, Y_lastT) = schedule_single_lane('B', b, W_same, W_diff, make_tuple(Y_lastFrom, 0, Y_lastT));
            a.clear();
            b.clear();
        }
        else if (a.size() > 0 and c.size() > 0)
        {
            tie(X_lastFrom, tmp, X_lastT) = schedule_single_lane('A', a, W_same, W_diff, make_tuple(X_lastFrom, 0, X_lastT));
            tie(Y_lastFrom, tmp, Y_lastT) = schedule_single_lane('C', c, W_same, W_diff, make_tuple(Y_lastFrom, 0, Y_lastT));
            a.clear();
            c.clear();
        }
        else if (b.size() > 0 and c.size() > 0)
        {
            tie(X_lastFrom, tmp, X_lastT) = schedule_single_lane('B', b, W_same, W_diff, make_tuple(X_lastFrom, 0, X_lastT));
            tie(Y_lastFrom, tmp, Y_lastT) = schedule_single_lane('C', c, W_same, W_diff, make_tuple(Y_lastFrom, 0, Y_lastT));
            b.clear();
            c.clear();
        }
        else if (a.size() > 0)
        {
            tie(X_lastFrom, tmp, X_lastT) = schedule_single_lane('A', a, W_same, W_diff, make_tuple(X_lastFrom, 0, X_lastT));
            a.clear();
        }
        else if (c.size() > 0)
        {
            tie(Y_lastFrom, tmp, Y_lastT) = schedule_single_lane('C', c, W_same, W_diff, make_tuple(Y_lastFrom, 0, Y_lastT));
            c.clear();
        }
        else if (b.size() > 0)
        {
            while (b.size() > 0)
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    X_lastFrom = 'B';
                    b.erase(b.begin());
                }
                else
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    Y_lastFrom = 'B';
                    b.erase(b.begin());
                }
            }
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    float T_last = max(X_lastT, Y_lastT);
    cout << "fcfs result: " << T_last << " " << totalComputeTime << endl;
    return make_tuple(T_last, totalComputeTime);
}
