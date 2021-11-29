// Compilation: g++ -std=c++11 group_milp.cpp
//                  -I /Library/gurobi912/mac64/include
//                  -L /Library/gurobi912/mac64/lib
//                  -lgurobi_c++ -lgurobi91
#include <bits/stdc++.h>
#include "gurobi_c++.h"
#define endl '\n'
using namespace std;

vector<pair<int, int>> grouping_v1(vector<float> &traffic, float timeStep)
{
    int groups = 0;
    int max_groups = 12;
    float grouping_threshold = 1;
    vector<pair<int, int>> grouped_index;

    while (1)
    {
        grouped_index.push_back({0, 0});
        float head = 1, tail = 0;
        for (int i = 2; i < traffic.size(); ++i)
        {
            if (traffic[i] - traffic[i - 1] > grouping_threshold)
            {
                if (++groups > max_groups)
                    break;
                tail = i - 1;
                grouped_index.push_back({head, tail});
                head = i;
            }
        }
        if (head == traffic.size() - 1)
            grouped_index.push_back({head, head});
        else
            grouped_index.push_back({head, traffic.size() - 1});
        if (groups > max_groups)
        {
            grouping_threshold += timeStep;
            groups = 0;
            grouped_index.clear();
        }
        else
        {
            cout << "threshold: " << grouping_threshold << endl;
            break;
        }
    }
    return grouped_index;
}

vector<pair<int, int>> grouping_v2(vector<float> &traffic, float timeStep)
{
    float grouping_threshold = 1;
    vector<pair<int, int>> grouped_index;

    grouped_index.push_back({0, 0});
    float head = 1, tail = 0;
    for (int i = 2; i < traffic.size(); ++i)
    {
        if (traffic[i] - traffic[i - 1] > grouping_threshold)
        {
            tail = i - 1;
            grouped_index.push_back({head, tail});
            head = i;
        }
    }
    if (head == traffic.size() - 1)
        grouped_index.push_back({head, head});
    else
        grouped_index.push_back({head, traffic.size() - 1});
    return grouped_index;
}

float digit_round(float value, int digit)
{
    return roundf(value * pow(10, digit)) / pow(10, digit);
}

vector<float> generate_traffic(float timeStep, int num, float p, int seed)
{
    vector<float> earliestArrivalTimes;
    float t;
    default_random_engine generator(time(NULL) + seed);
    uniform_real_distribution<float> unif(0.0, 1.0);

    earliestArrivalTimes.push_back(0.0);

    t = 1.0;
    while (num > 0)
    {
        if (unif(generator) < p)
        {
            earliestArrivalTimes.push_back(digit_round(t, 1));
            num -= 1;
        }
        t += timeStep;
    }
    return earliestArrivalTimes;
}

void solve(vector<float> &A, vector<float> &B, vector<float> &C, float W_same, float W_diff, vector<pair<int, int>> &grouped_A, vector<pair<int, int>> &grouped_B, vector<pair<int, int>> &grouped_C)
{
    int alpha = A.size() - 1;
    int beta = B.size() - 1;
    int gamma = C.size() - 1;
    float D = 30;
    int L = grouped_A.size() - 1;
    int M = grouped_B.size() - 1;
    int N = grouped_C.size() - 1;

    try
    {
        // Create an environment
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "milp.log");
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        // Create variables: s_i, t_j, u_k (scheduled entering time)
        GRBVar s[alpha + 1], t[beta + 1], u[gamma + 1];
        for (int i = 0; i <= alpha; ++i)
        {
            s[i] = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "s_" + to_string(i));
        }
        for (int j = 0; j <= beta; ++j)
        {
            t[j] = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "t_" + to_string(j));
        }
        for (int k = 0; k <= gamma; ++k)
        {
            u[k] = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "u_" + to_string(k));
        }

        // Create variables: ph_l, pt_l, qh_m, qt_m, rh_n, rt_n (scheduled entering time of groups)
        GRBVar ph[L + 1], pt[L + 1], qh[M + 1], qt[M + 1], rh[N + 1], rt[N + 1];
        for (int l = 0; l <= L; ++l)
        {
            ph[l] = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "ph_" + to_string(l));
            pt[l] = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "pt_" + to_string(l));
        }
        for (int m = 0; m <= M; ++m)
        {
            qh[m] = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "qh_" + to_string(m));
            qt[m] = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "qt_" + to_string(m));
        }
        for (int n = 0; n <= N; ++n)
        {
            rh[n] = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "rh_" + to_string(n));
            rt[n] = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "rt_" + to_string(n));
        }

        // Create variables: x_lm, y_nm (allocation indicator)
        GRBVar x[L + 1][N + 1], y[M + 1][N + 1];
        for (int l = 0; l <= L; ++l)
        {
            for (int m = 0; m <= M; ++m)
            {
                x[l][m] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_" + to_string(l) + "_" + to_string(m));
            }
        }
        for (int n = 0; n <= N; ++n)
        {
            for (int m = 0; m <= M; ++m)
            {
                y[n][m] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y_" + to_string(n) + "_" + to_string(m));
            }
        }

        // Create variables: f (T_last)
        GRBVar f = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "f");

        // Set objective: minimize f
        // model.setObjective(f + 0, GRB_MINIMIZE);

        // Set objective: minimize sigma(entering time - arrival time)
        GRBLinExpr obj = 0;
        for (int i = 1; i <= alpha; ++i)
            obj += (s[i] - A[i]);
        for (int j = 1; j <= beta; ++j)
            obj += (t[j] - B[j]);
        for (int k = 1; k <= gamma; ++k)
            obj += (u[k] - C[k]);
        model.setObjective(obj, GRB_MINIMIZE);

        // Add constraint: scheduled entering time >= earliest arrival time
        for (int i = 0; i <= alpha; ++i)
        {
            model.addConstr(s[i] >= A[i], "c0_" + to_string(i));
        }
        for (int j = 0; j <= beta; ++j)
        {
            model.addConstr(t[j] >= B[j], "c1_" + to_string(j));
        }
        for (int k = 0; k <= gamma; ++k)
        {
            model.addConstr(u[k] >= C[k], "c2_" + to_string(k));
        }

        // Add constraint: waiting time if from the same lane
        for (int i = 1; i < alpha; ++i)
        {
            model.addConstr(s[i + 1] - s[i] >= W_same, "c3_" + to_string(i));
        }
        for (int j = 1; j < beta; ++j)
        {
            model.addConstr(t[j + 1] - t[j] >= W_same, "c4_" + to_string(j));
        }
        for (int k = 1; k < gamma; ++k)
        {
            model.addConstr(u[k + 1] - u[k] >= W_same, "c5_" + to_string(k));
        }

        // Add constraint: the head and the tail of each group
        for (int l = 1; l <= L; ++l)
        {
            model.addConstr(ph[l] == s[grouped_A[l].first], "c6_" + to_string(l));
            model.addConstr(pt[l] == s[grouped_A[l].second], "c7_" + to_string(l));
        }
        for (int m = 1; m <= M; ++m)
        {
            model.addConstr(qh[m] == t[grouped_B[m].first], "c8_" + to_string(m));
            model.addConstr(qt[m] == t[grouped_B[m].second], "c9_" + to_string(m));
        }
        for (int n = 1; n <= N; ++n)
        {
            model.addConstr(rh[n] == u[grouped_C[n].first], "c10_" + to_string(n));
            model.addConstr(rt[n] == u[grouped_C[n].second], "c11_" + to_string(n));
        }

        // Add constraint: q is put into only one gap
        for (int m = 1; m <= M; ++m)
        {
            GRBLinExpr sigma = 0;
            for (int l = 0; l <= L; ++l)
                sigma += x[l][m];
            for (int n = 0; n <= N; ++n)
                sigma += y[n][m];
            model.addConstr(sigma == 1, "c12_" + to_string(m));
        }

        // Add constraint: q_0 is not put into any gap
        GRBLinExpr sigma = 0;
        for (int l = 0; l <= L; ++l)
            sigma += x[l][0];
        for (int n = 0; n <= N; ++n)
            sigma += y[n][0];
        model.addConstr(sigma == 0, "c13");

        // Add constraint: if x[l][m] == 1, then p[l+1] - W_diff >= q[m] >= p[l] + W_diff
        for (int m = 0; m <= M; ++m)
        {
            for (int l = 0; l < L; ++l)
            {
                model.addGenConstrIndicator(x[l][m], 1, qt[m] <= ph[l + 1] - W_diff, "c14_" + to_string(l) + "_" + to_string(m));
                model.addGenConstrIndicator(x[l][m], 1, qh[m] >= pt[l] + W_diff, "c15_" + to_string(l) + "_" + to_string(m));
            }
            model.addGenConstrIndicator(x[L][m], 1, qh[m] >= pt[L] + W_diff, "c16_" + to_string(L) + "_" + to_string(m));
        }

        // Add constraint: if y[n][m] == 1, then r[n+1] - W_diff >= q[m] >= r[n] + W_diff
        for (int m = 0; m <= M; ++m)
        {
            for (int n = 0; n < N; ++n)
            {
                model.addGenConstrIndicator(y[n][m], 1, qt[m] <= rh[n + 1] - W_diff, "c17_" + to_string(n) + "_" + to_string(m));
                model.addGenConstrIndicator(y[n][m], 1, qh[m] >= rt[n] + W_diff, "c18_" + to_string(n) + "_" + to_string(m));
            }
            model.addGenConstrIndicator(y[N][m], 1, qh[m] >= rt[N] + W_diff, "c19_" + to_string(N) + "_" + to_string(m));
        }

        // Add constraint: delay <= D
        // for (int i = 0; i <= alpha; ++i)
        // {
        //     model.addConstr(s[i] - A[i] <= D, "c12_" + to_string(i));
        // }
        // for (int k = 0; k <= gamma; ++k)
        // {
        //     model.addConstr(u[k] - C[k] <= D, "c13_" + to_string(k));
        // }
        // for (int j = 0; j <= beta; ++j)
        // {
        //     model.addConstr(t[j] - B[j] <= D, "c14_" + to_string(j));
        // }

        // Add constraint: f = max(a[alpha], b[beta], c[gamma])
        // model.addConstr(f >= s[alpha], "c20");
        // model.addConstr(f >= t[beta], "c21");
        // model.addConstr(f >= u[gamma], "c22");
        cout << "Optimizing ..." << endl;
        // Optimize model
        model.optimize();

        // Output results
        cout << "s = {";
        for (int i = 0; i <= alpha; ++i)
        {
            // cout << s[i].get(GRB_StringAttr_VarName) << " "
            //      << s[i].get(GRB_DoubleAttr_X) << endl;
            cout << s[i].get(GRB_DoubleAttr_X) << ", ";
        }
        cout << "};" << endl;
        cout << "t = {";
        for (int j = 0; j <= beta; ++j)
        {
            // cout << t[j].get(GRB_StringAttr_VarName) << " "
            //      << t[j].get(GRB_DoubleAttr_X) << endl;
            cout << t[j].get(GRB_DoubleAttr_X) << ", ";
        }
        cout << "};" << endl;
        cout << "u = {";
        for (int k = 0; k <= gamma; ++k)
        {
            // cout << u[k].get(GRB_StringAttr_VarName) << " "
            //      << u[k].get(GRB_DoubleAttr_X) << endl;
            cout << u[k].get(GRB_DoubleAttr_X) << ", ";
        }
        cout << "};" << endl;
        // for (int m = 0; m <= M; ++m)
        // {
        //     for (int l = 0; l <= L; ++l)
        //     {
        //         if (x[l][m].get(GRB_DoubleAttr_X) == 1)
        //             // cout << x[i][j].get(GRB_StringAttr_VarName) << " "
        //             //      << x[i][j].get(GRB_DoubleAttr_X) << endl;
        //             cout << "X"
        //                  << " ";
        //     }
        //     for (int n = 0; n <= N; ++n)
        //     {
        //         if (y[n][m].get(GRB_DoubleAttr_X) == 1)
        //             // cout << y[k][j].get(GRB_StringAttr_VarName) << " "
        //             //      << y[k][j].get(GRB_DoubleAttr_X) << endl;
        //             cout << "Y"
        //                  << " ";
        //     }
        // }
        cout << endl;
        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
        cout << "T_last: " << max(max(s[alpha].get(GRB_DoubleAttr_X), t[beta].get(GRB_DoubleAttr_X)), max(t[beta].get(GRB_DoubleAttr_X), u[gamma].get(GRB_DoubleAttr_X))) << endl;
    }
    catch (GRBException e)
    {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    catch (...)
    {
        cout << "Exception during optimization" << endl;
    }
}

int main(int argc, char *argv[])
{
    ios::sync_with_stdio(false);
    cin.tie(0);

    float timeStep = 1;
    float W_same, W_diff;
    int alpha, beta, gamma;
    float p, pA, pB, pC;
    vector<float> A, B, C;
    vector<pair<int, int>> grouped_A, grouped_B, grouped_C;

    if (argc == 5)
    {
        W_same = atof(argv[3]);
        W_diff = atof(argv[4]);
        alpha = atoi(argv[2]);
        beta = atoi(argv[2]);
        gamma = atoi(argv[2]);
        p = atof(argv[1]);
        pA = p / 3;
        pB = p / 3;
        pC = p / 3;
    }
    else
    {
        cout << "Arguments: lambda, N, W=, W+" << endl;
        return 0;
    }

    A = generate_traffic(timeStep, alpha, p, 0);
    B = generate_traffic(timeStep, beta, p, 1);
    C = generate_traffic(timeStep, gamma, p, 2);
    grouped_A = grouping_v1(A, timeStep);
    grouped_B = grouping_v1(B, timeStep);
    grouped_C = grouping_v1(C, timeStep);
    solve(A, B, C, W_same, W_diff, grouped_A, grouped_B, grouped_C);

    cout << "A = {" << A[0];
    for (int i = 1; i < A.size(); ++i)
        cout << ", " << A[i];
    cout << "};" << endl;
    cout << "B = {" << B[0];
    for (int i = 1; i < B.size(); ++i)
        cout << ", " << B[i];
    cout << "};" << endl;
    cout << "C = {" << C[0];
    for (int i = 1; i < C.size(); ++i)
        cout << ", " << C[i];
    cout << "};" << endl;

    for (auto &g : grouped_A)
        cout << "(" << g.first << ", " << g.second << ")"
             << " ";
    cout << endl;
    for (auto &g : grouped_B)
        cout << "(" << g.first << ", " << g.second << ")"
             << " ";
    cout << endl;
    for (auto &g : grouped_C)
        cout << "(" << g.first << ", " << g.second << ")"
             << " ";
    cout << endl;

    return 0;
}
