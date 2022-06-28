// Compilation: g++ -std=c++11 group_milp.cpp
//                  -I /Library/gurobi912/mac64/include
//                  -L /Library/gurobi912/mac64/lib
//                  -lgurobi_c++ -lgurobi91
#include "group_milp.h"

tuple<double, double, double> solve_group_milp(vector<double> A, vector<double> B, vector<double> C)
{
    auto t0 = chrono::high_resolution_clock::now();
    vector<pair<int, int>> grouped_A = grouping(A);
    vector<pair<int, int>> grouped_B = grouping(B);
    vector<pair<int, int>> grouped_C = grouping(C);
    int alpha = A.size() - 1;
    int beta = B.size() - 1;
    int gamma = C.size() - 1;
    // double D = 30;
    int L = grouped_A.size() - 1;
    int M = grouped_B.size() - 1;
    int N = grouped_C.size() - 1;

    try
    {
        // Create an environment
        GRBEnv env = GRBEnv(true);
        // env.set("LogFile", "milp.log");
        env.set("OutputFlag", "0");
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);
        model.set("TimeLimit", "1800.0");

        // Create variables: s_i, t_j, u_k (scheduled entering time)
        GRBVar s[alpha + 1], t[beta + 1], u[gamma + 1];
        for (int i = 0; i <= alpha; ++i)
        {
            s[i] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "s_" + to_string(i));
        }
        for (int j = 0; j <= beta; ++j)
        {
            t[j] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "t_" + to_string(j));
        }
        for (int k = 0; k <= gamma; ++k)
        {
            u[k] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "u_" + to_string(k));
        }

        // Create variables: ph_l, pt_l, qh_m, qt_m, rh_n, rt_n (scheduled entering time of groups)
        GRBVar ph[L + 1], pt[L + 1], qh[M + 1], qt[M + 1], rh[N + 1], rt[N + 1];
        for (int l = 0; l <= L; ++l)
        {
            ph[l] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "ph_" + to_string(l));
            pt[l] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "pt_" + to_string(l));
        }
        for (int m = 0; m <= M; ++m)
        {
            qh[m] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "qh_" + to_string(m));
            qt[m] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "qt_" + to_string(m));
        }
        for (int n = 0; n <= N; ++n)
        {
            rh[n] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "rh_" + to_string(n));
            rt[n] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "rt_" + to_string(n));
        }

        // Create variables: x_lm, y_nm (allocation indicator)
        GRBVar x[L + 1][M + 1], y[N + 1][M + 1];
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
        GRBVar f = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "f");

        // Set objective: minimize f
        model.setObjective(f + 0, GRB_MINIMIZE);

        // Set objective: minimize weighted sum
        // GRBLinExpr weightedSum = f * (alpha + beta + gamma);
        // for (int i = 1; i <= alpha; ++i)
        //     weightedSum += s[i];
        // for (int j = 1; j <= beta; ++j)
        //     weightedSum += t[j];
        // for (int k = 1; k <= gamma; ++k)
        //     weightedSum += u[k];
        // model.setObjective(weightedSum, GRB_MINIMIZE);

        // Set objective: minimize sigma(entering time - arrival time)
        // GRBLinExpr obj = 0;
        // for (int i = 1; i <= alpha; ++i)
        //     obj += (s[i] - A[i]);
        // for (int j = 1; j <= beta; ++j)
        //     obj += (t[j] - B[j]);
        // for (int k = 1; k <= gamma; ++k)
        //     obj += (u[k] - C[k]);
        // model.setObjective(obj, GRB_MINIMIZE);

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
        for (int l = 0; l <= L; ++l)
        {
            model.addConstr(ph[l] == s[grouped_A[l].first], "c6_" + to_string(l));
            model.addConstr(pt[l] == s[grouped_A[l].second], "c7_" + to_string(l));
        }
        for (int m = 0; m <= M; ++m)
        {
            model.addConstr(qh[m] == t[grouped_B[m].first], "c8_" + to_string(m));
            model.addConstr(qt[m] == t[grouped_B[m].second], "c9_" + to_string(m));
        }
        for (int n = 0; n <= N; ++n)
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
            model.addGenConstrIndicator(x[0][m], 1, qt[m] <= ph[1] - W_diff, "c14_" + to_string(0) + "_" + to_string(m));
            for (int l = 1; l < L; ++l)
            {
                model.addGenConstrIndicator(x[l][m], 1, qt[m] <= ph[l + 1] - W_diff, "c14_" + to_string(l) + "_" + to_string(m));
                model.addGenConstrIndicator(x[l][m], 1, qh[m] >= pt[l] + W_diff, "c15_" + to_string(l) + "_" + to_string(m));
            }
            model.addGenConstrIndicator(x[L][m], 1, qh[m] >= pt[L] + W_diff, "c16_" + to_string(L) + "_" + to_string(m));
        }

        // Add constraint: if y[n][m] == 1, then r[n+1] - W_diff >= q[m] >= r[n] + W_diff
        for (int m = 0; m <= M; ++m)
        {
            model.addGenConstrIndicator(y[0][m], 1, qt[m] <= rh[1] - W_diff, "c17_" + to_string(0) + "_" + to_string(m));
            for (int n = 1; n < N; ++n)
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
        model.addConstr(f >= s[alpha], "c20");
        model.addConstr(f >= t[beta], "c21");
        model.addConstr(f >= u[gamma], "c22");

        // Optimize model
        model.optimize();

        // Output results
        double total_wait = 0;
        // cout << "s = {";
        for (int i = 0; i <= alpha; ++i)
        {
            // cout << s[i].get(GRB_DoubleAttr_X) << ", ";
            total_wait += (s[i].get(GRB_DoubleAttr_X) - A[i]);
        }
        // cout << "};" << endl;
        // cout << "t = {";
        for (int j = 0; j <= beta; ++j)
        {
            // cout << t[j].get(GRB_DoubleAttr_X) << ", ";
            total_wait += (t[j].get(GRB_DoubleAttr_X) - B[j]);
        }
        // cout << "};" << endl;
        // cout << "k = {";
        for (int k = 0; k <= gamma; ++k)
        {
            // cout << u[k].get(GRB_DoubleAttr_X) << ", ";
            total_wait += (u[k].get(GRB_DoubleAttr_X) - C[k]);
        }
        // cout << "};" << endl;
        // for (int m = 0; m <= M; ++m)
        // {
        //     for (int l = 0; l <= L; ++l)
        //     {
        //         if (x[l][m].get(GRB_DoubleAttr_X))
        //             // cout << x[l][m].get(GRB_StringAttr_VarName) << " "
        //             //      << x[l][m].get(GRB_DoubleAttr_X) << endl;
        //             cout << "X"
        //                  << " ";
        //     }
        //     for (int n = 0; n <= N; ++n)
        //     {
        //         if (y[n][m].get(GRB_DoubleAttr_X))
        //             // cout << y[n][m].get(GRB_StringAttr_VarName) << " "
        //             //      << y[n][m].get(GRB_DoubleAttr_X) << endl;
        //             cout << "Y"
        //                  << " ";
        //     }
        // }
        // cout << endl;
        // cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
        // cout << "T_last: " << max(max(s[alpha].get(GRB_DoubleAttr_X), t[beta].get(GRB_DoubleAttr_X)), max(t[beta].get(GRB_DoubleAttr_X), u[gamma].get(GRB_DoubleAttr_X))) << endl;
        // double T_last = model.get(GRB_DoubleAttr_ObjVal);
        double T_last = f.get(GRB_DoubleAttr_X);
        double T_delay = total_wait / (alpha + beta + gamma);
        auto t1 = chrono::high_resolution_clock::now();
        double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
        totalComputeTime *= 1e-9;
        // cout << "milp_group result: " << T_last << " " << T_delay << " " << totalComputeTime << endl;
        return {T_last, T_delay, totalComputeTime};
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
    return {-1, -1, -1};
}
