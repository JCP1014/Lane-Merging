// Compilation: g++ -std=c++11 group_milp.cpp
//                  -I /Library/gurobi912/mac64/include
//                  -L /Library/gurobi912/mac64/lib
//                  -lgurobi_c++ -lgurobi91
#include "group_milp.h"

// Schedule by grouping MILP
tuple<double, double, double> solve_group_milp(vector<double> A, vector<double> B, vector<double> C)
{
    auto t0 = chrono::high_resolution_clock::now(); // starting time of computation
    vector<pair<int, int>> grouped_A = grouping(A); // vector of pairs of the starting index and ending index of each group on Lane A
    vector<pair<int, int>> grouped_B = grouping(B); // vector of pairs of the starting index and ending index of each group on Lane B
    vector<pair<int, int>> grouped_C = grouping(C); // vector of pairs of the starting index and ending index of each group on Lane C
    int alpha = A.size() - 1;   // number of vehicles on Lane A
    int beta = B.size() - 1;    // number of vehicles on Lane B
    int gamma = C.size() - 1;   // number of vehicles on Lane C
    int L = grouped_A.size() - 1;   // number of groups on Lane A
    int M = grouped_B.size() - 1;   // number of groups on Lane B
    int N = grouped_C.size() - 1;   // number of groups on Lane C

    try
    {
        GRBEnv env = GRBEnv(true);  // create an environment
        env.set("OutputFlag", "0"); // disable solver output
        env.start();    // start an empty environment

        GRBModel model = GRBModel(env); // create an empty model
        model.set("TimeLimit", "1800.0");   // limit the total time expended (in seconds)

        // Create variables: a_i, b_j, c_k (scheduled entering time)
        GRBVar a[alpha + 1], b[beta + 1], c[gamma + 1];
        for (int i = 0; i <= alpha; ++i)
        {
            a[i] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "a_" + to_string(i));
        }
        for (int j = 0; j <= beta; ++j)
        {
            b[j] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "b_" + to_string(j));
        }
        for (int k = 0; k <= gamma; ++k)
        {
            c[k] = model.addVar(0.0, INFINITY, 0.0, GRB_CONTINUOUS, "c_" + to_string(k));
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

        // Create variables: x_lm, y_nm (insertion indicator)
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

        // Add constraint: scheduled entering time >= earliest arrival time
        for (int i = 0; i <= alpha; ++i)
        {
            model.addConstr(a[i] >= A[i], "c2_" + to_string(i));
        }
        for (int j = 0; j <= beta; ++j)
        {
            model.addConstr(b[j] >= B[j], "c3_" + to_string(j));
        }
        for (int k = 0; k <= gamma; ++k)
        {
            model.addConstr(c[k] >= C[k], "c4_" + to_string(k));
        }

        // Add constraint: waiting time if from the same lane
        for (int i = 1; i < alpha; ++i)
        {
            model.addConstr(a[i + 1] - a[i] >= W_same, "c5_" + to_string(i));
        }
        for (int j = 1; j < beta; ++j)
        {
            model.addConstr(b[j + 1] - b[j] >= W_same, "c6_" + to_string(j));
        }
        for (int k = 1; k < gamma; ++k)
        {
            model.addConstr(c[k + 1] - c[k] >= W_same, "c7_" + to_string(k));
        }

        // Add constraint: the head and the tail of each group
        for (int l = 0; l <= L; ++l)
        {
            model.addConstr(ph[l] == a[grouped_A[l].first], "c14_" + to_string(l));
            model.addConstr(pt[l] == a[grouped_A[l].second], "c15_" + to_string(l));
        }
        for (int m = 0; m <= M; ++m)
        {
            model.addConstr(qh[m] == b[grouped_B[m].first], "c16_" + to_string(m));
            model.addConstr(qt[m] == b[grouped_B[m].second], "c17_" + to_string(m));
        }
        for (int n = 0; n <= N; ++n)
        {
            model.addConstr(rh[n] == c[grouped_C[n].first], "c18_" + to_string(n));
            model.addConstr(rt[n] == c[grouped_C[n].second], "c19_" + to_string(n));
        }

        // Add constraint: q is put into only one gap
        for (int m = 1; m <= M; ++m)
        {
            GRBLinExpr sigma = 0;
            for (int l = 0; l <= L; ++l)
                sigma += x[l][m];
            for (int n = 0; n <= N; ++n)
                sigma += y[n][m];
            model.addConstr(sigma == 1, "c8_" + to_string(m));
        }

        // Add constraint: q_0 is not put into any gap
        GRBLinExpr sigma = 0;
        for (int l = 0; l <= L; ++l)
            sigma += x[l][0];
        for (int n = 0; n <= N; ++n)
            sigma += y[n][0];
        model.addConstr(sigma == 0, "c9");

        // Add constraint: if x[l][m] == 1, then p[l+1] - W_diff >= q[m] >= p[l] + W_diff
        for (int m = 0; m <= M; ++m)
        {
            model.addGenConstrIndicator(x[0][m], 1, qt[m] <= ph[1] - W_diff, "c11_" + to_string(0) + "_" + to_string(m));
            for (int l = 1; l < L; ++l)
            {
                model.addGenConstrIndicator(x[l][m], 1, qt[m] <= ph[l + 1] - W_diff, "c11_" + to_string(l) + "_" + to_string(m));
                model.addGenConstrIndicator(x[l][m], 1, qh[m] >= pt[l] + W_diff, "c10_" + to_string(l) + "_" + to_string(m));
            }
            model.addGenConstrIndicator(x[L][m], 1, qh[m] >= pt[L] + W_diff, "c10_" + to_string(L) + "_" + to_string(m));
        }

        // Add constraint: if y[n][m] == 1, then r[n+1] - W_diff >= q[m] >= r[n] + W_diff
        for (int m = 0; m <= M; ++m)
        {
            model.addGenConstrIndicator(y[0][m], 1, qt[m] <= rh[1] - W_diff, "c13_" + to_string(0) + "_" + to_string(m));
            for (int n = 1; n < N; ++n)
            {
                model.addGenConstrIndicator(y[n][m], 1, qt[m] <= rh[n + 1] - W_diff, "c13_" + to_string(n) + "_" + to_string(m));
                model.addGenConstrIndicator(y[n][m], 1, qh[m] >= rt[n] + W_diff, "c12_" + to_string(n) + "_" + to_string(m));
            }
            model.addGenConstrIndicator(y[N][m], 1, qh[m] >= rt[N] + W_diff, "c12_" + to_string(N) + "_" + to_string(m));
        }

        // Add constraint: f = max(a[alpha], b[beta], c[gamma])
        model.addConstr(f >= a[alpha], "c1_1");
        model.addConstr(f >= b[beta], "c1_2");
        model.addConstr(f >= c[gamma], "c1_3");

        // Optimize model
        model.optimize();

        // Calculate waiting time (difference between scheduled entering time and earliest arrival time)
        double total_wait = 0;
        for (int i = 0; i <= alpha; ++i)
        {
            total_wait += (a[i].get(GRB_DoubleAttr_X) - A[i]);
        }
        for (int j = 0; j <= beta; ++j)
        {
            total_wait += (b[j].get(GRB_DoubleAttr_X) - B[j]);
        }
        for (int k = 0; k <= gamma; ++k)
        {
            total_wait += (c[k].get(GRB_DoubleAttr_X) - C[k]);
        }

        double T_last = f.get(GRB_DoubleAttr_X);    // get the objective value
        double T_delay = total_wait / (alpha + beta + gamma);   // average difference betweeb each vehicle's scheduled entering time and its earliest arrival time
        auto t1 = chrono::high_resolution_clock::now(); // ending time of computation
        double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();  // computation time
        totalComputeTime *= 1e-9;

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
