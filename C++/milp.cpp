// Compilation: g++ -std=c++11 milp.cpp
//                  -I /Library/gurobi912/mac64/include
//                  -L /Library/gurobi912/mac64/lib
//                  -lgurobi_c++ -lgurobi91
#include "milp.h"

// Scheduled by MILP
tuple<double, double, double> solve_milp(vector<double> A, vector<double> B, vector<double> C)
{
    auto t0 = chrono::high_resolution_clock::now(); // starting time of computation
    int alpha = A.size() - 1;   // number of vehicleas on Lane A
    int beta = B.size() - 1;    // number of vehicleas on Lane B
    int gamma = C.size() - 1;   // number of vehicleas on Lane C

    try
    {
        GRBEnv env = GRBEnv(true);  // create an environment
        env.set("OutputFlag", "0"); // disable solver output
        env.start();    // start an empty environment

        GRBModel model = GRBModel(env);  // create an empty model
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

        // Create variables: x_ij, y_kj (insertion indicator)
        GRBVar x[alpha + 1][beta + 1], y[gamma + 1][beta + 1];
        for (int i = 0; i <= alpha; ++i)
        {
            for (int j = 0; j <= beta; ++j)
            {
                x[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_" + to_string(i) + "_" + to_string(j));
            }
        }
        for (int k = 0; k <= gamma; ++k)
        {
            for (int j = 0; j <= beta; ++j)
            {
                y[k][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y_" + to_string(k) + "_" + to_string(j));
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

        // Add constraint: B_j is put into only one gap
        for (int j = 1; j <= beta; ++j)
        {
            GRBLinExpr sigma = 0;
            for (int i = 0; i <= alpha; ++i)
                sigma += x[i][j];
            for (int k = 0; k <= gamma; ++k)
                sigma += y[k][j];
            model.addConstr(sigma == 1, "c8_" + to_string(j));
        }

        // Add constraint: B_0 is not put into any gap
        GRBLinExpr sigma = 0;
        for (int i = 0; i <= alpha; ++i)
            sigma += x[i][0];
        for (int k = 0; k <= gamma; ++k)
            sigma += y[k][0];
        model.addConstr(sigma == 0, "c9");

        // Add constraint: if x[i][j] == 1, then a[i+1] - W_diff >= b[j] >= a[i] + W_diff
        for (int j = 0; j <= beta; ++j)
        {
            model.addGenConstrIndicator(x[0][j], 1, b[j] <= a[1] - W_diff, "c11_" + to_string(0) + "_" + to_string(j));
            for (int i = 1; i < alpha; ++i)
            {
                model.addGenConstrIndicator(x[i][j], 1, b[j] <= a[i + 1] - W_diff, "c11_" + to_string(i) + "_" + to_string(j));
                model.addGenConstrIndicator(x[i][j], 1, b[j] >= a[i] + W_diff, "c10_" + to_string(i) + "_" + to_string(j));
            }
            model.addGenConstrIndicator(x[alpha][j], 1, b[j] >= a[alpha] + W_diff, "c10_" + to_string(alpha) + "_" + to_string(j));
        }

        // Add constraint: if y[k][j] == 1, then c[k+1] - W_diff >= b[j] >= c[k] + W_diff
        for (int j = 0; j <= beta; ++j)
        {
            model.addGenConstrIndicator(y[0][j], 1, b[j] <= c[1] - W_diff, "c13_" + to_string(0) + "_" + to_string(j));
            for (int k = 1; k < gamma; ++k)
            {
                model.addGenConstrIndicator(y[k][j], 1, b[j] <= c[k + 1] - W_diff, "c13_" + to_string(k) + "_" + to_string(j));
                model.addGenConstrIndicator(y[k][j], 1, b[j] >= c[k] + W_diff, "c12_" + to_string(k) + "_" + to_string(j));
            }
            model.addGenConstrIndicator(y[gamma][j], 1, b[j] >= c[gamma] + W_diff, "c12_" + to_string(gamma) + "_" + to_string(j));
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
