#include "milp.h"

void milp_compute_entering_time(vector<vehicle> &A, vector<vehicle> &B, vector<vehicle> &C, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    int alpha = A.size() - 1;
    int beta = B.size() - 1;
    int gamma = C.size() - 1;
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

        // Create variables: x_ij, y_kj (allocation indicator)
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
        // model.setObjective(f + 0, GRB_MINIMIZE);

        // Set objective: minimize weighted sum
        GRBLinExpr weightedSum = f * (alpha + beta + gamma);
        for (int i = 1; i <= alpha; ++i)
            weightedSum += s[i];
        for (int j = 1; j <= beta; ++j)
            weightedSum += t[j];
        for (int k = 1; k <= gamma; ++k)
            weightedSum += u[k];
        model.setObjective(weightedSum, GRB_MINIMIZE);

        // Add constraint: scheduled entering time >= earliest arrival time
        for (int i = 0; i <= alpha; ++i)
        {
            model.addConstr(s[i] >= A[i].time, "c0_" + to_string(i));
            // cout << "A[" << i << "] = " << A[i].time << endl;
        }
        for (int j = 0; j <= beta; ++j)
        {
            model.addConstr(t[j] >= B[j].time, "c1_" + to_string(j));
            // cout << "B[" << j << "] = " << B[j].time << endl;
        }
        for (int k = 0; k <= gamma; ++k)
        {
            model.addConstr(u[k] >= C[k].time, "c2_" + to_string(k));
            // cout << "C[" << k << "] = " << C[k].time << endl;
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

        // Add constraint: B_j is put into only one gap
        for (int j = 1; j <= beta; ++j)
        {
            GRBLinExpr sigma = 0;
            for (int i = 0; i <= alpha; ++i)
                sigma += x[i][j];
            for (int k = 0; k <= gamma; ++k)
                sigma += y[k][j];
            model.addConstr(sigma == 1, "c6_" + to_string(j));
        }

        // Add constraint: B_0 is not put into any gap
        GRBLinExpr sigma = 0;
        for (int i = 0; i <= alpha; ++i)
            sigma += x[i][0];
        for (int k = 0; k <= gamma; ++k)
            sigma += y[k][0];
        model.addConstr(sigma == 0, "c7");

        // Add constraint: if x[i][j] == 1, then a[i+1] - W_diff >= b[j] >= a[i] + W_diff
        for (int j = 0; j <= beta; ++j)
        {
            model.addGenConstrIndicator(x[0][j], 1, t[j] <= s[1] - W_diff, "c8_" + to_string(0) + "_" + to_string(j));
            for (int i = 1; i < alpha; ++i)
            {
                model.addGenConstrIndicator(x[i][j], 1, t[j] <= s[i + 1] - W_diff, "c8_" + to_string(i) + "_" + to_string(j));
                model.addGenConstrIndicator(x[i][j], 1, t[j] >= s[i] + W_diff, "c9_" + to_string(i) + "_" + to_string(j));
            }
            model.addGenConstrIndicator(x[alpha][j], 1, t[j] >= s[alpha] + W_diff, "c9_" + to_string(alpha) + "_" + to_string(j));
        }

        // Add constraint: if y[k][j] == 1, then c[k+1] - W_diff >= b[j] >= c[k] + W_diff
        for (int j = 0; j <= beta; ++j)
        {
            model.addGenConstrIndicator(y[0][j], 1, t[j] <= u[1] - W_diff, "c10_" + to_string(0) + "_" + to_string(j));
            for (int k = 1; k < gamma; ++k)
            {
                model.addGenConstrIndicator(y[k][j], 1, t[j] <= u[k + 1] - W_diff, "c10_" + to_string(k) + "_" + to_string(j));
                model.addGenConstrIndicator(y[k][j], 1, t[j] >= u[k] + W_diff, "c11_" + to_string(k) + "_" + to_string(j));
            }
            model.addGenConstrIndicator(y[gamma][j], 1, t[j] >= u[gamma] + W_diff, "c11_" + to_string(gamma) + "_" + to_string(j));
        }

        // Add constraint: f = max(a[alpha], b[beta], c[gamma])
        model.addConstr(f >= s[alpha], "c15");
        model.addConstr(f >= t[beta], "c16");
        model.addConstr(f >= u[gamma], "c17");

        // Optimize model
        model.optimize();

        // Output results
        double total_wait = 0;
        schedule_A.clear();
        schedule_B.clear();
        schedule_C.clear();
        for (int i = 1; i <= alpha; ++i)
        {
            total_wait += (s[i].get(GRB_DoubleAttr_X) - A[i].time);
            schedule_A.push_back(vehicle(A[i].id, s[i].get(GRB_DoubleAttr_X)));
        }
        for (int j = 1; j <= beta; ++j)
        {
            total_wait += (t[j].get(GRB_DoubleAttr_X) - B[j].time);
            bool isFound = false;
            for (int i = 0; i <= alpha; ++i)
            {
                if (x[i][j].get(GRB_DoubleAttr_X))
                {
                    Vehicle::setRouteID(B[j].id, "route_1");
                    schedule_B.push_back(vehicle(B[j].id, t[j].get(GRB_DoubleAttr_X)));
                    isFound = true;
                    break;
                }
            }
            if (!isFound)
            {
                for (int k = 0; k <= gamma; ++k)
                {
                    if (y[k][j].get(GRB_DoubleAttr_X))
                    {
                        Vehicle::setRouteID(B[j].id, "route_2");
                        schedule_B.push_back(vehicle(B[j].id, t[j].get(GRB_DoubleAttr_X)));
                        break;
                    }
                }
            }
        }
        for (int k = 1; k <= gamma; ++k)
        {
            total_wait += (u[k].get(GRB_DoubleAttr_X) - C[k].time);
            schedule_C.push_back(vehicle(C[k].id, u[k].get(GRB_DoubleAttr_X)));
        }

        // cout << "s = {";
        // for (int i = 0; i <= alpha; ++i)
        // {
        //     cout << s[i].get(GRB_DoubleAttr_X) << ", ";
        // }
        // cout << "};" << endl;
        // cout << "t = {";
        // for (int j = 0; j <= beta; ++j)
        // {
        //     cout << t[j].get(GRB_DoubleAttr_X) << ", ";
        // }
        // cout << "};" << endl;
        // cout << "k = {";
        // for (int k = 0; k <= gamma; ++k)
        // {
        //     cout << u[k].get(GRB_DoubleAttr_X) << ", ";
        // }
        // cout << "};" << endl;
        // for (int j = 1; j <= beta; ++j)
        // {
        //     for (int i = 0; i <= alpha; ++i)
        //     {
        //         if (x[i][j].get(GRB_DoubleAttr_X) == 1)
        //             // cout << x[i][j].get(GRB_StringAttr_VarName) << " "
        //             //      << x[i][j].get(GRB_DoubleAttr_X) << endl;
        //             cout << "X ";
        //     }
        //     for (int k = 0; k <= gamma; ++k)
        //     {
        //         if (y[k][j].get(GRB_DoubleAttr_X) == 1)
        //             // cout << y[k][j].get(GRB_StringAttr_VarName) << " "
        //             //      << y[k][j].get(GRB_DoubleAttr_X) << endl;
        //             cout << "Y ";
        //     }
        // }

        double T_last = model.get(GRB_DoubleAttr_ObjVal);
        double T_delay = total_wait / (alpha + beta + gamma);
        if (model.get(GRB_DoubleAttr_Runtime) >= 1800)
            cout << "*** Exceeds the time limit. ***" << endl;
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
