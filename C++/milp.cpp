#include <bits/stdc++.h>
#include "gurobi_c++.h"
#define endl '\n'
using namespace std;

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

void solve(vector<float> A, vector<float> B, vector<float> C, float W_same, float W_diff)
{
    int alpha = A.size() - 1;
    int beta = B.size() - 1;
    int gamma = C.size() - 1;
    float D = 30;

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
        GRBVar f = model.addVar(0.0, INFINITY, 0.0, GRB_INTEGER, "f");

        // Set objective: minimize f
        model.setObjective(f + 0, GRB_MINIMIZE);

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
            for (int i = 0; i < alpha; ++i)
            {
                model.addGenConstrIndicator(x[i][j], 1, t[j] <= s[i + 1] - W_diff, "c8_" + to_string(i) + "_" + to_string(j));
                model.addGenConstrIndicator(x[i][j], 1, t[j] >= s[i] + W_diff, "c9_" + to_string(i) + "_" + to_string(j));
            }
            model.addGenConstrIndicator(x[alpha][j], 1, t[j] >= s[alpha] + W_diff, "c9_" + to_string(alpha) + "_" + to_string(j));
        }

        // Add constraint: if y[k][j] == 1, then c[k+1] - W_diff >= b[j] >= c[k] + W_diff
        for (int j = 0; j <= beta; ++j)
        {
            for (int k = 0; k < gamma; ++k)
            {
                model.addGenConstrIndicator(y[k][j], 1, t[j] <= u[k + 1] - W_diff, "c10_" + to_string(k) + "_" + to_string(j));
                model.addGenConstrIndicator(y[k][j], 1, t[j] >= u[k] + W_diff, "c11_" + to_string(k) + "_" + to_string(j));
            }
            model.addGenConstrIndicator(y[gamma][j], 1, t[j] >= u[gamma] + W_diff, "c11_" + to_string(gamma) + "_" + to_string(j));
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
        model.addConstr(f >= s[alpha], "c15");
        model.addConstr(f >= t[beta], "c16");
        model.addConstr(f >= u[gamma], "c17");

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
        cout << "k = {";
        for (int k = 0; k <= gamma; ++k)
        {
            // cout << u[k].get(GRB_StringAttr_VarName) << " "
            //      << u[k].get(GRB_DoubleAttr_X) << endl;
            cout << u[k].get(GRB_DoubleAttr_X) << ", ";
        }
        cout << "};" << endl;
        for (int j = 0; j <= beta; ++j)
        {
            for (int i = 0; i <= alpha; ++i)
            {
                if (x[i][j].get(GRB_DoubleAttr_X) == 1)
                    // cout << x[i][j].get(GRB_StringAttr_VarName) << " "
                    //      << x[i][j].get(GRB_DoubleAttr_X) << endl;
                    cout << "X"
                         << " ";
            }
            for (int k = 0; k <= gamma; ++k)
            {
                if (y[k][j].get(GRB_DoubleAttr_X) == 1)
                    // cout << y[k][j].get(GRB_StringAttr_VarName) << " "
                    //      << y[k][j].get(GRB_DoubleAttr_X) << endl;
                    cout << "Y"
                         << " ";
            }
        }
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
    solve(A, B, C, W_same, W_diff);

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

    return 0;
}
