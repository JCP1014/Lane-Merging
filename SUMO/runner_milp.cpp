// Environment Variable:
//     export SUMO_HOME="/Users/jcp/sumo"
// Compilation:
//     g++ -o milp -std=c++11 -I$SUMO_HOME/src runner_milp.cpp -L$SUMO_HOME/bin -ltracicpp
// Run:
//     LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <time.h>
#include <libsumo/libtraci.h>
#include "gurobi_c++.h"

using namespace std;
using namespace libtraci;

struct ListNode
{
    int val;
    ListNode *next;
    ListNode() : val(0), next(nullptr) {}
    ListNode(int x) : val(x), next(nullptr) {}
    ListNode(int x, ListNode *next) : val(x), next(next) {}
};

struct vehicle
{
    string id;
    double time;
    vehicle() : id(""), time(0) {}
    vehicle(string s, double d) : id(s), time(d) {}
};

struct find_id
{
    string target;
    find_id(string id) : target(id){};
    bool operator()(const vehicle &v) const
    {
        return v.id == target;
    }
};

bool sort_id(vehicle a, vehicle b)
{
    int a_id = stoi(a.id.substr(a.id.find("_") + 1));
    int b_id = stoi(b.id.substr(b.id.find("_") + 1));
    return a_id < b_id;
}

void generate_routefile(double timeStep, int N, double pA, double pB, double pC)
{
    ofstream file;
    int num_A = 1, num_B = 1, num_C = 1;
    double t = 1.0;
    default_random_engine generator(time(NULL));
    uniform_real_distribution<double> unif(0.0, 1.0);

    file.open("./sumo_data/laneMerging.rou.xml");
    file << "<routes>" << endl;
    file << "    <vType id=\"typeA\" type=\"passenger\" length=\"2\" accel=\"1.5\" decel=\"2\" sigma=\"0.0\" maxSpeed=\"20\" color=\"yellow\"/>" << endl;
    file << "    <vType id=\"typeB\" type=\"passenger\" length=\"2\" accel=\"1.5\" decel=\"2\" sigma=\"0.0\" maxSpeed=\"20\" color=\"blue\"/>" << endl;
    file << "    <vType id=\"typeC\" type=\"passenger\" length=\"2\" accel=\"1.5\" decel=\"2\" sigma=\"0.0\" maxSpeed=\"20\" color=\"magenta\"/>" << endl;
    file << "    <route edges=\"E6 E9\" color=\"yellow\" id=\"route_0\"/>" << endl;
    file << "    <route edges=\"E7 E9\" color=\"yellow\" id=\"route_1\"/>" << endl;
    file << "    <route edges=\"E7 E10\" color=\"yellow\" id=\"route_2\"/>" << endl;
    file << "    <route edges=\"E8 E10\" color=\"yellow\" id=\"route_3\"/>" << endl;

    while (num_A <= N || num_B <= N || num_C <= N)
    {
        if (num_A <= N && unif(generator) < pA)
            file << "    <vehicle id=\"A_" << num_A++ << "\" type=\"typeA\" route=\"route_0\" depart=\"" << t << "\" departLane=\"0\" departSpeed=\"random\"/>" << endl;
        if (num_B <= N && unif(generator) < pB)
            file << "    <vehicle id=\"B_" << num_B++ << "\" type=\"typeB\" route=\"route_1\" depart=\"" << t << "\" departLane=\"0\" departSpeed=\"random\"/>" << endl;
        if (num_C <= N && unif(generator) < pC)
            file << "    <vehicle id=\"C_" << num_C++ << "\" type=\"typeC\" route=\"route_3\" depart=\"" << t << "\" departLane=\"0\" departSpeed=\"random\"/>" << endl;
        t += timeStep;
    }
    file << "</routes>" << endl;
    file.close();
}

vector<vehicle> compute_earliest_arrival(int laneLength, vector<vector<vehicle>> &schedules, char lane)
{
    vector<vehicle> arrival_times = {vehicle()};
    double currentTime = Simulation::getTime();
    vector<string> vehicle_ids;

    if (lane == 'A')
        vehicle_ids = LaneArea::getLastStepVehicleIDs("dA");
    else if (lane == 'B')
        vehicle_ids = LaneArea::getLastStepVehicleIDs("dB");
    else if (lane == 'C')
        vehicle_ids = LaneArea::getLastStepVehicleIDs("dC");

    for (auto &vehID : vehicle_ids)
    {
        double dist = laneLength - Vehicle::getDistance(vehID);
        double speed = Vehicle::getSpeed(vehID);
        double arrivalTime = 0;
        if (speed == 0)
        {
            for (auto schedule : schedules)
            {
                vector<vehicle>::iterator it = find_if(schedule.begin(), schedule.end(), find_id(vehID));
                if (it != schedule.end())
                    arrivalTime = it->time;
            }
        }
        else
        {
            arrivalTime = currentTime + dist / speed;
            for (auto schedule : schedules)
            {
                vector<vehicle>::iterator it = find_if(schedule.begin(), schedule.end(), find_id(vehID));
                if (it != schedule.end())
                    arrivalTime = min(arrivalTime, it->time);
            }
        }
        arrival_times.push_back(vehicle(vehID, arrivalTime));
        sort(arrival_times.begin() + 1, arrival_times.end(), sort_id);
    }
    return arrival_times;
}

void milp_compute_entering_time(vector<vehicle> &A, vector<vehicle> &B, vector<vehicle> &C, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C, double W_same, double W_diff)
{
    int alpha = A.size() - 1;
    int beta = B.size() - 1;
    int gamma = C.size() - 1;
    try
    {
        // Create an environment
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "milp.log");
        env.set("OutputFlag", "0");
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);
        model.set("TimeLimit", "1800.0");

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

        // Add constraint: scheduled entering time >= earliest arrival time
        for (int i = 0; i <= alpha; ++i)
        {
            model.addConstr(s[i] >= A[i].time, "c0_" + to_string(i));
        }
        for (int j = 0; j <= beta; ++j)
        {
            model.addConstr(t[j] >= B[j].time, "c1_" + to_string(j));
        }
        for (int k = 0; k <= gamma; ++k)
        {
            model.addConstr(u[k] >= C[k].time, "c2_" + to_string(k));
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

        // Add constraint: f = max(a[alpha], b[beta], c[gamma])
        model.addConstr(f >= s[alpha], "c15");
        model.addConstr(f >= t[beta], "c16");
        model.addConstr(f >= u[gamma], "c17");

        // Optimize model
        model.optimize();

        // Output results
        float total_wait = 0;
        schedule_A.clear();
        schedule_B.clear();
        schedule_C.clear();
        // cout << "s = {";
        for (int i = 0; i <= alpha; ++i)
        {
            // cout << s[i].get(GRB_DoubleAttr_X) << ", ";
            total_wait += (s[i].get(GRB_DoubleAttr_X) - A[i].time);
            schedule_A.push_back(vehicle(A[i].id, s[i].get(GRB_DoubleAttr_X)));
        }
        // cout << "};" << endl;
        // cout << "t = {";
        for (int j = 0; j <= beta; ++j)
        {
            // cout << t[j].get(GRB_DoubleAttr_X) << ", ";
            total_wait += (t[j].get(GRB_DoubleAttr_X) - B[j].time);
            schedule_B.push_back(vehicle(B[j].id, t[j].get(GRB_DoubleAttr_X)));
        }
        // cout << "};" << endl;
        // cout << "k = {";
        for (int k = 0; k <= gamma; ++k)
        {
            // cout << u[k].get(GRB_DoubleAttr_X) << ", ";
            total_wait += (u[k].get(GRB_DoubleAttr_X) - C[k].time);
            schedule_C.push_back(vehicle(C[k].id, u[k].get(GRB_DoubleAttr_X)));
        }
        // cout << "};" << endl;
        // for (int j = 0; j <= beta; ++j)
        // {
        //     for (int i = 0; i <= alpha; ++i)
        //     {
        //         if (x[i][j].get(GRB_DoubleAttr_X) == 1)
        //             // cout << x[i][j].get(GRB_StringAttr_VarName) << " "
        //             //      << x[i][j].get(GRB_DoubleAttr_X) << endl;
        //             cout << "X"
        //                  << " ";
        //     }
        //     for (int k = 0; k <= gamma; ++k)
        //     {
        //         if (y[k][j].get(GRB_DoubleAttr_X) == 1)
        //             // cout << y[k][j].get(GRB_StringAttr_VarName) << " "
        //             //      << y[k][j].get(GRB_DoubleAttr_X) << endl;
        //             cout << "Y"
        //                  << " ";
        //     }
        // }
        // cout << endl;
        // cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
        // cout << "T_last: " << max(max(s[alpha].get(GRB_DoubleAttr_X), t[beta].get(GRB_DoubleAttr_X)), max(t[beta].get(GRB_DoubleAttr_X), u[gamma].get(GRB_DoubleAttr_X))) << endl;
        float T_last = model.get(GRB_DoubleAttr_ObjVal);
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

void run()
{
    while (Simulation::getMinExpectedNumber() > 0)
        Simulation::step();
    Simulation::close();
}

int main(int argc, char *argv[])
{
    double timeStep = 1;
    double p, pA, pB, pC;
    int N, alpha, beta, gamma;
    double W_same, W_diff;
    vector<double> A, B, C;

    if (argc == 5)
    {
        p = atof(argv[1]);
        N = atoi(argv[2]);
        W_same = atof(argv[3]);
        W_diff = atof(argv[4]);
        pA = p / 3;
        pB = p / 3;
        pC = p / 3;
        alpha = N;
        beta = N;
        gamma = N;
    }
    else
    {
        cout << "Arguments: p, N, W=, W+" << endl;
        return 0;
    }

    generate_routefile(timeStep, N, pA, pB, pC);
    Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                       "--tripinfo-output", "sumo_data/tripinfo_dp.xml",
                       "-S",
                       "--no-step-log", "true", "-W", "--duration-log.disable", "true"});

    run();
}