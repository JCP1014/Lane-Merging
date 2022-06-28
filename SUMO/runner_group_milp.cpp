// Environment Variable:
//     export SUMO_HOME="/Users/jcp/sumo"
// Compilation:
//     g++ -o group_milp -std=c++11 -I$SUMO_HOME/src runner_group_milp.cpp -L$SUMO_HOME/bin -ltracicpp -I /Library/gurobi912/mac64/include -L /Library/gurobi912/mac64/lib -lgurobi_c++ -lgurobi91
// Run:
//     LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp [density] [number] [W=] [W+] [generateNewTest (T/F)] [inputPath]
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <time.h>
#include <utility>
#include <libsumo/libtraci.h>
#include "gurobi_c++.h"

using namespace std;
using namespace libtraci;

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

bool sort_vehicle(vehicle a, vehicle b)
{
    int a_id = stoi(a.id.substr(a.id.find("_") + 1));
    int b_id = stoi(b.id.substr(b.id.find("_") + 1));
    return a_id < b_id;
}

bool sort_id(string a, string b)
{
    int a_id = stoi(a.substr(a.find("_") + 1));
    int b_id = stoi(b.substr(b.find("_") + 1));
    return a_id < b_id;
}

bool sort_time(vehicle a, vehicle b)
{
    return a.time < b.time;
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
    file << "    <vType id=\"typeA\" type=\"passenger\" length=\"4.5\" accel=\"3\" decel=\"4.5\" sigma=\"0.0\" maxSpeed=\"20\" minGap=\"3\" carFollowModel=\"CACC\" tau=\"1.0\" color=\"yellow\"/>" << endl;
    file << "    <vType id=\"typeB\" type=\"passenger\" length=\"4.5\" accel=\"3\" decel=\"4.5\" sigma=\"0.0\" maxSpeed=\"20\" minGap=\"3\" carFollowModel=\"CACC\" tau=\"1.0\" color=\"blue\"/>" << endl;
    file << "    <vType id=\"typeC\" type=\"passenger\" length=\"4.5\" accel=\"3\" decel=\"4.5\" sigma=\"0.0\" maxSpeed=\"20\" minGap=\"3\" carFollowModel=\"CACC\" tau=\"1.0\" color=\"magenta\"/>" << endl;
    file << "    <route edges=\"incoming X\" color=\"yellow\" id=\"route_0\"/>" << endl;
    file << "    <route edges=\"incoming X\" color=\"yellow\" id=\"route_1\"/>" << endl;
    file << "    <route edges=\"incoming Y\" color=\"yellow\" id=\"route_2\"/>" << endl;
    file << "    <route edges=\"incoming Y\" color=\"yellow\" id=\"route_3\"/>" << endl;

    while (num_A <= N || num_B <= N || num_C <= N)
    {
        if (num_A <= N && unif(generator) < pA)
            file << "    <vehicle id=\"A_" << num_A++ << "\" type=\"typeA\" route=\"route_0\" depart=\"" << t << "\" departLane=\"2\" departSpeed=\"max\"/>" << endl;
        if (num_B <= N && unif(generator) < pB)
            file << "    <vehicle id=\"B_" << num_B++ << "\" type=\"typeB\" route=\"route_1\" depart=\"" << t << "\" departLane=\"1\" departSpeed=\"max\"/>" << endl;
        if (num_C <= N && unif(generator) < pC)
            file << "    <vehicle id=\"C_" << num_C++ << "\" type=\"typeC\" route=\"route_3\" depart=\"" << t << "\" departLane=\"0\" departSpeed=\"max\"/>" << endl;
        t += timeStep;
    }
    file << "</routes>" << endl;
    file.close();
}

void print_schedule(vector<vehicle> schedule, string lane)
{
    cout << "---------- schedule_" << lane << " " << schedule.size() << " ----------" << endl;
    for (auto veh : schedule)
    {
        cout << veh.id << " " << veh.time << " " << Vehicle::getRouteID(veh.id) << endl;
    }
}

vector<vehicle> compute_earliest_arrival(int laneLength, double W_same, vector<vehicle> &schedule, char lane)
{
    vector<vehicle> arrival_times = {vehicle()};
    double currentTime = Simulation::getTime();
    vector<string> vehicle_ids = LaneArea::getLastStepVehicleIDs(string("d") + lane);

    for (auto vehID : vehicle_ids)
    {
        double dist = laneLength - Vehicle::getDistance(vehID);
        double speed = Vehicle::getSpeed(vehID);
        double accel = Vehicle::getAcceleration(vehID);
        double arrivalTime = 0;
        if (speed == 0)
        {
            vector<vehicle>::iterator it = find_if(schedule.begin(), schedule.end(), find_id(vehID));
            if (it != schedule.end())
                arrivalTime = it->time;
        }
        else
        {
            if (accel != 0)
                arrivalTime = currentTime + (sqrt(max(0.0, speed * speed + 2 * accel * dist)) - speed) / accel;
            else
                arrivalTime = currentTime + dist / speed;
            vector<vehicle>::iterator it = find_if(schedule.begin(), schedule.end(), find_id(vehID));
            if (it != schedule.end())
                arrivalTime = min(arrivalTime, it->time);
        }
        arrival_times.push_back(vehicle(vehID, arrivalTime));
    }
    sort(arrival_times.begin() + 1, arrival_times.end(), sort_vehicle);
    for (int i = 1; i < arrival_times.size(); ++i)
        arrival_times[i].time = max(arrival_times[i].time, arrival_times[i - 1].time + W_same);
    return arrival_times;
}

vector<pair<int, int>> grouping(vector<vehicle> &traffic, double timeStep)
{
    int groups = 1;
    int max_groups = 35;
    double grouping_threshold = 1;
    vector<pair<int, int>> grouped_index;

    while (1)
    {
        grouped_index.push_back({0, 0});
        int head = 1, tail = 0;
        for (int i = 2; i < traffic.size(); ++i)
        {
            if (traffic[i].time - traffic[i - 1].time > grouping_threshold)
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
            // cout << "threshold: " << grouping_threshold << ",  number of groups: " << groups << endl;
            break;
        }
    }
    // for (auto g : grouped_index)
    //     cout << g.first << " " << g.second << endl;
    return grouped_index;
}

void group_milp_compute_entering_time(vector<vehicle> &A, vector<vehicle> &B, vector<vehicle> &C, double W_same, double W_diff, double timeStep, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    vector<pair<int, int>> grouped_A = grouping(A, timeStep);
    vector<pair<int, int>> grouped_B = grouping(B, timeStep);
    vector<pair<int, int>> grouped_C = grouping(C, timeStep);
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

        // Add constraint: f = max(a[alpha], b[beta], c[gamma])
        model.addConstr(f >= s[alpha], "c20");
        model.addConstr(f >= t[beta], "c21");
        model.addConstr(f >= u[gamma], "c22");

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
        for (int m = 1; m <= M; ++m)
        {
            bool isFound = false;
            for (int l = 0; l <= L; ++l)
            {
                if (x[l][m].get(GRB_DoubleAttr_X))
                {
                    for (int j = grouped_B[m].first; j <= grouped_B[m].second; ++j)
                    {
                        total_wait += (t[j].get(GRB_DoubleAttr_X) - B[j].time);
                        Vehicle::setRouteID(B[j].id, "route_1");
                        schedule_B.push_back(vehicle(B[j].id, t[j].get(GRB_DoubleAttr_X)));
                    }
                    isFound = true;
                    break;
                }
            }
            if (!isFound)
            {
                for (int n = 0; n <= N; ++n)
                {
                    if (y[n][m].get(GRB_DoubleAttr_X))
                    {
                        for (int j = grouped_B[m].first; j <= grouped_B[m].second; ++j)
                        {
                            total_wait += (t[j].get(GRB_DoubleAttr_X) - B[j].time);
                            Vehicle::setRouteID(B[j].id, "route_2");
                            schedule_B.push_back(vehicle(B[j].id, t[j].get(GRB_DoubleAttr_X)));
                        }
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

void set_headway(vector<vehicle> &schedule)
{
    for (int i = 1; i < schedule.size(); ++i)
        Vehicle::openGap(schedule[i].id, max(1.0, schedule[i].time - schedule[i - 1].time), 0, 1, 1000);
}

void set_headway(vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    vector<vehicle> total_schedule;
    int i;
    total_schedule.insert(total_schedule.end(), schedule_A.begin(), schedule_A.end());
    total_schedule.insert(total_schedule.end(), schedule_B.begin(), schedule_B.end());
    sort(total_schedule.begin(), total_schedule.end(), sort_time);
    i = 0;
    while (i < total_schedule.size())
    {
        if (Vehicle::getRouteID(total_schedule[i].id) == "route_2")
            total_schedule.erase(total_schedule.begin() + i);
        else
            ++i;
    }
    for (i = 1; i < total_schedule.size(); ++i)
    {
        // cout << "SET_HEADWAY (" << total_schedule[i].id << ", " << total_schedule[i].time << ") (" << total_schedule[i - 1].id << ", " << total_schedule[i - 1].time << ")\n";
        Vehicle::openGap(total_schedule[i].id, max(1.0, total_schedule[i].time - total_schedule[i - 1].time), 0, 0.5, 0.5, -1, total_schedule[i - 1].id);
    }
    total_schedule.clear();
    total_schedule.insert(total_schedule.end(), schedule_C.begin(), schedule_C.end());
    total_schedule.insert(total_schedule.end(), schedule_B.begin(), schedule_B.end());
    sort(total_schedule.begin(), total_schedule.end(), sort_time);
    i = 0;
    while (i < total_schedule.size())
    {
        if (Vehicle::getRouteID(total_schedule[i].id) == "route_1")
            total_schedule.erase(total_schedule.begin() + i);
        else
            ++i;
    }
    for (i = 1; i < total_schedule.size(); ++i)
    {
        // cout << "SET_HEADWAY (" << total_schedule[i].id << ", " << total_schedule[i].time << ") (" << total_schedule[i - 1].id << ", " << total_schedule[i - 1].time << ")\n";
        Vehicle::openGap(total_schedule[i].id, max(1.0, total_schedule[i].time - total_schedule[i - 1].time), 0, 0.5, 0.5, -1, total_schedule[i - 1].id);
    }
}

void run(int alpha, int beta, int gamma, double W_same, double W_diff, double timeStep)
{
    int period = 300;
    vector<vehicle> arrival_A, arrival_B, arrival_C;
    vector<vehicle> schedule_A, schedule_B, schedule_C;
    double countdownX = 0, countdownY = 0;
    bool gA = false, gBX = false, gBY = false, gC = false;
    int laneLength = 6500;
    double passTime_dX = 0, passTime_dY = 0;
    vector<string> A_IDs, B_IDs, C_IDs;
    string A_head = "A_1", B_head = "B_1", C_head = "C_1";
    double waitingTime = 0;

    while (Simulation::getMinExpectedNumber() > 0)
    {
        for (auto &vehID : Simulation::getLoadedIDList())
            Vehicle::setLaneChangeMode(vehID, 0b000000000000);
        Simulation::step();
        // cout << Simulation::getTime() << endl;

        // Detect the passing vehicles
        A_IDs = Lane::getLastStepVehicleIDs("incoming_2");
        B_IDs = Lane::getLastStepVehicleIDs("incoming_1");
        C_IDs = Lane::getLastStepVehicleIDs("incoming_0");
        sort(A_IDs.begin(), A_IDs.end(), sort_id);
        sort(B_IDs.begin(), B_IDs.end(), sort_id);
        sort(C_IDs.begin(), C_IDs.end(), sort_id);
        if (!A_IDs.empty() && A_IDs[0] != A_head)
        {
            // cout << A_head << " leaves" << endl;
            passTime_dX = Simulation::getTime();
            vector<vehicle>::iterator it = find_if(schedule_A.begin(), schedule_A.end(), find_id(A_head));
            schedule_A.erase(it);
            it = find_if(arrival_A.begin(), arrival_A.end(), find_id(A_head));
            waitingTime += max(0.0, passTime_dX - it->time);
            A_head = A_IDs[0];
        }
        else if (A_IDs.empty() && !A_head.empty())
        {
            if (stoi(A_head.substr(A_head.find("_") + 1)) == alpha)
            {
                // cout << A_head << " leaves" << endl;
                passTime_dX = Simulation::getTime();
                vector<vehicle>::iterator it = find_if(schedule_A.begin(), schedule_A.end(), find_id(A_head));
                schedule_A.erase(it);
                it = find_if(arrival_A.begin(), arrival_A.end(), find_id(A_head));
                waitingTime += max(0.0, passTime_dX - it->time);
                A_head = "";
            }
        }

        if (!B_IDs.empty() && B_IDs[0] != B_head)
        {
            // cout << B_head << " leaves" << endl;
            vector<vehicle>::iterator it = find_if(schedule_B.begin(), schedule_B.end(), find_id(B_head));
            if (Vehicle::getRouteID(it->id) == "route_1")
                passTime_dX = Simulation::getTime();
            else if (Vehicle::getRouteID(it->id) == "route_2")
                passTime_dY = Simulation::getTime();
            schedule_B.erase(it);
            it = find_if(arrival_B.begin(), arrival_B.end(), find_id(B_head));
            waitingTime += max(0.0, Simulation::getTime() - it->time);
            B_head = B_IDs[0];
        }
        else if (B_IDs.empty() && !B_head.empty())
        {
            if (stoi(B_head.substr(B_head.find("_") + 1)) == beta)
            {
                // cout << B_head << " leaves" << endl;
                vector<vehicle>::iterator it = find_if(schedule_B.begin(), schedule_B.end(), find_id(B_head));
                if (Vehicle::getRouteID(it->id) == "route_1")
                    passTime_dX = Simulation::getTime();
                else if (Vehicle::getRouteID(it->id) == "route_2")
                    passTime_dY = Simulation::getTime();
                schedule_B.erase(it);
                it = find_if(arrival_B.begin(), arrival_B.end(), find_id(B_head));
                waitingTime += max(0.0, Simulation::getTime() - it->time);
                B_head = "";
            }
        }

        if (!C_IDs.empty() && C_IDs[0] != C_head)
        {
            // cout << C_head << " leaves" << endl;
            passTime_dY = Simulation::getTime();
            vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
            schedule_C.erase(it);
            it = find_if(arrival_C.begin(), arrival_C.end(), find_id(C_head));
            waitingTime += max(0.0, passTime_dY - it->time);
            C_head = C_IDs[0];
        }
        else if (C_IDs.empty() && !C_head.empty())
        {
            if (stoi(C_head.substr(C_head.find("_") + 1)) == gamma)
            {
                // cout << C_head << " leaves" << endl;
                passTime_dY = Simulation::getTime();
                vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
                schedule_C.erase(it);
                it = find_if(arrival_C.begin(), arrival_C.end(), find_id(C_head));
                waitingTime += max(0.0, passTime_dY - it->time);
                C_head = "";
            }
        }

        if (!schedule_A.empty() && !schedule_B.empty() && !schedule_C.empty())
        {
            set_headway(schedule_A, schedule_B, schedule_C);
            // set_headway(schedule_A);
            // set_headway(schedule_B);
            // set_headway(schedule_C);
        }

        // Schedule
        if (Simulation::getTime() == period)
        {
            if (LaneArea::getLastStepVehicleNumber("dA") and LaneArea::getLastStepVehicleNumber("dB") and LaneArea::getLastStepVehicleNumber("dC"))
            {
                arrival_A = compute_earliest_arrival(laneLength, W_same, schedule_A, 'A');
                arrival_B = compute_earliest_arrival(laneLength, W_same, schedule_B, 'B');
                arrival_C = compute_earliest_arrival(laneLength, W_same, schedule_C, 'C');
                group_milp_compute_entering_time(arrival_A, arrival_B, arrival_C, W_same, W_diff, timeStep, schedule_A, schedule_B, schedule_C);
                set_headway(schedule_A, schedule_B, schedule_C);
                // set_headway(schedule_A);
                // set_headway(schedule_B);
                // set_headway(schedule_C);

                // print_schedule(schedule_A, "A");
                // print_schedule(schedule_B, "B");
                // print_schedule(schedule_C, "C");
            }
        }
    }
    cout << max(passTime_dX, passTime_dY) << ", " << waitingTime / (alpha + beta + gamma) << endl;
    Simulation::close();
}

int main(int argc, char *argv[])
{
    double timeStep = 0.5;
    double p, pA, pB, pC;
    int N, alpha, beta, gamma;
    double W_same, W_diff;
    vector<double> A, B, C;
    char isNewTest;

    if (argc >= 6)
    {
        p = atof(argv[1]);
        N = atoi(argv[2]);
        W_same = atof(argv[3]);
        W_diff = atof(argv[4]);
        pA = p;
        pB = p;
        pC = p;
        alpha = N;
        beta = N;
        gamma = N;
        isNewTest = argv[5][0];
    }
    else
    {
        cout << "Arguments: p, N, W=, W+, isNewTest" << endl;
        return 0;
    }
    if (argc > 6)
    {
        string inputPath(argv[6]);
        string outputPath = "output/output" + inputPath.substr(inputPath.find("_"));
        outputPath.replace(outputPath.end() - 8, outputPath.end(), "_groupMILP.xml");
        Simulation::start({"sumo", "-c", inputPath,
                           "--tripinfo-output", outputPath,
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
    }
    else
    {
        if (isNewTest == 'T' || isNewTest == 't' || isNewTest == '1')
        {
            cout << "Generate a new test" << endl;
            generate_routefile(timeStep, N, pA, pB, pC);
        }
        Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                           "--tripinfo-output", "sumo_data/tripinfo_groupMILP.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
    }
    run(alpha, beta, gamma, W_same, W_diff, timeStep);
}