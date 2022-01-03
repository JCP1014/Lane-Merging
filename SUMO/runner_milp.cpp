// Environment Variable:
//     export SUMO_HOME="/Users/jcp/sumo"
// Compilation:
//     g++ -o milp -std=c++11 -I$SUMO_HOME/src runner_milp.cpp -L$SUMO_HOME/bin -ltracicpp -I /Library/gurobi912/mac64/include -L /Library/gurobi912/mac64/lib -lgurobi_c++ -lgurobi91
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
    file << "    <route edges=\"A X\" color=\"yellow\" id=\"route_0\"/>" << endl;
    file << "    <route edges=\"B X\" color=\"yellow\" id=\"route_1\"/>" << endl;
    file << "    <route edges=\"B Y\" color=\"yellow\" id=\"route_2\"/>" << endl;
    file << "    <route edges=\"C Y\" color=\"yellow\" id=\"route_3\"/>" << endl;

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

void print_schedule(vector<vehicle> schedule, string lane)
{
    cout << "---------- schedule_" << lane << " " << schedule.size() << " ----------" << endl;
    for (auto veh : schedule)
    {
        cout << veh.id << " " << veh.time << endl;
    }
}

vector<vehicle> compute_earliest_arrival(int laneLength, vector<vehicle> &schedule, char lane)
{
    vector<vehicle> arrival_times = {vehicle()};
    double currentTime = Simulation::getTime();
    vector<string> vehicle_ids;

    if (lane == 'A')
        vehicle_ids = LaneArea::getLastStepVehicleIDs("dA");
    else if (lane == 'C')
        vehicle_ids = LaneArea::getLastStepVehicleIDs("dC");

    for (auto &vehID : vehicle_ids)
    {
        double dist = laneLength - Vehicle::getDistance(vehID);
        double speed = Vehicle::getSpeed(vehID);
        double arrivalTime = 0;
        if (speed == 0)
        {
            vector<vehicle>::iterator it = find_if(schedule.begin(), schedule.end(), find_id(vehID));
            if (it != schedule.end())
                arrivalTime = it->time;
        }
        else
        {
            arrivalTime = currentTime + dist / speed;
            vector<vehicle>::iterator it = find_if(schedule.begin(), schedule.end(), find_id(vehID));
            if (it != schedule.end())
                arrivalTime = min(arrivalTime, it->time);
        }
        arrival_times.push_back(vehicle(vehID, arrivalTime));
        sort(arrival_times.begin() + 1, arrival_times.end(), sort_vehicle);
    }
    return arrival_times;
}

vector<vehicle> compute_earliest_arrival(int laneLength, vector<vehicle> &schedule_1, vector<vehicle> &schedule_2, char lane)
{
    vector<vehicle> arrival_times = {vehicle()};
    double currentTime = Simulation::getTime();
    vector<string> vehicle_ids;

    if (lane == 'B')
        vehicle_ids = LaneArea::getLastStepVehicleIDs("dB");

    for (auto &vehID : vehicle_ids)
    {
        double dist = laneLength - Vehicle::getDistance(vehID);
        double speed = Vehicle::getSpeed(vehID);
        double arrivalTime = 0;
        if (speed == 0)
        {
            vector<vehicle>::iterator it = find_if(schedule_1.begin(), schedule_1.end(), find_id(vehID));
            if (it != schedule_1.end())
                arrivalTime = it->time;
            else
            {
                it = find_if(schedule_2.begin(), schedule_2.end(), find_id(vehID));
                if (it != schedule_2.end())
                    arrivalTime = it->time;
            }
        }
        else
        {
            arrivalTime = currentTime + dist / speed;
            vector<vehicle>::iterator it = find_if(schedule_1.begin(), schedule_1.end(), find_id(vehID));
            if (it != schedule_1.end())
                arrivalTime = min(arrivalTime, it->time);
            else
            {
                vector<vehicle>::iterator it = find_if(schedule_2.begin(), schedule_2.end(), find_id(vehID));
                if (it != schedule_2.end())
                    arrivalTime = min(arrivalTime, it->time);
            }
        }
        arrival_times.push_back(vehicle(vehID, arrivalTime));
        sort(arrival_times.begin() + 1, arrival_times.end(), sort_vehicle);
    }
    return arrival_times;
}

void milp_compute_entering_time(vector<vehicle> &A, vector<vehicle> &B, vector<vehicle> &C, double W_same, double W_diff, vector<vehicle> &schedule_A, vector<vehicle> &schedule_BX, vector<vehicle> &schedule_BY, vector<vehicle> &schedule_C)
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
        double total_wait = 0;
        schedule_A.clear();
        schedule_BX.clear();
        schedule_BY.clear();
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
                    schedule_BX.push_back(vehicle(B[j].id, t[j].get(GRB_DoubleAttr_X)));
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
                        schedule_BY.push_back(vehicle(B[j].id, t[j].get(GRB_DoubleAttr_X)));
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

void simple_compute_entering_time(char lane, vector<vehicle> traffic, double W_same, double W_diff, char prevLane, double prevTime, vector<vehicle> &schedule)
{
    schedule.clear();
    if (prevLane == '0')
        schedule.push_back(vehicle(traffic[1].id, traffic[1].time));
    else if (lane == prevLane)
        schedule.push_back(vehicle(traffic[1].id, max(traffic[1].time, prevTime + W_same)));
    else
        schedule.push_back(vehicle(traffic[1].id, max(traffic[1].time, prevTime + W_diff)));
    for (int i = 2; i < traffic.size(); ++i)
    {
        prevTime = schedule.back().time;
        schedule.push_back(vehicle(traffic[i].id, max(traffic[i].time, prevTime + W_same)));
    }
}

void run(int alpha, int beta, int gamma, double W_same, double W_diff)
{
    int period = 400;
    vector<vehicle> schedule_A, schedule_BX, schedule_BY, schedule_C;
    bool leaveA = false, leaveBX = false, leaveBY = false, leaveC = false;
    double countdownX = 0, countdownY = 0;
    bool gA = false, gBX = false, gBY = false, gC = false;
    int timeStep_cnt = 0;
    int laneLength = 6000;
    double passTime_dX = 0, passTime_dY = 0;
    vector<string> A_IDs, B_IDs, C_IDs;
    string A_head = "A_1", B_head = "B_1", C_head = "C_1";

    while (Simulation::getMinExpectedNumber() > 0)
    {
        for (auto &vehID : Simulation::getLoadedIDList())
            Vehicle::setLaneChangeMode(vehID, 0b000000000000);
        Simulation::step();
        ++timeStep_cnt;
        TrafficLight::setPhase("TL1", 1);

        // Detect the passing vehicles
        leaveA = false;
        leaveBX = false;
        leaveBY = false;
        leaveC = false;
        A_IDs = Edge::getLastStepVehicleIDs("A");
        B_IDs = Edge::getLastStepVehicleIDs("B");
        C_IDs = Edge::getLastStepVehicleIDs("C");
        sort(A_IDs.begin(), A_IDs.end(), sort_id);
        sort(B_IDs.begin(), B_IDs.end(), sort_id);
        sort(C_IDs.begin(), C_IDs.end(), sort_id);
        if (!A_IDs.empty() && A_IDs[0] != A_head)
        {
            cout << A_head << " leaves" << endl;
            passTime_dX = Simulation::getTime();
            leaveA = true;
            vector<vehicle>::iterator it = find_if(schedule_A.begin(), schedule_A.end(), find_id(A_head));
            if (it != schedule_A.end())
                schedule_A.erase(it);
            A_head = A_IDs[0];
        }
        else if (A_IDs.empty() && !A_head.empty())
        {
            if (stoi(A_head.substr(A_head.find("_") + 1)) == alpha)
            {
                cout << A_head << " leaves" << endl;
                passTime_dX = Simulation::getTime();
                leaveA = true;
                vector<vehicle>::iterator it = find_if(schedule_A.begin(), schedule_A.end(), find_id(A_head));
                if (it != schedule_A.end())
                    schedule_A.erase(it);
                A_head = "";
            }
        }

        if (!B_IDs.empty() && B_IDs[0] != B_head)
        {
            cout << B_head << " leaves" << endl;
            vector<vehicle>::iterator it = find_if(schedule_BX.begin(), schedule_BX.end(), find_id(B_head));
            if (it != schedule_BX.end())
            {
                passTime_dX = Simulation::getTime();
                leaveBX = true;
                schedule_BX.erase(it);
            }
            else
            {
                vector<vehicle>::iterator it = find_if(schedule_BY.begin(), schedule_BY.end(), find_id(B_head));
                if (it != schedule_BY.end())
                {
                    passTime_dY = Simulation::getTime();
                    leaveBY = true;
                    schedule_BY.erase(it);
                }
            }
            B_head = B_IDs[0];
        }
        else if (B_IDs.empty() && !B_head.empty())
        {
            if (stoi(B_head.substr(B_head.find("_") + 1)) == beta)
            {
                cout << B_head << " leaves" << endl;
                vector<vehicle>::iterator it = find_if(schedule_BX.begin(), schedule_BX.end(), find_id(B_head));
                if (it != schedule_BX.end())
                {
                    passTime_dX = Simulation::getTime();
                    leaveBX = true;
                    schedule_BX.erase(it);
                }
                else
                {
                    vector<vehicle>::iterator it = find_if(schedule_BY.begin(), schedule_BY.end(), find_id(B_head));
                    if (it != schedule_BY.end())
                    {
                        passTime_dY = Simulation::getTime();
                        leaveBY = true;
                        schedule_BY.erase(it);
                    }
                }
                B_head = "";
            }
        }

        if (!C_IDs.empty() && C_IDs[0] != C_head)
        {
            cout << C_head << " leaves" << endl;
            passTime_dY = Simulation::getTime();
            leaveC = true;
            vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
            if (it != schedule_C.end())
                schedule_C.erase(it);
            C_head = C_IDs[0];
        }
        else if (C_IDs.empty() && !C_head.empty())
        {
            if (stoi(C_head.substr(C_head.find("_") + 1)) == gamma)
            {
                cout << C_head << " leaves" << endl;
                passTime_dY = Simulation::getTime();
                leaveC = true;
                vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
                if (it != schedule_C.end())
                    schedule_C.erase(it);
                C_head = "";
            }
        }

        // Schedule
        if (timeStep_cnt - period == 0)
        {
            if (LaneArea::getLastStepVehicleNumber("dA") and LaneArea::getLastStepVehicleNumber("dB") and LaneArea::getLastStepVehicleNumber("dC"))
            {
                vector<vehicle> arrival_A = compute_earliest_arrival(laneLength, schedule_A, 'A');
                vector<vehicle> arrival_B = compute_earliest_arrival(laneLength, schedule_BX, schedule_BY, 'B');
                vector<vehicle> arrival_C = compute_earliest_arrival(laneLength, schedule_C, 'C');
                milp_compute_entering_time(arrival_A, arrival_B, arrival_C, W_same, W_diff, schedule_A, schedule_BX, schedule_BY, schedule_C);

                for (auto it = schedule_BX.begin(); it != schedule_BX.end();)
                {
                    try
                    {
                        Vehicle::setRouteID(it->id, "route_1");
                        ++it;
                    }
                    catch (std::exception &e)
                    {
                        cout << "exception: " << e.what() << "\n";
                        it = schedule_BX.erase(it);
                        leaveBY = true;
                    }
                }
                for (auto it = schedule_BY.begin(); it != schedule_BY.end();)
                {
                    try
                    {
                        Vehicle::setRouteID(it->id, "route_2");
                        ++it;
                    }
                    catch (std::exception &e)
                    {
                        cout << "exception: " << e.what() << "\n";
                        it = schedule_BY.erase(it);
                        leaveBX = true;
                    }
                }
                print_schedule(schedule_A, "A");
                print_schedule(schedule_BX, "BX");
                print_schedule(schedule_BY, "BY");
                print_schedule(schedule_C, "C");
            }
        }

        if (LaneArea::getLastStepVehicleNumber("dA") || LaneArea::getLastStepVehicleNumber("dB") || LaneArea::getLastStepVehicleNumber("dC"))
        {
            // Initialize the decision variables about traffic lights
            gA = false;
            gBX = false;
            gBY = false;
            gC = false;

            // Control outgoing lane X
            if (!schedule_A.empty() && !schedule_BX.empty())
            {
                if (schedule_A[0].time < schedule_BX[0].time)
                {
                    if (leaveA)
                        countdownX = W_same;
                    else if (leaveBX)
                        countdownX = W_diff;
                    else if (!countdownX)
                        gA = true;
                }
                else
                {
                    if (leaveBX)
                        countdownX = W_same;
                    else if (leaveA)
                        countdownX = W_diff;
                    else if (!countdownX)
                        gBX = true;
                }
            }
            else if (!schedule_A.empty())
            {
                if (leaveA)
                    countdownX = W_same;
                else if (leaveBX)
                    countdownX = W_diff;
                else if (!countdownX)
                    gA = true;
            }
            else if (!schedule_BX.empty())
            {
                if (leaveBX)
                    countdownX = W_same;
                else if (leaveA)
                    countdownX = W_diff;
                else if (!countdownX)
                    gBX = true;
            }
            else if (!countdownX)
            {
                gA = true;
                gBX = true;
            }

            // Control outgoing lane Y
            if (!schedule_C.empty() && !schedule_BY.empty())
            {
                if (schedule_C[0].time < schedule_BY[0].time)
                {
                    if (leaveC)
                        countdownY = W_same;
                    else if (leaveBY)
                        countdownY = W_diff;
                    else if (!countdownY)
                        gC = true;
                }
                else
                {
                    if (leaveBY)
                        countdownY = W_same;
                    else if (leaveC)
                        countdownY = W_diff;
                    else if (!countdownY)
                        gBY = true;
                }
            }
            else if (!schedule_C.empty())
            {
                if (leaveC)
                    countdownY = W_same;
                else if (leaveBY)
                    countdownY = W_diff;
                else if (!countdownY)
                    gC = true;
            }
            else if (!schedule_BY.empty())
            {
                if (leaveBY)
                    countdownY = W_same;
                else if (leaveC)
                    countdownY = W_diff;
                else if (!countdownY)
                    gBY = true;
            }
            else if (!countdownY)
            {
                gC = true;
                gBY = true;
            }

            // Set traffic lights
            if (gA && gBX && gBY && gC)
                TrafficLight::setPhase("TL1", 0);
            else if (gA && gBY)
                TrafficLight::setPhase("TL1", 2);
            else if (gA && gC)
                TrafficLight::setPhase("TL1", 4);
            else if (gBX && gC)
                TrafficLight::setPhase("TL1", 6);
            else if (gBX && gBY)
                TrafficLight::setPhase("TL1", 20);
            else if (gA)
                TrafficLight::setPhase("TL1", 8);
            else if (gBX)
                TrafficLight::setPhase("TL1", 10);
            else if (gBY)
                TrafficLight::setPhase("TL1", 12);
            else if (gC)
                TrafficLight::setPhase("TL1", 14);
            else
                TrafficLight::setPhase("TL1", 1);
            cout << "time: " << Simulation::getTime() << endl;
            // cout << gA << " " << gBX << " " << gBY << " " << gC << endl;
        }
        // Reduce waiting time
        if (countdownX)
            --countdownX;
        if (countdownY)
            --countdownY;
    }
    cout << max(passTime_dX, passTime_dY) << endl;
    Simulation::close();
}

int main(int argc, char *argv[])
{
    double timeStep = 1;
    double p, pA, pB, pC;
    int N, alpha, beta, gamma;
    double W_same, W_diff;
    vector<double> A, B, C;
    char isNewTest;

    if (argc == 6)
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
        isNewTest = argv[5][0];
    }
    else
    {
        cout << "Arguments: p, N, W=, W+, isNewTest" << endl;
        return 0;
    }

    if (isNewTest == 'T' || isNewTest == 't' || isNewTest == '1')
    {
        cout << "Generate a new test" << endl;
        generate_routefile(timeStep, N, pA, pB, pC);
    }
    Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                       "--tripinfo-output", "sumo_data/tripinfo_dp.xml",
                       "-S",
                       "--no-step-log", "true", "-W", "--duration-log.disable", "true"});

    run(alpha, beta, gamma, W_same, W_diff);
}