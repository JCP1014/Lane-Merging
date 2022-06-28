// Environment Variable:
//     export SUMO_HOME="/Users/jcp/sumo"
// Compilation:
//     g++ -o runner -std=c++11 -I$SUMO_HOME/src runner_cplus.cpp -L$SUMO_HOME/bin -ltracicpp -I /Library/gurobi912/mac64/include -L /Library/gurobi912/mac64/lib -lgurobi_c++ -lgurobi91
// Run:
//     LD_LIBRARY_PATH=$SUMO_HOME/bin ./runner
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
    file << "    <vType id=\"typeA\" type=\"passenger\" length=\"4.5\" accel=\"2.6\" decel=\"4.5\" sigma=\"0.0\" maxSpeed=\"25\" color=\"yellow\"/>" << endl;
    file << "    <vType id=\"typeB\" type=\"passenger\" length=\"4.5\" accel=\"2.6\" decel=\"4.5\" sigma=\"0.0\" maxSpeed=\"25\" color=\"blue\"/>" << endl;
    file << "    <vType id=\"typeC\" type=\"passenger\" length=\"4.5\" accel=\"2.6\" decel=\"4.5\" sigma=\"0.0\" maxSpeed=\"25\" color=\"magenta\"/>" << endl;
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
        double accel = Vehicle::getAcceleration(vehID);
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
            if (accel != 0)
                arrivalTime = currentTime + (sqrt(max(0.0, speed * speed + 2 * accel * dist)) - speed) / accel;
            else
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

void run(int alpha, int beta, int gamma, double W_same, double W_diff)
{
    int period = 20;
    vector<vehicle> schedule_A, schedule_BX, schedule_BY, schedule_C;
    bool leaveA = false, leaveBX = false, leaveBY = false, leaveC = false;
    double countdownX = 0, countdownY = 0;
    bool gA = false, gBX = false, gBY = false, gC = false;
    int timeStep_cnt = 0;
    int laneLength = 600;
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

        leaveA = false;
        leaveBX = false;
        leaveBY = false;
        leaveC = false;

        // Schedule
        if (timeStep_cnt % period == 0)
        {
            if (LaneArea::getLastStepVehicleNumber("dA") and LaneArea::getLastStepVehicleNumber("dB") and LaneArea::getLastStepVehicleNumber("dC"))
            {
                vector<vehicle> arrival_A = compute_earliest_arrival(laneLength, schedule_A, 'A');
                vector<vehicle> arrival_B = compute_earliest_arrival(laneLength, schedule_BX, schedule_BY, 'B');
                vector<vehicle> arrival_C = compute_earliest_arrival(laneLength, schedule_C, 'C');
                compute_entering_time(arrival_A, arrival_B, arrival_C, W_same, W_diff, schedule_A, schedule_BX, schedule_BY, schedule_C);

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
            }
        }

        // Detect the passing vehicles
        A_IDs = Edge::getLastStepVehicleIDs("A");
        B_IDs = Edge::getLastStepVehicleIDs("B");
        C_IDs = Edge::getLastStepVehicleIDs("C");
        sort(A_IDs.begin(), A_IDs.end(), sort_id);
        sort(B_IDs.begin(), B_IDs.end(), sort_id);
        sort(C_IDs.begin(), C_IDs.end(), sort_id);
        if (!A_IDs.empty() && A_IDs[0] != A_head)
        {
            // cout << A_head << " leaves" << endl;
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
                // cout << A_head << " leaves" << endl;
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
            // cout << B_head << " leaves" << endl;
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
                // cout << B_head << " leaves" << endl;
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
            // cout << C_head << " leaves" << endl;
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
                // cout << C_head << " leaves" << endl;
                passTime_dY = Simulation::getTime();
                leaveC = true;
                vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
                if (it != schedule_C.end())
                    schedule_C.erase(it);
                C_head = "";
            }
        }

        // Decide the phase of traffic lights
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

    generate_routefile(timeStep, N, pA, pB, pC);
    /* GUI */
    Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                       "--tripinfo-output", "sumo_data/tripinfo_dp.xml",
                       "-S",
                       "--no-step-log", "true", "-W", "--duration-log.disable", "true"});
    /* No GUI */
    Simulation::start({"sumo", "-c", "sumo_data/laneMerging.sumocfg",
                       "--tripinfo-output", "sumo_data/tripinfo_dp.xml",
                       "-S",
                       "--no-step-log", "true", "-W", "--duration-log.disable", "true"});
    run(alpha, beta, gamma, W_same, W_diff);
}