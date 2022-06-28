// Environment Variable:
//     export SUMO_HOME="/Users/jcp/sumo"
// Compilation:
//     g++ -o fcfs -std=c++11 -I$SUMO_HOME/src runner_fcfs.cpp -L$SUMO_HOME/bin -ltracicpp
// Run:
//     LD_LIBRARY_PATH=$SUMO_HOME/bin ./fcfs [density] [number] [W=] [W+] [generateNewTest (T/F)] [inputPath]
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <time.h>
#include <libsumo/libtraci.h>

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

tuple<tuple<char, int, double>, double> schedule_single_lane(char lane, vector<vehicle> traffic, double W_same, double W_diff, tuple<char, int, double> prev, vector<vehicle> &schedule)
{
    char prevLane = get<0>(prev);
    double prevTime = get<2>(prev);
    tuple<char, int, double> last = make_tuple('0', 0, 0.0);
    double wait_time = 0;
    if (prevLane == '0')
    {
        last = make_tuple(lane, 1, traffic[0].time);
        schedule.push_back(vehicle(traffic[0].id, traffic[0].time));
    }
    else if (lane == prevLane)
    {
        last = make_tuple(lane, 1, max(traffic[0].time, prevTime + W_same));
        schedule.push_back(vehicle(traffic[0].id, max(traffic[0].time, prevTime + W_same)));
    }
    else
    {
        last = make_tuple(lane, 1, max(traffic[0].time, prevTime + W_diff));
        schedule.push_back(vehicle(traffic[0].id, max(traffic[0].time, prevTime + W_diff)));
    }
    wait_time += (get<2>(last) - traffic[0].time);
    for (int i = 1; i < traffic.size(); ++i)
    {
        prevTime = get<2>(last);
        last = make_tuple(lane, i, max(traffic[i].time, prevTime + W_same));
        wait_time += (get<2>(last) - traffic[i].time);
        schedule.push_back(vehicle(traffic[i].id, max(traffic[i].time, prevTime + W_same)));
    }
    return make_tuple(last, wait_time);
}

void fcfs_compute_entering_time(vector<vehicle> a_all, vector<vehicle> b_all, vector<vehicle> c_all, double W_same, double W_diff, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    vector<vehicle> a(a_all.begin() + 1, a_all.end());
    vector<vehicle> b(b_all.begin() + 1, b_all.end());
    vector<vehicle> c(c_all.begin() + 1, c_all.end());
    double X_lastT = -W_diff;
    double Y_lastT = -W_diff;
    char X_lastFrom = '0';
    char Y_lastFrom = '0';
    tuple<char, int, double> last_X, last_Y;
    double first = 0;
    default_random_engine generator(time(NULL));
    uniform_int_distribution<int> distribution(0, 1);
    double total_wait = 0;
    double wait_time = 0;
    int vehicle_num = a_all.size() + b_all.size() + c_all.size() - 3;

    schedule_A.clear();
    schedule_B.clear();
    schedule_C.clear();
    while (a.size() > 0 || b.size() > 0 || c.size() > 0)
    {
        if (a.size() > 0 && b.size() > 0 && c.size() > 0)
        {
            first = min(min(a[0].time, b[0].time), c[0].time);
            if (first == a[0].time)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0].time, X_lastT + W_same);
                else
                    X_lastT = max(a[0].time, X_lastT + W_diff);
                X_lastFrom = 'A';
                schedule_A.push_back(vehicle(a[0].id, X_lastT));
                total_wait += (X_lastT - a[0].time);
                a.erase(a.begin());
            }
            if (first == c[0].time)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0].time, Y_lastT + W_same);
                else
                    Y_lastT = max(c[0].time, Y_lastT + W_diff);
                Y_lastFrom = 'C';
                schedule_C.push_back(vehicle(c[0].id, Y_lastT));
                total_wait += (Y_lastT - c[0].time);
                c.erase(c.begin());
            }
            if (first == b[0].time)
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0].time, X_lastT + W_same);
                    else
                        X_lastT = max(b[0].time, X_lastT + W_diff);
                    X_lastFrom = 'B';
                    Vehicle::setRouteID(b[0].id, "route_1");
                    schedule_B.push_back(vehicle(b[0].id, X_lastT));
                    total_wait += (X_lastT - b[0].time);
                    b.erase(b.begin());
                }
                else if (Y_lastT < X_lastT)
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0].time, Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0].time, Y_lastT + W_diff);
                    Y_lastFrom = 'B';
                    Vehicle::setRouteID(b[0].id, "route_2");
                    schedule_B.push_back(vehicle(b[0].id, Y_lastT));
                    total_wait += (Y_lastT - b[0].time);
                    b.erase(b.begin());
                }
                else
                {
                    if (a[0].time > c[0].time)
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0].time, X_lastT + W_same);
                        else
                            X_lastT = max(b[0].time, X_lastT + W_diff);
                        X_lastFrom = 'B';
                        Vehicle::setRouteID(b[0].id, "route_1");
                        schedule_B.push_back(vehicle(b[0].id, X_lastT));
                        total_wait += (X_lastT - b[0].time);
                        b.erase(b.begin());
                    }
                    else if (c[0].time > a[0].time)
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0].time, Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0].time, Y_lastT + W_diff);
                        Y_lastFrom = 'B';
                        Vehicle::setRouteID(b[0].id, "route_2");
                        schedule_B.push_back(vehicle(b[0].id, Y_lastT));
                        total_wait += (Y_lastT - b[0].time);
                        b.erase(b.begin());
                    }
                    else
                    {
                        if (distribution(generator) == 0)
                        {
                            if (X_lastFrom == 'B')
                                X_lastT = max(b[0].time, X_lastT + W_same);
                            else
                                X_lastT = max(b[0].time, X_lastT + W_diff);
                            X_lastFrom = 'B';
                            Vehicle::setRouteID(b[0].id, "route_1");
                            schedule_B.push_back(vehicle(b[0].id, X_lastT));
                            total_wait += (X_lastT - b[0].time);
                            b.erase(b.begin());
                        }
                        else
                        {
                            if (Y_lastFrom == 'B')
                                Y_lastT = max(b[0].time, Y_lastT + W_same);
                            else
                                Y_lastT = max(b[0].time, Y_lastT + W_diff);
                            Y_lastFrom = 'B';
                            Vehicle::setRouteID(b[0].id, "route_2");
                            schedule_B.push_back(vehicle(b[0].id, Y_lastT));
                            total_wait += (Y_lastT - b[0].time);
                            b.erase(b.begin());
                        }
                    }
                }
            }
        }
        else if (a.size() > 0 && b.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', a, W_same, W_diff, last_X, schedule_A);
            total_wait += wait_time;
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('B', b, W_same, W_diff, last_Y, schedule_B);
            for (auto veh : b)
                Vehicle::setRouteID(veh.id, "route_2");
            total_wait += wait_time;
            a.clear();
            b.clear();
        }
        else if (a.size() > 0 and c.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', a, W_same, W_diff, last_X, schedule_A);
            total_wait += wait_time;
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', c, W_same, W_diff, last_Y, schedule_C);
            total_wait += wait_time;
            a.clear();
            c.clear();
        }
        else if (b.size() > 0 and c.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('B', b, W_same, W_diff, last_X, schedule_B);
            for (auto veh : b)
                Vehicle::setRouteID(veh.id, "route_1");
            total_wait += wait_time;
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', c, W_same, W_diff, last_Y, schedule_C);
            total_wait += wait_time;
            b.clear();
            c.clear();
        }
        else if (a.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', a, W_same, W_diff, last_X, schedule_A);
            total_wait += wait_time;
            a.clear();
        }
        else if (c.size() > 0)
        {
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', c, W_same, W_diff, last_Y, schedule_C);
            total_wait += wait_time;
            c.clear();
        }
        else if (b.size() > 0)
        {
            while (b.size() > 0)
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0].time, X_lastT + W_same);
                    else
                        X_lastT = max(b[0].time, X_lastT + W_diff);
                    X_lastFrom = 'B';
                    Vehicle::setRouteID(b[0].id, "route_1");
                    schedule_B.push_back(vehicle(b[0].id, X_lastT));
                    total_wait += (X_lastT - b[0].time);
                    b.erase(b.begin());
                }
                else
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0].time, Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0].time, Y_lastT + W_diff);
                    Y_lastFrom = 'B';
                    Vehicle::setRouteID(b[0].id, "route_2");
                    schedule_B.push_back(vehicle(b[0].id, Y_lastT));
                    total_wait += (Y_lastT - b[0].time);
                    b.erase(b.begin());
                }
            }
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    // cout << "fcfs result: " << T_last << " " << T_delay << " " << totalComputeTime << endl;
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

void run(int alpha, int beta, int gamma, double W_same, double W_diff)
{
    int period = 300;
    vector<vehicle> arrival_A, arrival_B, arrival_C;
    vector<vehicle> schedule_A, schedule_B, schedule_C;
    bool leaveA = false, leaveBX = false, leaveBY = false, leaveC = false;
    double countdownX = 0, countdownY = 0;
    bool gA = false, gBX = false, gBY = false, gC = false;
    int laneLength = 6500;
    double passTime_dX = 0, passTime_dY = 0;
    vector<string> A_IDs, B_IDs, C_IDs;
    string A_head = "A_1", B_head = "B_1", C_head = "C_1";
    double waitingTime = 0;

    while (Simulation::getMinExpectedNumber() > 0)
    {
        for (auto vehID : Simulation::getLoadedIDList())
            Vehicle::setLaneChangeMode(vehID, 0b000000000000);
        Simulation::step();
        // TrafficLight::setPhase("TL1", 0);
        // cout << Simulation::getTime() << endl;

        leaveA = false;
        leaveBX = false;
        leaveBY = false;
        leaveC = false;

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
            leaveA = true;
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
                leaveA = true;
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
            {
                passTime_dX = Simulation::getTime();
                leaveBX = true;
            }
            else if (Vehicle::getRouteID(it->id) == "route_2")
            {
                passTime_dY = Simulation::getTime();
                leaveBY = true;
            }
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
                {
                    passTime_dX = Simulation::getTime();
                    leaveBX = true;
                }
                else if (Vehicle::getRouteID(it->id) == "route_2")
                {
                    passTime_dY = Simulation::getTime();
                    leaveBY = true;
                }
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
            leaveC = true;
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
                leaveC = true;
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
        // if (LaneArea::getLastStepVehicleNumber("dA") || LaneArea::getLastStepVehicleNumber("dB") || LaneArea::getLastStepVehicleNumber("dC"))
        // {
        //     // Initialize the decision variables about traffic lights
        //     gA = false;
        //     gBX = false;
        //     gBY = false;
        //     gC = false;

        //     // Control outgoing lane X
        //     if (!schedule_A.empty() && !schedule_BX.empty())
        //     {
        //         if (schedule_A[0].time < schedule_BX[0].time)
        //         {
        //             if (leaveA)
        //                 countdownX = W_same - 1;
        //             else if (leaveBX)
        //                 countdownX = W_diff - 1;
        //             if (!countdownX)
        //                 gA = true;
        //         }
        //         else
        //         {
        //             if (leaveBX)
        //                 countdownX = W_same - 1;
        //             else if (leaveA)
        //                 countdownX = W_diff - 1;
        //             if (!countdownX)
        //                 gBX = true;
        //         }
        //     }
        //     else if (!schedule_A.empty())
        //     {
        //         if (leaveA)
        //             countdownX = W_same - 1;
        //         else if (leaveBX)
        //             countdownX = W_diff - 1;
        //         if (!countdownX)
        //             gA = true;
        //     }
        //     else if (!schedule_BX.empty())
        //     {
        //         if (leaveBX)
        //             countdownX = W_same - 1;
        //         else if (leaveA)
        //             countdownX = W_diff - 1;
        //         if (!countdownX)
        //             gBX = true;
        //     }
        //     else if (!countdownX)
        //     {
        //         gA = true;
        //         gBX = true;
        //     }

        //     // Control outgoing lane Y
        //     if (!schedule_C.empty() && !schedule_BY.empty())
        //     {
        //         if (schedule_C[0].time < schedule_BY[0].time)
        //         {
        //             if (leaveC)
        //                 countdownY = W_same - 1;
        //             else if (leaveBY)
        //                 countdownY = W_diff - 1;
        //             if (!countdownY)
        //                 gC = true;
        //         }
        //         else
        //         {
        //             if (leaveBY)
        //                 countdownY = W_same - 1;
        //             else if (leaveC)
        //                 countdownY = W_diff - 1;
        //             if (!countdownY)
        //                 gBY = true;
        //         }
        //     }
        //     else if (!schedule_C.empty())
        //     {
        //         if (leaveC)
        //             countdownY = W_same - 1;
        //         else if (leaveBY)
        //             countdownY = W_diff - 1;
        //         if (!countdownY)
        //             gC = true;
        //     }
        //     else if (!schedule_BY.empty())
        //     {
        //         if (leaveBY)
        //             countdownY = W_same - 1;
        //         else if (leaveC)
        //             countdownY = W_diff - 1;
        //         if (!countdownY)
        //             gBY = true;
        //     }
        //     else if (!countdownY)
        //     {
        //         gC = true;
        //         gBY = true;
        //     }

        //     // Set traffic lights
        //     if (gA && gBX && gBY && gC)
        //         TrafficLight::setPhase("TL1", 0);
        //     else if (gA && gBY)
        //         TrafficLight::setPhase("TL1", 2);
        //     else if (gA && gC)
        //         TrafficLight::setPhase("TL1", 4);
        //     else if (gBX && gC)
        //         TrafficLight::setPhase("TL1", 6);
        //     else if (gBX && gBY)
        //         TrafficLight::setPhase("TL1", 20);
        //     else if (gA)
        //         TrafficLight::setPhase("TL1", 8);
        //     else if (gBX)
        //         TrafficLight::setPhase("TL1", 10);
        //     else if (gBY)
        //         TrafficLight::setPhase("TL1", 12);
        //     else if (gC)
        //         TrafficLight::setPhase("TL1", 14);
        //     else
        //         TrafficLight::setPhase("TL1", 1);
        //     // cout << "time: " << Simulation::getTime() << endl;
        //     // cout << gA << " " << gBX << " " << gBY << " " << gC << endl;
        // }
        // // Reduce waiting time
        // if (countdownX)
        //     --countdownX;
        // if (countdownY)
        //     --countdownY;

        // Schedule
        if (Simulation::getTime() == period)
        {
            if (LaneArea::getLastStepVehicleNumber("dA") and LaneArea::getLastStepVehicleNumber("dB") and LaneArea::getLastStepVehicleNumber("dC"))
            {
                arrival_A = compute_earliest_arrival(laneLength, W_same, schedule_A, 'A');
                arrival_B = compute_earliest_arrival(laneLength, W_same, schedule_B, 'B');
                arrival_C = compute_earliest_arrival(laneLength, W_same, schedule_C, 'C');
                fcfs_compute_entering_time(arrival_A, arrival_B, arrival_C, W_same, W_diff, schedule_A, schedule_B, schedule_C);
                set_headway(schedule_A, schedule_B, schedule_C);
                // set_headway(schedule_A);
                // set_headway(schedule_B);
                // set_headway(schedule_C);

                // for (auto it = schedule_BX.begin(); it != schedule_BX.end();)
                // {
                //     try
                //     {
                //         Vehicle::setRouteID(it->id, "route_1");
                //         ++it;
                //     }
                //     catch (std::exception &e)
                //     {
                //         cout << "exception: " << e.what() << "\n";
                //         it = schedule_BX.erase(it);
                //         leaveBY = true;
                //     }
                // }
                // for (auto it = schedule_BY.begin(); it != schedule_BY.end();)
                // {
                //     try
                //     {
                //         Vehicle::setRouteID(it->id, "route_2");
                //         ++it;
                //     }
                //     catch (std::exception &e)
                //     {
                //         cout << "exception: " << e.what() << "\n";
                //         it = schedule_BY.erase(it);
                //         leaveBX = true;
                //     }
                // }
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
        outputPath.replace(outputPath.end() - 8, outputPath.end(), "_fcfs.xml");
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
            generate_routefile(W_same, N, pA, pB, pC);
        }
        Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                           "--tripinfo-output", "sumo_data/tripinfo_fcfs.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
    }
    run(alpha, beta, gamma, W_same, W_diff);
}