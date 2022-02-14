// Environment Variable:
//     export SUMO_HOME="/Users/jcp/sumo"
// Compilation:
//     g++ -o dp -std=c++11 -I$SUMO_HOME/src runner_dp.cpp -L$SUMO_HOME/bin -ltracicpp
// Run:
//     LD_LIBRARY_PATH=$SUMO_HOME/bin ./dp [density] [number] [W=] [W+] [windowSize=100] [generateNewTest (T/F)] [inputPath]
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <time.h>
#include <utility>
#include <stack>
#include <libsumo/libtraci.h>

using namespace std;
using namespace libtraci;

double W_same, W_diff;
struct Solution
{
    double time[2] = {INFINITY, INFINITY};
    string table = "";
    string lane = "";
    Solution *src = NULL;
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

Solution update_sol(Solution s, double newTimeX, double newTimeY, string newTable, string newLane)
{
    s.time[0] = newTimeX;
    s.time[1] = newTimeY;
    s.table = newTable;
    s.lane = newLane;
    return s;
}

Solution update_sol(Solution s, double newTimeX, double newTimeY, string newTable, string newLane, Solution *newSrc)
{
    s.time[0] = newTimeX;
    s.time[1] = newTimeY;
    s.table = newTable;
    s.lane = newLane;
    s.src = newSrc;
    return s;
}

Solution choose_best_sol(Solution s, vector<Solution> solVec)
{
    double minMax = max(solVec[0].time[0], solVec[0].time[1]);
    double minSum;
    double minSrcSum, minSrcMax;
    int index = 0;
    double tmpMax, tmpMin, tmpSum, tmpSrcMax, tmpSrcMin, tmpSrcSum;

    for (int i = 1; i < solVec.size(); ++i)
    {
        tmpMax = max(solVec[i].time[0], solVec[i].time[1]);
        if (tmpMax < minMax)
        {
            minMax = tmpMax;
            index = i;
        }
        else if (tmpMax == minMax)
        {
            tmpSum = solVec[i].time[0] + solVec[i].time[1];
            minSum = solVec[index].time[0] + solVec[index].time[1];
            if (tmpSum < minSum)
            {
                minMax = tmpMax;
                index = i;
            }
            else if (tmpSum == minSum && solVec[i].src)
            {
                tmpSrcMax = max(solVec[i].src->time[0], solVec[i].src->time[1]);
                minSrcMax = max(solVec[index].src->time[0], solVec[index].src->time[1]);
                if (tmpSrcMax < minSrcMax)
                {
                    minMax = tmpMax;
                    index = i;
                }
                else if (tmpSrcMax == minSrcMax)
                {
                    tmpSrcSum = solVec[i].src->time[0] + solVec[i].src->time[1];
                    minSrcSum = solVec[index].src->time[0] + solVec[index].src->time[1];
                    if (tmpSrcSum < minSrcSum)
                    {
                        minMax = tmpMax;
                        index = i;
                    }
                }
            }
        }
    }
    s = update_sol(s, solVec[index].time[0], solVec[index].time[1], solVec[index].table, solVec[index].lane);
    return s;
}

string get_opt_table(vector<Solution> solVec)
{
    double minMax = max(solVec[0].time[0], solVec[0].time[1]);
    double minSum;
    double minSrcSum, minSrcMax;
    int index = 0;
    double tmpMax, tmpMin, tmpSum, tmpSrcMax, tmpSrcMin, tmpSrcSum;
    string optTable = "AB";
    for (int i = 1; i < solVec.size(); ++i)
    {
        tmpMax = max(solVec[i].time[0], solVec[i].time[1]);
        if (tmpMax < minMax)
        {
            minMax = tmpMax;
            index = i;
        }
        else if (tmpMax == minMax)
        {
            tmpSum = solVec[i].time[0] + solVec[i].time[1];
            minSum = solVec[index].time[0] + solVec[index].time[1];
            if (tmpSum < minSum)
            {
                minMax = tmpMax;
                index = i;
            }
            else if (tmpSum == minSum && solVec[i].src)
            {
                tmpSrcMax = max(solVec[i].src->time[0], solVec[i].src->time[1]);
                minSrcMax = max(solVec[index].src->time[0], solVec[index].src->time[1]);
                if (tmpSrcMax < minSrcMax)
                {
                    minMax = tmpMax;
                    index = i;
                }
                else if (tmpSrcMax == minSrcMax)
                {
                    tmpSrcSum = solVec[i].src->time[0] + solVec[i].src->time[1];
                    minSrcSum = solVec[index].src->time[0] + solVec[index].src->time[1];
                    if (tmpSrcSum < minSrcSum)
                    {
                        minMax = tmpMax;
                        index = i;
                    }
                }
            }
        }
    }
    switch (index)
    {
    case 0:
    {
        optTable = "AB";
        break;
    }
    case 1:
    {
        optTable = "AC";
        break;
    }
    case 2:
    {
        optTable = "BB";
        break;
    }
    case 3:
    {
        optTable = "BC";
        break;
    }
    default:
    {
        optTable = "";
        break;
    }
    }
    return optTable;
}

void print_3d_table(vector<vector<vector<Solution>>> &table)
{
    for (int k = 0; k < table[0][0].size(); ++k)
    {
        cout << "k = " << k << endl;
        for (int i = 0; i < table.size(); ++i)
        {
            for (int j = 0; j < table[0].size(); ++j)
                cout << table[i][j][k].time[0] << " " << table[i][j][k].time[1] << " " << table[i][j][k].table << " " << table[i][j][k].lane << " || ";
            cout << endl;
        }
        cout << endl;
    }
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
    file << "    <vType id=\"typeA\" type=\"passenger\" length=\"4\" accel=\"2.6\" decel=\"4.5\" sigma=\"0.0\" maxSpeed=\"20\" color=\"yellow\"/>" << endl;
    file << "    <vType id=\"typeB\" type=\"passenger\" length=\"4\" accel=\"2.6\" decel=\"4.5\" sigma=\"0.0\" maxSpeed=\"20\" color=\"blue\"/>" << endl;
    file << "    <vType id=\"typeC\" type=\"passenger\" length=\"4\" accel=\"2.6\" decel=\"4.5\" sigma=\"0.0\" maxSpeed=\"20\" color=\"magenta\"/>" << endl;
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

tuple<tuple<char, int, double>, tuple<char, int, double>, double> window_dp_compute_entering_time(vector<vehicle> &a, vector<vehicle> &b, vector<vehicle> &c, tuple<char, int, double> last_X, tuple<char, int, double> last_Y, vector<vehicle> &schedule_A, vector<vehicle> &schedule_BX, vector<vehicle> &schedule_BY, vector<vehicle> &schedule_C)
{
    int alpha = a.size() - 1;
    int beta = b.size() - 1;
    int gamma = c.size() - 1;
    vector<vector<vector<Solution>>> L_AB;
    vector<vector<vector<Solution>>> L_AC;
    vector<vector<vector<Solution>>> L_BB;
    vector<vector<vector<Solution>>> L_BC;
    string last_XY;
    double T_X, T_Y;
    vector<Solution> tmpSolVec;
    double wait_time = 0;

    L_AB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_AC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));

    // Initialize
    L_AB[0][0][0] = update_sol(L_AB[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_AC[0][0][0] = update_sol(L_AC[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_BB[0][0][0] = update_sol(L_BB[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_BC[0][0][0] = update_sol(L_BC[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");

    last_XY.push_back(get<0>(last_X));
    last_XY.push_back(get<0>(last_Y));
    T_X = get<2>(last_X);
    T_Y = get<2>(last_Y);
    if (last_XY == "AB")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1].time, T_X + W_same), max(b[1].time, T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1].time, T_X + W_same), max(c[1].time, T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1].time, T_X + W_diff), max(c[1].time, T_Y + W_diff), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1].time, T_X + W_same), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1].time, T_Y + W_same), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1].time, T_X + W_same), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1].time, T_Y + W_diff), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1].time, T_X + W_diff), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1].time, T_Y + W_diff), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, T_X + W_diff), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1].time, T_Y + W_same), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "AC")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1].time, T_X + W_same), max(b[1].time, T_Y + W_diff), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1].time, T_X + W_same), max(c[1].time, T_Y + W_same), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1].time, T_X + W_diff), max(c[1].time, T_Y + W_same), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1].time, T_X + W_same), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1].time, T_Y + W_diff), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1].time, T_X + W_same), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1].time, T_Y + W_same), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1].time, T_X + W_diff), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1].time, T_Y + W_same), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, T_X + W_diff), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1].time, T_Y + W_diff), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "BB")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1].time, T_X + W_diff), max(b[1].time, T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1].time, T_X + W_diff), max(c[1].time, T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1].time, T_X + W_same), max(c[1].time, T_Y + W_diff), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1].time, T_X + W_diff), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1].time, T_Y + W_same), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1].time, T_X + W_diff), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1].time, T_Y + W_diff), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1].time, T_X + W_same), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1].time, T_Y + W_diff), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, T_X + W_same), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1].time, T_Y + W_same), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "BC")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1].time, T_X + W_diff), max(b[1].time, T_Y + W_diff), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1].time, T_X + W_diff), max(c[1].time, T_Y + W_same), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1].time, T_X + W_same), max(c[1].time, T_Y + W_same), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1].time, T_X + W_diff), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1].time, T_Y + W_diff), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1].time, T_X + W_diff), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1].time, T_Y + W_same), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1].time, T_X + W_same), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1].time, T_Y + W_same), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, T_X + W_same), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1].time, T_Y + W_diff), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], a[1].time, b[1].time, "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], a[1].time, c[1].time, "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], b[1].time, c[1].time, "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], a[1].time, -W_diff, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], -W_diff, b[1].time, "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], a[1].time, -W_diff, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], -W_diff, c[1].time, "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], b[1].time, -W_diff, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], -W_diff, c[1].time, "BC", "Y");
        L_BB[0][1][0] = (a[1].time <= c[1].time) ? update_sol(L_BB[0][1][0], -W_diff, b[1].time, "BB", "Y") : update_sol(L_BB[0][1][0], b[1].time, -W_diff, "BB", "X");
    }
    for (int j = 2; j <= beta; ++j)
    {
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j].time, L_BB[0][j - 1][0].time[0] + W_same), L_BB[0][j - 1][0].time[1], "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][0].time[0], max(b[j].time, L_BB[0][j - 1][0].time[1] + W_same), "BB", "Y");
        L_BB[0][j][0] = choose_best_sol(L_BB[0][j][0], tmpSolVec);
        tmpSolVec.clear();
    }
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][0][0] = update_sol(L_AB[i][0][0], max(a[i].time, L_AB[i - 1][0][0].time[0] + W_same), L_AB[i - 1][0][0].time[1], "AB", "X");
    for (int j = 2; j <= beta; ++j)
        L_AB[0][j][0] = update_sol(L_AB[0][j][0], L_AB[0][j - 1][0].time[0], max(b[j].time, L_AB[0][j - 1][0].time[1] + W_same), "AB", "Y");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][0] = update_sol(L_AC[i][0][0], max(a[i].time, L_AC[i - 1][0][0].time[0] + W_same), L_AC[i - 1][0][0].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[0][0][k] = update_sol(L_AC[0][0][k], L_AC[0][0][k - 1].time[0], max(c[k].time, L_AC[0][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int j = 2; j <= beta; ++j)
        L_BC[0][j][0] = update_sol(L_BC[0][j][0], max(b[j].time, L_BC[0][j - 1][0].time[0] + W_same), L_BC[0][j - 1][0].time[1], "BC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][0][k] = update_sol(L_BC[0][0][k], L_BC[0][0][k - 1].time[0], max(c[k].time, L_BC[0][0][k - 1].time[1] + W_same), "BC", "Y");

    if (beta > 1)
    {
        if (last_XY == "AB")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, L_AC[i][0][0].time[0] + W_diff), max(b[2].time, T_Y + W_same), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2].time, L_AC[i][0][0].time[0] + W_diff), max(b[1].time, T_Y + W_same), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, T_X + W_diff), max(b[2].time, L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2].time, T_X + W_diff), max(b[1].time, L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "AC")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, L_AC[i][0][0].time[0] + W_diff), max(b[2].time, T_Y + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2].time, L_AC[i][0][0].time[0] + W_diff), max(b[1].time, T_Y + W_diff), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, T_X + W_diff), max(b[2].time, L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2].time, T_X + W_diff), max(b[1].time, L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "BB")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, L_AC[i][0][0].time[0] + W_diff), max(b[2].time, T_Y + W_same), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2].time, L_AC[i][0][0].time[0] + W_diff), max(b[1].time, T_Y + W_same), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, T_X + W_same), max(b[2].time, L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2].time, T_X + W_same), max(b[1].time, L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "BC")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, L_AC[i][0][0].time[0] + W_diff), max(b[2].time, T_Y + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2].time, L_AC[i][0][0].time[0] + W_diff), max(b[1].time, T_Y + W_diff), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, T_X + W_same), max(b[2].time, L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2].time, T_X + W_same), max(b[1].time, L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1].time, L_AC[i][0][0].time[0] + W_diff), b[2].time, "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2].time, L_AC[i][0][0].time[0] + W_diff), b[1].time, "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], b[1].time, max(b[2].time, L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], b[2].time, max(b[1].time, L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
    }

    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 2; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i].time, L_AB[i - 1][j][0].time[0] + W_same), L_AB[i - 1][j][0].time[1], "AB", "X", &L_AB[i - 1][j][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i].time, L_BB[i - 1][j][0].time[0] + W_diff), L_BB[i - 1][j][0].time[1], "BB", "X", &L_BB[i - 1][j][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][0].time[0], max(b[j].time, L_AB[i][j - 1][0].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][0]);
            L_AB[i][j][0] = choose_best_sol(L_AB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 2; i <= alpha; ++i)
    {
        for (int k = 2; k <= gamma; ++k)
            L_AC[i][0][k] = update_sol(L_AC[i][0][k], max(a[i].time, L_AC[i - 1][0][k - 1].time[0] + W_same), max(c[k].time, L_AC[i - 1][0][k - 1].time[1] + W_same), "AC", "XY");
    }
    for (int j = 2; j <= beta; ++j)
    {
        for (int k = 1; k <= gamma; ++k)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j].time, L_BC[0][j - 1][k].time[0] + W_same), L_BC[0][j - 1][k].time[1], "BC", "X", &L_BC[0][j - 1][k]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BC[0][j][k - 1].time[0], max(c[k].time, L_BC[0][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[0][j][k - 1]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[0][j][k - 1].time[0], max(c[k].time, L_BB[0][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[0][j][k - 1]);
            L_BC[0][j][k] = choose_best_sol(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j].time, L_BB[i][j - 1][0].time[0] + W_same), L_BB[i][j - 1][0].time[1], "BB", "X", &L_BB[i][j - 1][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j].time, L_AB[i][j - 1][0].time[0] + W_diff), L_AB[i][j - 1][0].time[1], "AB", "X", &L_AB[i][j - 1][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][0].time[0], max(b[j].time, L_BB[i][j - 1][0].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][0]);
            L_BB[i][j][0] = choose_best_sol(L_BB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }

    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 1; j <= beta; ++j)
        {
            for (int k = 1; k <= gamma; ++k)
            {
                // L_AB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i].time, L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i].time, L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(b[j].time, L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(b[j].time, L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i].time, L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i].time, L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k].time, L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k].time, L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j].time, L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j].time, L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k].time, L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k].time, L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j].time, L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j].time, L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j].time, L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j].time, L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
                L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
    }

    // Print table
    // cout << "L_AB-------------------------------" << endl;
    // print_3d_table(L_AB);
    // cout << "L_AC-------------------------------" << endl;
    // print_3d_table(L_AC);
    // cout << "L_BB-------------------------------" << endl;
    // print_3d_table(L_BB);
    // cout << "L_BC-------------------------------" << endl;
    // print_3d_table(L_BC);

    // Push order to stack
    stack<tuple<char, int, double>> stack_X, stack_Y;
    int i = alpha;
    int j = beta;
    int k = gamma;
    string optTable;
    string table = "";
    string lanes = "";
    // Choose the optimal solution and the table to start backtracking
    tmpSolVec.push_back(L_AB[i][j][k]);
    tmpSolVec.push_back(L_AC[i][j][k]);
    tmpSolVec.push_back(L_BB[i][j][k]);
    tmpSolVec.push_back(L_BC[i][j][k]);
    optTable = get_opt_table(tmpSolVec);
    tmpSolVec.clear();

    if (optTable == "AB")
    {
        table = L_AB[i][j][k].table;
        lanes = L_AB[i][j][k].lane;
        if (lanes == "X")
        {
            stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
            --i;
        }
        else if (lanes == "Y")
        {
            stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
            --j;
        }
        else if (lanes == "XY")
        {
            stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
            stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
            --i;
            --j;
        }
    }
    else if (optTable == "AC")
    {
        table = L_AC[i][j][k].table;
        lanes = L_AC[i][j][k].lane;
        if (lanes == "X")
        {
            stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
            --i;
        }
        else if (lanes == "Y")
        {
            stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
            --k;
        }
        else if (lanes == "XY")
        {
            stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
            stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
            --i;
            --k;
        }
    }
    else if (optTable == "BB")
    {
        table = L_BB[i][j][k].table;
        lanes = L_BB[i][j][k].lane;
        if (lanes == "X")
        {
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            --j;
        }
        else if (lanes == "Y")
        {
            stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
            --j;
        }
        else if (lanes == "XY")
        {
            stack_X.push(make_tuple('B', j - 1, L_BB[i][j][k].time[0]));
            stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
            j -= 2;
        }
        else if (lanes == "YX")
        {
            stack_Y.push(make_tuple('B', j - 1, L_BB[i][j][k].time[1]));
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            j -= 2;
        }
    }
    else if (optTable == "BC")
    {
        table = L_BC[i][j][k].table;
        lanes = L_BC[i][j][k].lane;
        if (lanes == "X")
        {
            stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
            --j;
        }
        else if (lanes == "Y")
        {
            stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
            --k;
        }
        else if (lanes == "XY")
        {
            stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
            stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
            --j;
            --k;
        }
    }

    // Backtracking
    while (i > 0 || j > 0 || k > 0)
    {
        if (i < 0 || j < 0 || k < 0)
            exit(1);
        // cout << i << " " << j << " " << k << " ";
        // cout << table << " " << lanes << endl;
        if (table == "AB")
        {
            table = L_AB[i][j][k].table;
            lanes = L_AB[i][j][k].lane;
            if (lanes == "X")
            {
                stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
                --i;
            }
            else if (lanes == "Y")
            {
                stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
                --j;
            }
            else if (lanes == "XY")
            {
                stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
                stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
                --i;
                --j;
            }
        }
        else if (table == "AC")
        {
            table = L_AC[i][j][k].table;
            lanes = L_AC[i][j][k].lane;
            if (lanes == "X")
            {
                stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
                --i;
            }
            else if (lanes == "Y")
            {
                stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
                --k;
            }
            else if (lanes == "XY")
            {
                stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
                stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
                --i;
                --k;
            }
        }
        else if (table == "BB")
        {
            table = L_BB[i][j][k].table;
            lanes = L_BB[i][j][k].lane;
            if (lanes == "X")
            {
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
                --j;
            }
            else if (lanes == "Y")
            {
                stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
                --j;
            }
            else if (lanes == "XY")
            {
                stack_X.push(make_tuple('B', j - 1, L_BB[i][j][k].time[0]));
                stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
                j -= 2;
            }
            else if (lanes == "YX")
            {
                stack_Y.push(make_tuple('B', j - 1, L_BB[i][j][k].time[1]));
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
                j -= 2;
            }
        }
        else if (table == "BC")
        {
            table = L_BC[i][j][k].table;
            lanes = L_BC[i][j][k].lane;
            if (lanes == "X")
            {
                stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
                --j;
            }
            else if (lanes == "Y")
            {
                stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
                --k;
            }
            else if (lanes == "XY")
            {
                stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
                stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
                --j;
                --k;
            }
        }
    }

    // Delete the redundant element (i==0 or j==0 or k==0)
    while (get<1>(stack_X.top()) <= 0)
        stack_X.pop();
    while (get<1>(stack_Y.top()) <= 0)
        stack_Y.pop();

    // Output order
    schedule_A.clear();
    schedule_BX.clear();
    schedule_BY.clear();
    schedule_C.clear();
    // cout << "Lane X: " << endl;
    while (stack_X.size() > 1)
    {
        // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
        if (get<0>(stack_X.top()) == 'A')
        {
            schedule_A.push_back(vehicle(a[get<1>(stack_X.top())].id, get<2>(stack_X.top())));
            wait_time += (get<2>(stack_X.top()) - a[get<1>(stack_X.top())].time);
        }
        else
        {
            schedule_BX.push_back(vehicle(b[get<1>(stack_X.top())].id, get<2>(stack_X.top())));
            wait_time += (get<2>(stack_X.top()) - b[get<1>(stack_X.top())].time);
        }
        stack_X.pop();
    }
    // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
    if (get<0>(stack_X.top()) == 'A')
    {
        schedule_A.push_back(vehicle(a[get<1>(stack_X.top())].id, get<2>(stack_X.top())));
        wait_time += (get<2>(stack_X.top()) - a[get<1>(stack_X.top())].time);
    }
    else
    {
        schedule_BX.push_back(vehicle(b[get<1>(stack_X.top())].id, get<2>(stack_X.top())));
        wait_time += (get<2>(stack_X.top()) - b[get<1>(stack_X.top())].time);
    }
    last_X = stack_X.top();

    // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
    while (stack_Y.size() > 1)
    {
        // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
        if (get<0>(stack_Y.top()) == 'C')
        {
            schedule_C.push_back(vehicle(c[get<1>(stack_Y.top())].id, get<2>(stack_Y.top())));
            wait_time += (get<2>(stack_Y.top()) - c[get<1>(stack_Y.top())].time);
        }
        else
        {
            schedule_BY.push_back(vehicle(b[get<1>(stack_Y.top())].id, get<2>(stack_Y.top())));
            wait_time += (get<2>(stack_Y.top()) - b[get<1>(stack_Y.top())].time);
        }
        stack_Y.pop();
    }
    // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
    if (get<0>(stack_Y.top()) == 'C')
    {
        schedule_C.push_back(vehicle(c[get<1>(stack_Y.top())].id, get<2>(stack_Y.top())));
        wait_time += (get<2>(stack_Y.top()) - c[get<1>(stack_Y.top())].time);
    }
    else
    {
        schedule_BY.push_back(vehicle(b[get<1>(stack_Y.top())].id, get<2>(stack_Y.top())));
        wait_time += (get<2>(stack_Y.top()) - b[get<1>(stack_Y.top())].time);
    }
    last_Y = stack_Y.top();

    return make_tuple(last_X, last_Y, wait_time);
}

tuple<tuple<char, int, double>, double> schedule_single_lane(char lane, vector<vehicle> traffic, double W_same, double W_diff, tuple<char, int, double> prev, vector<vehicle> &schedule)
{
    char prevLane = get<0>(prev);
    double prevTime = get<2>(prev);
    tuple<char, int, double> last = make_tuple('0', 0, 0.0);
    double wait_time = 0;
    schedule.clear();
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

vector<vehicle> get_window_by_num(vector<vehicle> &traffic, int carNum)
{
    if (carNum >= traffic.size())
        carNum = traffic.size() - 1;
    vector<vehicle> subtraffic(traffic.begin(), traffic.begin() + carNum + 1);
    traffic.erase(traffic.begin() + 1, traffic.begin() + carNum + 1);
    return subtraffic;
}

void schedule_by_window_dp(vector<vehicle> a_all, vector<vehicle> b_all, vector<vehicle> c_all, int carNum, double W_same, double W_diff, vector<vehicle> &schedule_A, vector<vehicle> &schedule_BX, vector<vehicle> &schedule_BY, vector<vehicle> &schedule_C)
{
    tuple<char, int, double> last_X = make_tuple('0', 0, 0.0);
    tuple<char, int, double> last_Y = make_tuple('0', 0, 0.0);
    double wait_time = 0;
    double total_wait = 0;
    int vehicle_num = a_all.size() + b_all.size() + c_all.size() - 3;
    vector<vehicle> subSchedule_A, subSchedule_BX, subSchedule_BY, subSchedule_C;

    schedule_A.clear();
    schedule_BX.clear();
    schedule_BY.clear();
    schedule_C.clear();
    while (a_all.size() > 1 || b_all.size() > 1 || c_all.size() > 1)
    {
        vector<vehicle> a = get_window_by_num(a_all, carNum);
        vector<vehicle> b = get_window_by_num(b_all, carNum);
        vector<vehicle> c = get_window_by_num(c_all, carNum);
        if (a.size() > 1 && b.size() > 1 && c.size() > 1)
        {
            tie(last_X, last_Y, wait_time) = window_dp_compute_entering_time(a, b, c, last_X, last_Y, subSchedule_A, subSchedule_BX, subSchedule_BY, subSchedule_C);
            total_wait += wait_time;
            schedule_A.insert(schedule_A.end(), subSchedule_A.begin(), subSchedule_A.end());
            schedule_BX.insert(schedule_BX.end(), subSchedule_BX.begin(), subSchedule_BX.end());
            schedule_BY.insert(schedule_BY.end(), subSchedule_BY.begin(), subSchedule_BY.end());
            schedule_C.insert(schedule_C.end(), subSchedule_C.begin(), subSchedule_C.end());
        }
        else if (a.size() > 1 && b.size() > 1)
        {
            a.erase(a.begin());
            b.erase(b.begin());
            tie(last_X, wait_time) = schedule_single_lane('A', a, W_same, W_diff, last_X, subSchedule_A);
            total_wait += wait_time;
            schedule_A.insert(schedule_A.end(), subSchedule_A.begin(), subSchedule_A.end());
            tie(last_Y, wait_time) = schedule_single_lane('B', b, W_same, W_diff, last_Y, subSchedule_BY);
            total_wait += wait_time;
            schedule_BY.insert(schedule_BY.end(), subSchedule_BY.begin(), subSchedule_BY.end());
        }
        else if (a.size() > 1 and c.size() > 1)
        {
            a.erase(a.begin());
            c.erase(c.begin());
            tie(last_X, wait_time) = schedule_single_lane('A', a, W_same, W_diff, last_X, subSchedule_A);
            total_wait += wait_time;
            schedule_A.insert(schedule_A.end(), subSchedule_A.begin(), subSchedule_A.end());
            tie(last_Y, wait_time) = schedule_single_lane('C', c, W_same, W_diff, last_Y, subSchedule_C);
            total_wait += wait_time;
            schedule_C.insert(schedule_C.end(), subSchedule_C.begin(), subSchedule_C.end());
        }
        else if (b.size() > 1 and c.size() > 1)
        {
            b.erase(b.begin());
            c.erase(c.begin());
            tie(last_X, wait_time) = schedule_single_lane('B', b, W_same, W_diff, last_X, subSchedule_BX);
            total_wait += wait_time;
            schedule_BX.insert(schedule_BX.end(), subSchedule_BX.begin(), subSchedule_BX.end());
            tie(last_Y, wait_time) = schedule_single_lane('C', c, W_same, W_diff, last_Y, subSchedule_C);
            total_wait += wait_time;
            schedule_C.insert(schedule_C.end(), subSchedule_C.begin(), subSchedule_C.end());
        }
        else if (a.size() > 1)
        {
            a.erase(a.begin());
            tie(last_X, wait_time) = schedule_single_lane('A', a, W_same, W_diff, last_X, subSchedule_A);
            total_wait += wait_time;
            schedule_A.insert(schedule_A.end(), subSchedule_A.begin(), subSchedule_A.end());
        }
        else if (c.size() > 1)
        {
            c.erase(c.begin());
            tie(last_Y, wait_time) = schedule_single_lane('C', c, W_same, W_diff, last_Y, subSchedule_C);
            total_wait += wait_time;
            schedule_C.insert(schedule_C.end(), subSchedule_C.begin(), subSchedule_C.end());
        }
        else if (b.size() > 1)
        {
            if (get<2>(last_X) < get<2>(last_Y))
            {
                if (get<0>(last_X) == '0')
                {
                    schedule_BX.push_back(vehicle(b[1].id, b[1].time));
                    last_X = make_tuple('B', 1, b[1].time);
                }
                else if (get<0>(last_X) == 'A')
                {
                    schedule_BX.push_back(vehicle(b[1].id, max(b[1].time, get<2>(last_X) + W_diff)));
                    last_X = make_tuple('B', 1, max(b[1].time, get<2>(last_X) + W_diff));
                }
                else
                {
                    schedule_BX.push_back(vehicle(b[1].id, max(b[1].time, get<2>(last_X) + W_same)));
                    last_X = make_tuple('B', 1, max(b[1].time, get<2>(last_X) + W_same));
                }
                total_wait += (get<2>(last_X) - b[1].time);
                if (b.size() > 2)
                {
                    if (get<0>(last_Y) == '0')
                    {
                        schedule_BY.push_back(vehicle(b[2].id, b[2].time));
                        last_Y = make_tuple('B', 2, b[2].time);
                    }
                    else if (get<0>(last_Y) == 'C')
                    {
                        schedule_BY.push_back(vehicle(b[2].id, max(b[2].time, get<2>(last_Y) + W_diff)));
                        last_Y = make_tuple('B', 2, max(b[2].time, get<2>(last_Y) + W_diff));
                    }
                    else
                    {
                        schedule_BY.push_back(vehicle(b[2].id, max(b[2].time, get<2>(last_Y) + W_same)));
                        last_Y = make_tuple('B', 2, max(b[2].time, get<2>(last_Y) + W_same));
                    }
                    total_wait += (get<2>(last_Y) - b[2].time);
                    for (int i = 3; i < b.size(); ++i)
                    {
                        if (i % 2 == 1)
                        {
                            schedule_BX.push_back(vehicle(b[i].id, max(b[i].time, get<2>(last_X) + W_same)));
                            last_X = make_tuple('B', i, max(b[i].time, get<2>(last_X) + W_same));
                            total_wait += (get<2>(last_X) - b[i].time);
                        }
                        else
                        {
                            schedule_BY.push_back(vehicle(b[i].id, max(b[i].time, get<2>(last_Y) + W_same)));
                            last_Y = make_tuple('B', i, max(b[i].time, get<2>(last_Y) + W_same));
                            total_wait += (get<2>(last_Y) - b[i].time);
                        }
                    }
                }
            }
            else
            {
                if (get<0>(last_Y) == '0')
                {
                    schedule_BY.push_back(vehicle(b[1].id, b[1].time));
                    last_Y = make_tuple('B', 1, b[1].time);
                }
                else if (get<0>(last_Y) == 'C')
                {
                    schedule_BY.push_back(vehicle(b[1].id, max(b[1].time, get<2>(last_Y) + W_diff)));
                    last_Y = make_tuple('B', 1, max(b[1].time, get<2>(last_Y) + W_diff));
                }
                else
                {
                    schedule_BY.push_back(vehicle(b[1].id, max(b[1].time, get<2>(last_Y) + W_same)));
                    last_Y = make_tuple('B', 1, max(b[1].time, get<2>(last_Y) + W_same));
                }
                total_wait += (get<2>(last_Y) - b[1].time);
                if (b.size() > 2)
                {
                    if (get<0>(last_X) == '0')
                    {
                        schedule_BX.push_back(vehicle(b[2].id, b[2].time));
                        last_X = make_tuple('B', 2, b[2].time);
                    }
                    else if (get<0>(last_X) == 'A')
                    {
                        schedule_BX.push_back(vehicle(b[2].id, max(b[2].time, get<2>(last_X) + W_diff)));
                        last_X = make_tuple('B', 2, max(b[2].time, get<2>(last_X) + W_diff));
                    }
                    else
                    {
                        schedule_BX.push_back(vehicle(b[2].id, max(b[2].time, get<2>(last_X) + W_same)));
                        last_X = make_tuple('B', 2, max(b[2].time, get<2>(last_X) + W_same));
                    }
                    total_wait += (get<2>(last_X) - b[2].time);
                    for (int i = 3; i < b.size(); ++i)
                    {
                        if (i % 2 == 1)
                        {
                            schedule_BY.push_back(vehicle(b[i].id, max(b[i].time, get<2>(last_Y) + W_same)));
                            last_Y = make_tuple('B', i, max(b[i].time, get<2>(last_Y) + W_same));
                            total_wait += (get<2>(last_Y) - b[i].time);
                        }
                        else
                        {
                            schedule_BX.push_back(vehicle(b[i].id, max(b[i].time, get<2>(last_X) + W_same)));
                            last_X = make_tuple('B', i, max(b[i].time, get<2>(last_X) + W_same));
                            total_wait += (get<2>(last_X) - b[i].time);
                        }
                    }
                }
            }
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    // cout << "dp_" << carNum << " result: " << T_last << " " << T_delay << " " << totalComputeTime << endl;
}

void run(int alpha, int beta, int gamma, double W_same, double W_diff, int windowSize)
{
    int period = 300;
    vector<vehicle> arrival_A, arrival_B, arrival_C;
    vector<vehicle> schedule_A, schedule_BX, schedule_BY, schedule_C;
    bool leaveA = false, leaveBX = false, leaveBY = false, leaveC = false;
    double countdownX = 0, countdownY = 0;
    bool gA = false, gBX = false, gBY = false, gC = false;
    int timeStep_cnt = 0;
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
        ++timeStep_cnt;
        TrafficLight::setPhase("TL1", 1);

        leaveA = false;
        leaveBX = false;
        leaveBY = false;
        leaveC = false;

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
            schedule_A.erase(it);
            it = find_if(arrival_A.begin(), arrival_A.end(), find_id(A_head));
            waitingTime += passTime_dX - it->time;
            A_head = A_IDs[0];
        }
        else if (A_IDs.empty() && !A_head.empty())
        {
            if (stoi(A_head.substr(A_head.find("_") + 1)) == alpha)
            {
                // cout << A_head << " leaves" << endl;
                waitingTime += Vehicle::getWaitingTime(A_head);
                passTime_dX = Simulation::getTime();
                leaveA = true;
                vector<vehicle>::iterator it = find_if(schedule_A.begin(), schedule_A.end(), find_id(A_head));
                schedule_A.erase(it);
                it = find_if(arrival_A.begin(), arrival_A.end(), find_id(A_head));
                waitingTime += passTime_dX - it->time;
                A_head = "";
            }
        }

        if (!B_IDs.empty() && B_IDs[0] != B_head)
        {
            // cout << B_head << " leaves" << endl;
            waitingTime += Vehicle::getWaitingTime(B_head);
            vector<vehicle>::iterator it = find_if(schedule_BX.begin(), schedule_BX.end(), find_id(B_head));
            if (it != schedule_BX.end())
            {
                passTime_dX = Simulation::getTime();
                leaveBX = true;
                schedule_BX.erase(it);
                it = find_if(arrival_B.begin(), arrival_B.end(), find_id(B_head));
                waitingTime += passTime_dX - it->time;
            }
            else
            {
                vector<vehicle>::iterator it = find_if(schedule_BY.begin(), schedule_BY.end(), find_id(B_head));
                passTime_dY = Simulation::getTime();
                leaveBY = true;
                schedule_BY.erase(it);
                it = find_if(arrival_B.begin(), arrival_B.end(), find_id(B_head));
                waitingTime += passTime_dY - it->time;
            }
            B_head = B_IDs[0];
        }
        else if (B_IDs.empty() && !B_head.empty())
        {
            if (stoi(B_head.substr(B_head.find("_") + 1)) == beta)
            {
                // cout << B_head << " leaves" << endl;
                waitingTime += Vehicle::getWaitingTime(B_head);
                vector<vehicle>::iterator it = find_if(schedule_BX.begin(), schedule_BX.end(), find_id(B_head));
                if (it != schedule_BX.end())
                {
                    passTime_dX = Simulation::getTime();
                    leaveBX = true;
                    schedule_BX.erase(it);
                    it = find_if(arrival_B.begin(), arrival_B.end(), find_id(B_head));
                    waitingTime += passTime_dX - it->time;
                }
                else
                {
                    vector<vehicle>::iterator it = find_if(schedule_BY.begin(), schedule_BY.end(), find_id(B_head));
                    if (it != schedule_BY.end())
                    {
                        passTime_dY = Simulation::getTime();
                        leaveBY = true;
                        schedule_BY.erase(it);
                        it = find_if(arrival_B.begin(), arrival_B.end(), find_id(B_head));
                        waitingTime += passTime_dY - it->time;
                    }
                }
                B_head = "";
            }
        }

        if (!C_IDs.empty() && C_IDs[0] != C_head)
        {
            // cout << C_head << " leaves" << endl;
            waitingTime += Vehicle::getWaitingTime(C_head);
            passTime_dY = Simulation::getTime();
            leaveC = true;
            vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
            schedule_C.erase(it);
            it = find_if(arrival_C.begin(), arrival_C.end(), find_id(C_head));
            waitingTime += passTime_dY - it->time;
            C_head = C_IDs[0];
        }
        else if (C_IDs.empty() && !C_head.empty())
        {
            if (stoi(C_head.substr(C_head.find("_") + 1)) == gamma)
            {
                // cout << C_head << " leaves" << endl;
                waitingTime += Vehicle::getWaitingTime(C_head);
                passTime_dY = Simulation::getTime();
                leaveC = true;
                vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
                schedule_C.erase(it);
                it = find_if(arrival_C.begin(), arrival_C.end(), find_id(C_head));
                waitingTime += passTime_dY - it->time;
                C_head = "";
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
                        countdownX = W_same - 1;
                    else if (leaveBX)
                        countdownX = W_diff - 1;
                    if (!countdownX)
                        gA = true;
                }
                else
                {
                    if (leaveBX)
                        countdownX = W_same - 1;
                    else if (leaveA)
                        countdownX = W_diff - 1;
                    if (!countdownX)
                        gBX = true;
                }
            }
            else if (!schedule_A.empty())
            {
                if (leaveA)
                    countdownX = W_same - 1;
                else if (leaveBX)
                    countdownX = W_diff - 1;
                if (!countdownX)
                    gA = true;
            }
            else if (!schedule_BX.empty())
            {
                if (leaveBX)
                    countdownX = W_same - 1;
                else if (leaveA)
                    countdownX = W_diff - 1;
                if (!countdownX)
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
                        countdownY = W_same - 1;
                    else if (leaveBY)
                        countdownY = W_diff - 1;
                    if (!countdownY)
                        gC = true;
                }
                else
                {
                    if (leaveBY)
                        countdownY = W_same - 1;
                    else if (leaveC)
                        countdownY = W_diff - 1;
                    if (!countdownY)
                        gBY = true;
                }
            }
            else if (!schedule_C.empty())
            {
                if (leaveC)
                    countdownY = W_same - 1;
                else if (leaveBY)
                    countdownY = W_diff - 1;
                if (!countdownY)
                    gC = true;
            }
            else if (!schedule_BY.empty())
            {
                if (leaveBY)
                    countdownY = W_same - 1;
                else if (leaveC)
                    countdownY = W_diff - 1;
                if (!countdownY)
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
            // cout << "time: " << Simulation::getTime() << endl;
            // cout << gA << " " << gBX << " " << gBY << " " << gC << endl;
        }
        // Reduce waiting time
        if (countdownX)
            --countdownX;
        if (countdownY)
            --countdownY;

        // Schedule
        if (timeStep_cnt - period == 0)
        {
            if (LaneArea::getLastStepVehicleNumber("dA") and LaneArea::getLastStepVehicleNumber("dB") and LaneArea::getLastStepVehicleNumber("dC"))
            {
                arrival_A = compute_earliest_arrival(laneLength, schedule_A, 'A');
                arrival_B = compute_earliest_arrival(laneLength, schedule_BX, schedule_BY, 'B');
                arrival_C = compute_earliest_arrival(laneLength, schedule_C, 'C');
                schedule_by_window_dp(arrival_A, arrival_B, arrival_C, windowSize, W_same, W_diff, schedule_A, schedule_BX, schedule_BY, schedule_C);

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
                // print_schedule(schedule_A, "A");
                // print_schedule(schedule_BX, "BX");
                // print_schedule(schedule_BY, "BY");
                // print_schedule(schedule_C, "C");
            }
        }
    }
    cout << max(passTime_dX, passTime_dY) << ", " << waitingTime / (alpha + beta + gamma) << endl;
    Simulation::close();
}

int main(int argc, char *argv[])
{
    double timeStep = 1;
    double p, pA, pB, pC;
    int N, alpha, beta, gamma;
    vector<double> A, B, C;
    int windowSize;
    char isNewTest;

    if (argc >= 7)
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
        windowSize = atoi(argv[5]);
        isNewTest = argv[6][0];
    }
    else
    {
        cout << "Arguments: p, N, W=, W+, windowSize, isNewTest" << endl;
        return 0;
    }
    if (argc > 7)
    {
        string inputPath(argv[7]);
        string outputPath = "output/output" + inputPath.substr(inputPath.find("_"));
        outputPath.replace(outputPath.end() - 8, outputPath.end(), "_groupDP.xml");
        Simulation::start({"sumo", "-c", inputPath,
                           "--tripinfo-output", outputPath,
                           "-S",
                           "--no-step-log", "true", "-W", "--duration-log.disable", "true"});
    }
    else
    {
        if (isNewTest == 'T' || isNewTest == 't' || isNewTest == '1')
        {
            cout << "Generate a new test" << endl;
            generate_routefile(timeStep, N, pA, pB, pC);
        }
        Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                           "--tripinfo-output", "sumo_data/tripinfo_dp.xml",
                           "-S",
                           "--no-step-log", "true", "-W", "--duration-log.disable", "true"});
    }

    run(alpha, beta, gamma, W_same, W_diff, windowSize);
}