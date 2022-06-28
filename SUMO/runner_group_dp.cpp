// Environment Variable:
//     export SUMO_HOME="/Users/jcp/sumo"
// Compilation:
//     g++ -o group_dp -std=c++11 -I$SUMO_HOME/src runner_group_dp.cpp -L$SUMO_HOME/bin -ltracicpp
// Run:
//     LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp [density] [number] [W=] [W+] [generateNewTest (T/F)] [inputPath]
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

bool sort_time(vehicle a, vehicle b)
{
    return a.time < b.time;
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

double get_tail_time(vector<vehicle> &traffic, pair<int, int> index, double head_time)
{
    double tail_time = max(traffic[index.first].time, head_time);
    for (int i = index.first + 1; i <= index.second; ++i)
    {
        tail_time = max(traffic[i].time, tail_time + W_same);
    }
    return tail_time;
}

void compute_member_time(vector<vehicle> &traffic, pair<int, int> index, double head_time, vector<vehicle> &schedule, double &total_wait)
{
    double tail_time = max(traffic[index.first].time, head_time);
    schedule.push_back(vehicle(traffic[index.first].id, tail_time));
    total_wait += (tail_time - traffic[index.first].time);
    for (int i = index.first + 1; i <= index.second; ++i)
    {
        tail_time = max(traffic[i].time, tail_time + W_same);
        schedule.push_back(vehicle(traffic[i].id, tail_time));
        total_wait += (tail_time - traffic[i].time);
    }
}

void group_dp_compute_entering_time(vector<vehicle> &A, vector<vehicle> &B, vector<vehicle> &C, double timeStep, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    vector<pair<int, int>> grouped_A = grouping(A, timeStep);
    vector<pair<int, int>> grouped_B = grouping(B, timeStep);
    vector<pair<int, int>> grouped_C = grouping(C, timeStep);
    int alpha = grouped_A.size() - 1;
    int beta = grouped_B.size() - 1;
    int gamma = grouped_C.size() - 1;
    vector<vector<vector<Solution>>> L_AB, L_AC, L_BB, L_BC;
    vector<Solution> tmpSolVec;
    double T_last;
    double total_wait = 0;
    int vehicle_num = A.size() + B.size() + C.size() - 3;

    L_AB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_AC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));

    // Initialize
    L_AB[0][0][0] = update_sol(L_AB[0][0][0], 0, 0, "", "");
    L_AC[0][0][0] = update_sol(L_AC[0][0][0], 0, 0, "", "");
    L_BB[0][0][0] = update_sol(L_BB[0][0][0], 0, 0, "", "");
    L_BC[0][0][0] = update_sol(L_BC[0][0][0], 0, 0, "", "");

    L_AB[1][1][0] = update_sol(L_AB[1][1][0], get_tail_time(A, grouped_A[1], 0), get_tail_time(B, grouped_B[1], 0), "AB", "XY");
    L_AC[1][0][1] = update_sol(L_AC[1][0][1], get_tail_time(A, grouped_A[1], 0), get_tail_time(C, grouped_C[1], 0), "AC", "XY");
    L_BC[0][1][1] = update_sol(L_BC[0][1][1], get_tail_time(B, grouped_B[1], 0), get_tail_time(C, grouped_C[1], 0), "BC", "XY");

    L_AB[1][0][0] = update_sol(L_AB[1][0][0], get_tail_time(A, grouped_A[1], 0), -W_diff, "AB", "X");
    L_AB[0][1][0] = update_sol(L_AB[0][1][0], -W_diff, get_tail_time(B, grouped_B[1], 0), "AB", "Y");
    L_AC[1][0][0] = update_sol(L_AC[1][0][0], get_tail_time(A, grouped_A[1], 0), -W_diff, "AC", "X");
    L_AC[0][0][1] = update_sol(L_AC[0][0][1], -W_diff, get_tail_time(C, grouped_C[1], 0), "AC", "Y");
    L_BC[0][1][0] = update_sol(L_BC[0][1][0], get_tail_time(B, grouped_B[1], 0), -W_diff, "BC", "X");
    L_BC[0][0][1] = update_sol(L_BC[0][0][1], -W_diff, get_tail_time(C, grouped_C[1], 0), "BC", "Y");
    L_BB[0][1][0] = (A[1].time <= C[1].time) ? update_sol(L_BB[0][1][0], -W_diff, get_tail_time(B, grouped_B[1], 0), "BB", "Y") : update_sol(L_BB[0][1][0], get_tail_time(B, grouped_B[1], 0), -W_diff, "BB", "X");

    for (int i = 2; i <= alpha; ++i)
        L_AB[i][1][0] = update_sol(L_AB[i][1][0], get_tail_time(A, grouped_A[i], L_AB[i - 1][1][0].time[0] + W_same), L_AB[i - 1][1][0].time[1], "AB", "X");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][1] = update_sol(L_AC[i][0][1], get_tail_time(A, grouped_A[i], L_AC[i - 1][0][1].time[0] + W_same), L_AC[i - 1][0][1].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[1][0][k] = update_sol(L_AC[1][0][k], L_AC[1][0][k - 1].time[0], get_tail_time(C, grouped_C[k], L_AC[1][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][1][k] = update_sol(L_BC[0][1][k], L_BC[0][1][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BC[0][1][k - 1].time[1] + W_same), "BC", "Y");
    for (int j = 2; j <= beta; ++j)
    {
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[j], L_BB[0][j - 1][0].time[0] + W_same), L_BB[0][j - 1][0].time[1], "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][0].time[0], get_tail_time(B, grouped_B[j], L_BB[0][j - 1][0].time[1] + W_same), "BB", "Y");
        L_BB[0][j][0] = choose_best_sol(L_BB[0][j][0], tmpSolVec);
        tmpSolVec.clear();
    }
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][0][0] = update_sol(L_AB[i][0][0], get_tail_time(A, grouped_A[i], L_AB[i - 1][0][0].time[0] + W_same), L_AB[i - 1][0][0].time[1], "AB", "X");
    for (int j = 2; j <= beta; ++j)
        L_AB[0][j][0] = update_sol(L_AB[0][j][0], L_AB[0][j - 1][0].time[0], get_tail_time(B, grouped_B[j], L_AB[0][j - 1][0].time[1] + W_same), "AB", "Y");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][0] = update_sol(L_AC[i][0][0], get_tail_time(A, grouped_A[i], L_AC[i - 1][0][0].time[0] + W_same), L_AC[i - 1][0][0].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[0][0][k] = update_sol(L_AC[0][0][k], L_AC[0][0][k - 1].time[0], get_tail_time(C, grouped_C[k], L_AC[0][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int j = 2; j <= beta; ++j)
        L_BC[0][j][0] = update_sol(L_BC[0][j][0], get_tail_time(B, grouped_B[j], L_BC[0][j - 1][0].time[0] + W_same), L_BC[0][j - 1][0].time[1], "BC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][0][k] = update_sol(L_BC[0][0][k], L_BC[0][0][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BC[0][0][k - 1].time[1] + W_same), "BC", "Y");

    if (beta > 1)
    {
        for (int i = 1; i <= alpha; ++i)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[1], L_AC[i][0][0].time[0] + W_diff), get_tail_time(B, grouped_B[2], 0), "AC", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(B, grouped_B[2], L_AC[i][0][0].time[0] + W_diff), get_tail_time(B, grouped_B[1], 0), "AC", "YX");
            L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
            tmpSolVec.clear();
        }
        for (int k = 1; k <= gamma; ++k)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[1], 0), get_tail_time(B, grouped_B[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(B, grouped_B[2], 0), get_tail_time(B, grouped_B[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
            L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }

    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 2; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(A, grouped_A[i], L_AB[i - 1][j][0].time[0] + W_same), L_AB[i - 1][j][0].time[1], "AB", "X", &L_AB[i - 1][j][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(A, grouped_A[i], L_BB[i - 1][j][0].time[0] + W_diff), L_BB[i - 1][j][0].time[1], "BB", "X", &L_BB[i - 1][j][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][0].time[0], get_tail_time(B, grouped_B[j], L_AB[i][j - 1][0].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][0]);
            L_AB[i][j][0] = choose_best_sol(L_AB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 2; i <= alpha; ++i)
    {
        for (int k = 2; k <= gamma; ++k)
            L_AC[i][0][k] = update_sol(L_AC[i][0][k], get_tail_time(A, grouped_A[i], L_AC[i - 1][0][k - 1].time[0] + W_same), get_tail_time(C, grouped_C[k], L_AC[i - 1][0][k - 1].time[1] + W_same), "AC", "XY");
    }
    for (int j = 2; j <= beta; ++j)
    {
        for (int k = 1; k <= gamma; ++k)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[j], L_BC[0][j - 1][k].time[0] + W_same), L_BC[0][j - 1][k].time[1], "BC", "X", &L_BC[0][j - 1][k]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BC[0][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BC[0][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[0][j][k - 1]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[0][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BB[0][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[0][j][k - 1]);
            L_BC[0][j][k] = choose_best_sol(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[j], L_BB[i][j - 1][0].time[0] + W_same), L_BB[i][j - 1][0].time[1], "BB", "X", &L_BB[i][j - 1][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(B, grouped_B[j], L_AB[i][j - 1][0].time[0] + W_diff), L_AB[i][j - 1][0].time[1], "AB", "X", &L_AB[i][j - 1][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][0].time[0], get_tail_time(B, grouped_B[j], L_BB[i][j - 1][0].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][0]);
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
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(A, grouped_A[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(A, grouped_A[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], get_tail_time(B, grouped_B[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], get_tail_time(B, grouped_B[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(A, grouped_A[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(A, grouped_A[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(B, grouped_B[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(B, grouped_B[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], get_tail_time(B, grouped_B[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], get_tail_time(B, grouped_B[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
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
    schedule_B.clear();
    schedule_C.clear();
    // cout << "Lane X: " << endl;
    double prev_tail = -W_diff;
    char prev_lane = '0';
    char curr_lane;
    while (stack_X.size() > 1)
    {
        // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
        curr_lane = get<0>(stack_X.top());
        if (curr_lane == 'A')
        {
            if (prev_lane == curr_lane)
                compute_member_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_same, schedule_A, total_wait);
            else
                compute_member_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_diff, schedule_A, total_wait);
        }
        else
        {
            pair<int, int> index = grouped_B[get<1>(stack_X.top())];
            if (prev_lane == curr_lane)
                compute_member_time(B, index, prev_tail + W_same, schedule_B, total_wait);
            else
                compute_member_time(B, index, prev_tail + W_diff, schedule_B, total_wait);
            for (int i = index.first; i <= index.second; ++i)
                Vehicle::setRouteID(B[i].id, "route_1");
        }
        prev_lane = curr_lane;
        prev_tail = get<2>(stack_X.top());
        stack_X.pop();
    }
    // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
    curr_lane = get<0>(stack_X.top());
    if (curr_lane == 'A')
    {
        if (prev_lane == curr_lane)
            compute_member_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_same, schedule_A, total_wait);
        else
            compute_member_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_diff, schedule_A, total_wait);
    }
    else
    {
        pair<int, int> index = grouped_B[get<1>(stack_X.top())];
        if (prev_lane == curr_lane)
            compute_member_time(B, index, prev_tail + W_same, schedule_B, total_wait);
        else
            compute_member_time(B, index, prev_tail + W_diff, schedule_B, total_wait);
        for (int i = index.first; i <= index.second; ++i)
            Vehicle::setRouteID(B[i].id, "route_1");
    }
    T_last = get<2>(stack_X.top());

    // cout << "Lane Y: " << endl;
    prev_tail = -W_diff;
    prev_lane = '0';
    while (stack_Y.size() > 1)
    {
        // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
        curr_lane = get<0>(stack_Y.top());
        if (curr_lane == 'C')
        {
            if (prev_lane == curr_lane)
                compute_member_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_same, schedule_C, total_wait);
            else
                compute_member_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_diff, schedule_C, total_wait);
        }
        else
        {
            pair<int, int> index = grouped_B[get<1>(stack_Y.top())];
            if (prev_lane == curr_lane)
                compute_member_time(B, index, prev_tail + W_same, schedule_B, total_wait);
            else
                compute_member_time(B, index, prev_tail + W_diff, schedule_B, total_wait);
            for (int i = index.first; i <= index.second; ++i)
                Vehicle::setRouteID(B[i].id, "route_2");
        }
        prev_lane = curr_lane;
        prev_tail = get<2>(stack_Y.top());
        stack_Y.pop();
    }
    // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
    curr_lane = get<0>(stack_Y.top());
    if (curr_lane == 'C')
    {
        if (prev_lane == curr_lane)
            compute_member_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_same, schedule_C, total_wait);
        else
            compute_member_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_diff, schedule_C, total_wait);
    }
    else
    {
        pair<int, int> index = grouped_B[get<1>(stack_Y.top())];
        if (prev_lane == curr_lane)
            compute_member_time(B, index, prev_tail + W_same, schedule_B, total_wait);
        else
            compute_member_time(B, index, prev_tail + W_diff, schedule_B, total_wait);
        for (int i = index.first; i <= index.second; ++i)
            Vehicle::setRouteID(B[i].id, "route_2");
    }
    T_last = max(T_last, get<2>(stack_Y.top()));
    double T_delay = total_wait / vehicle_num;
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

void run(int alpha, int beta, int gamma, double timeStep)
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
                group_dp_compute_entering_time(arrival_A, arrival_B, arrival_C, timeStep, schedule_A, schedule_B, schedule_C);
                sort(schedule_B.begin(), schedule_B.end(), sort_vehicle);
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
        outputPath.replace(outputPath.end() - 8, outputPath.end(), "_groupDP.xml");
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
                           "--tripinfo-output", "sumo_data/tripinfo_groupDP.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
    }

    run(alpha, beta, gamma, timeStep);
}