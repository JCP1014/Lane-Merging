#include "control.h"

// For sorting vehicles based on their scheduled entering times
bool sort_time(vehicle a, vehicle b)
{
    return a.time < b.time;
}

// Set the headway between consecutive vehicles based on the schedules
void set_headway(vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    vector<vehicle> total_schedule; // temporary vector for storing a schedule for Lane X and Lane Y
    int i;

    // Merge the schedule of Lane A and the scheudle of Lane B into a schedule for Lane X, and sort by each vehicle's scheduled entering time
    total_schedule.insert(total_schedule.end(), schedule_A.begin(), schedule_A.end());
    total_schedule.insert(total_schedule.end(), schedule_B.begin(), schedule_B.end());
    sort(total_schedule.begin(), total_schedule.end(), sort_time);
    i = 0;
    while (i < total_schedule.size())
    {
        if (Vehicle::getRouteID(total_schedule[i].id) == "route_2") // remove the vehicles going to Lane Y
            total_schedule.erase(total_schedule.begin() + i);
        else
            ++i;
    }
    for (i = 1; i < total_schedule.size(); ++i)
    {
        if (total_schedule[i].id[0] == total_schedule[i - 1].id[0]) // two consecutive vehicles are from the same lane
            Vehicle::openGap(total_schedule[i].id, W_same, 0, 0.5, 2, -1, total_schedule[i - 1].id);    // set headway
        else    // two consecutive vehicles are from different lanes
            Vehicle::openGap(total_schedule[i].id, W_diff, 0, 0.5, 2, -1, total_schedule[i - 1].id);
    }
    total_schedule.clear();

    // Merge the schedule of Lane B and the scheudle of Lane C into a schedule for Lane Y, and sort by each vehicle's scheduled entering time
    total_schedule.insert(total_schedule.end(), schedule_C.begin(), schedule_C.end());
    total_schedule.insert(total_schedule.end(), schedule_B.begin(), schedule_B.end());
    sort(total_schedule.begin(), total_schedule.end(), sort_time);
    i = 0;
    while (i < total_schedule.size())
    {
        if (Vehicle::getRouteID(total_schedule[i].id) == "route_1") // remove the vehicles going to Lane X
            total_schedule.erase(total_schedule.begin() + i);
        else
            ++i;
    }
    for (i = 1; i < total_schedule.size(); ++i) 
    {
        if (total_schedule[i].id[0] == total_schedule[i - 1].id[0]) // two consecutive vehicles are from the same lane
            Vehicle::openGap(total_schedule[i].id, W_same, 0, 0.5, 2, -1, total_schedule[i - 1].id);    // set headway
        else    // two consecutive vehicles are from different lanes
            Vehicle::openGap(total_schedule[i].id, W_diff, 0, 0.5, 2, -1, total_schedule[i - 1].id);
    }
}

// Set the initial headway between two consecutive vehicles when a vehicle departs,
// avoiding it accelerate to follow the leader vehicle and the traffic density be changed
void set_initial_gap(vector<string> &IDs, vector<bool> &inited, double laneLength, double minGap, double endTime)
{
    for (int i = 0; i < IDs.size(); ++i)
    {
        int num = stoi(IDs[i].substr(IDs[i].find("_") + 1));    // get the index of the vehicle
        if (!inited[num])   // if the vehicle has not been set the initial headway
        {
            auto leader = Vehicle::getLeader(IDs[i], laneLength);
            if (leader.second >= 0) // distance
            {
                Vehicle::openGap(IDs[i], 0.5, leader.second + minGap, endTime - Simulation::getTime(), 2, -1, leader.first);
            }
            inited[num] = true;
        }
    }
}

void run(int approach, int alpha, int beta, int gamma, int windowSize)
{
    int period = 300;   // time to schedule
    vector<vehicle> arrival_A, arrival_B, arrival_C;    // earliest arrival times of vehicles on Lane A, B, C
    vector<vehicle> schedule_A, schedule_B, schedule_C; // schedule of vehicles on Lane A, B, C
    const double laneLength = Lane::getLength("incoming_0");    // length of each incoming lane
    double T_last = 0;  // scheduled entering time of the last vehicle
    vector<string> A_IDs, B_IDs, C_IDs; // IDs of vehicles on Lane A, B, C
    string A_head = "A_1", B_head = "B_1", C_head = "C_1";  // the currently first vehicle on Lane A, B, C
    double total_wait = 0;  // total waiting time (difference between scheduled entering time and earliest arrival time)
    vector<bool> initGap_A(alpha + 1, 0), initGap_B(beta + 1, 0), initGap_C(gamma + 1, 0);  // for recording whether a vehicle has been set the initial headway
    const double minGap = 3.0;  // empty space after leader [m] (SUMO vType Attribute)

    while (Simulation::getMinExpectedNumber() > 0)  // while there is a vehicle in the simulation scenario
    {   
        for (auto vehID : Simulation::getLoadedIDList())    // when a vehicle is loaded
        {
            Vehicle::setLaneChangeMode(vehID, 0b000000000000);  // disable lane changing
        }
        Simulation::step(); // perform a simulation step
        // cout << Simulation::getTime() << endl;

        if (Simulation::getTime() == period)    // time to schedule
        {   
            if (LaneArea::getLastStepVehicleNumber("dA") and LaneArea::getLastStepVehicleNumber("dB") and LaneArea::getLastStepVehicleNumber("dC"))
            {
                // Estimate earliest arrival times of vehicles
                arrival_A = compute_earliest_arrival(laneLength, schedule_A, 'A');
                arrival_B = compute_earliest_arrival(laneLength, schedule_B, 'B');
                arrival_C = compute_earliest_arrival(laneLength, schedule_C, 'C');

                // Schedule
                switch (approach)
                {
                case FAFG:
                    fcfs_compute_entering_time(arrival_A, arrival_B, arrival_C, schedule_A, schedule_B, schedule_C);
                    break;
                case MILP:
                    milp_compute_entering_time(arrival_A, arrival_B, arrival_C, schedule_A, schedule_B, schedule_C);
                    break;
                case DP:
                    schedule_by_window_dp(arrival_A, arrival_B, arrival_C, windowSize, schedule_A, schedule_B, schedule_C);
                    break;
                case GROUP_MILP:
                    group_milp_compute_entering_time(arrival_A, arrival_B, arrival_C, schedule_A, schedule_B, schedule_C);
                    break;
                case GROUP_DP:
                    group_dp_compute_entering_time(arrival_A, arrival_B, arrival_C, schedule_A, schedule_B, schedule_C);
                    break;
                default:
                    cout << "Undefined approach." << endl;
                    break;
                }
                sort(schedule_B.begin(), schedule_B.end(), sort_vehicle);   // sort the schedule by the vehicles' scheduled entering times
                set_headway(schedule_A, schedule_B, schedule_C);    // set the headway between consecutive vehicles to make vehicles follow the scheduled passing order

                // print_schedule(schedule_A, "A");
                // print_schedule(schedule_B, "B");
                // print_schedule(schedule_C, "C");
            }
        }

        A_IDs = Lane::getLastStepVehicleIDs("incoming_2");
        B_IDs = Lane::getLastStepVehicleIDs("incoming_1");
        C_IDs = Lane::getLastStepVehicleIDs("incoming_0");
        sort(A_IDs.begin(), A_IDs.end(), sort_id);
        sort(B_IDs.begin(), B_IDs.end(), sort_id);
        sort(C_IDs.begin(), C_IDs.end(), sort_id);
        // The first vehicle does not need to be set the initial headway because there is no leader vehicle in front of it
        initGap_A[1] = true;
        initGap_B[1] = true;
        initGap_C[1] = true;
        set_initial_gap(A_IDs, initGap_A, laneLength, minGap, period);
        set_initial_gap(B_IDs, initGap_B, laneLength, minGap, period);
        set_initial_gap(C_IDs, initGap_C, laneLength, minGap, period);

        // Detect the passing vehicles
        if (!A_IDs.empty() && A_IDs[0] != A_head)
        {
            // cout << Simulation::getTime() << " " << A_head << " leaves" << endl;
            T_last = Simulation::getTime(); // update T_last

            // Remove the vehicle from the schedule because it has passed the merging intersection
            vector<vehicle>::iterator it = find_if(schedule_A.begin(), schedule_A.end(), find_id(A_head));
            schedule_A.erase(it);

            // Calculate the waiting time of the vehicle
            it = find_if(arrival_A.begin(), arrival_A.end(), find_id(A_head));
            total_wait += max(0.0, Simulation::getTime() - it->time);

            A_head = A_IDs[0];  // update the index of the first vehicle
        }
        else if (A_IDs.empty() && !A_head.empty())  // the last vehicle on Lane A
        {
            if (stoi(A_head.substr(A_head.find("_") + 1)) == alpha)
            {
                // cout << Simulation::getTime() << " " << A_head << " leaves" << endl;
                T_last = Simulation::getTime();
                vector<vehicle>::iterator it = find_if(schedule_A.begin(), schedule_A.end(), find_id(A_head));
                schedule_A.erase(it);
                it = find_if(arrival_A.begin(), arrival_A.end(), find_id(A_head));
                total_wait += max(0.0, Simulation::getTime() - it->time);
                A_head = "";
            }
        }

        if (!B_IDs.empty() && B_IDs[0] != B_head)
        {
            // cout << Simulation::getTime() << " " << B_head << " leaves" << endl;
            vector<vehicle>::iterator it = find_if(schedule_B.begin(), schedule_B.end(), find_id(B_head));
            T_last = Simulation::getTime();
            schedule_B.erase(it);
            it = find_if(arrival_B.begin(), arrival_B.end(), find_id(B_head));
            total_wait += max(0.0, Simulation::getTime() - it->time);
            B_head = B_IDs[0];
        }
        else if (B_IDs.empty() && !B_head.empty())
        {
            if (stoi(B_head.substr(B_head.find("_") + 1)) == beta)
            {
                // cout << Simulation::getTime() << " " << B_head << " leaves" << endl;
                vector<vehicle>::iterator it = find_if(schedule_B.begin(), schedule_B.end(), find_id(B_head));
                T_last = Simulation::getTime();
                schedule_B.erase(it);
                it = find_if(arrival_B.begin(), arrival_B.end(), find_id(B_head));
                total_wait += max(0.0, Simulation::getTime() - it->time);
                B_head = "";
            }
        }

        if (!C_IDs.empty() && C_IDs[0] != C_head)
        {
            // cout << Simulation::getTime() << " " << C_head << " leaves" << endl;
            vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
            schedule_C.erase(it);
            it = find_if(arrival_C.begin(), arrival_C.end(), find_id(C_head));
            total_wait += max(0.0, Simulation::getTime() - it->time);
            C_head = C_IDs[0];
        }
        else if (C_IDs.empty() && !C_head.empty())
        {
            if (stoi(C_head.substr(C_head.find("_") + 1)) == gamma)
            {
                // cout << Simulation::getTime() << " " << C_head << " leaves" << endl;
                vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
                schedule_C.erase(it);
                it = find_if(arrival_C.begin(), arrival_C.end(), find_id(C_head));
                total_wait += max(0.0, Simulation::getTime() - it->time);
                C_head = "";
            }
        }

        // Keep the vehicles follow the scheduled passing order
        if (!schedule_A.empty() && !schedule_B.empty() && !schedule_C.empty())
        {
            set_headway(schedule_A, schedule_B, schedule_C);
        }
    }
    cout << T_last << ", " << total_wait / (alpha + beta + gamma) << endl;
    Simulation::close();
}
