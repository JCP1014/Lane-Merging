#include "control.h"

bool sort_time(vehicle a, vehicle b)
{
    return a.time < b.time;
}

void set_headway(vector<vehicle> &schedule)
{
    for (int i = 1; i < schedule.size(); ++i)
        Vehicle::openGap(schedule[i].id, max(1.0, schedule[i].time - schedule[i - 1].time), 0, 0.5, 2);
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
        // Vehicle::openGap(total_schedule[i].id, max(1.0, total_schedule[i].time - total_schedule[i - 1].time), 0, 0.5, 0.5, -1, total_schedule[i - 1].id);
        if (total_schedule[i].id[0] == total_schedule[i - 1].id[0])
            Vehicle::openGap(total_schedule[i].id, W_same, 0, 0.5, 2, -1, total_schedule[i - 1].id);
        else
            Vehicle::openGap(total_schedule[i].id, W_diff, 0, 0.5, 2, -1, total_schedule[i - 1].id);
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
        // Vehicle::openGap(total_schedule[i].id, max(1.0, total_schedule[i].time - total_schedule[i - 1].time), 0, 0.5, 0.5, -1, total_schedule[i - 1].id);
        if (total_schedule[i].id[0] == total_schedule[i - 1].id[0])
            Vehicle::openGap(total_schedule[i].id, W_same, 0, 0.5, 2, -1, total_schedule[i - 1].id);
        else
            Vehicle::openGap(total_schedule[i].id, W_diff, 0, 0.5, 2, -1, total_schedule[i - 1].id);
    }
}

void set_initial_gap(vector<string> &IDs, vector<bool> &inited, double laneLength, double minGap, double endTime)
{
    for (int i = 0; i < IDs.size(); ++i)
    {
        int num = stoi(IDs[i].substr(IDs[i].find("_") + 1));
        if (!inited[num])
        {
            auto leader = Vehicle::getLeader(IDs[i], laneLength);
            if (leader.second >= 0)
            {
                // cout << "Init Gap " << IDs[i] << " " << leader.first << " " << leader.second << endl;
                Vehicle::openGap(IDs[i], 0.5, leader.second + minGap, endTime - Simulation::getTime(), 2, -1, leader.first);
            }
            // auto left_leaders = Vehicle::getNeighbors(IDs[i], 2);
            // auto right_leaders = Vehicle::getNeighbors(IDs[i], 3);
            // if (!left_leaders.empty() && left_leaders[0].second >= 0)
            // {
            //     // cout << "Init Gap " << IDs[i] << " " << left_leaders[0].first << " " << left_leaders[0].second << endl;
            //     Vehicle::openGap(IDs[i], 0.5, left_leaders[0].second + minGap, 0.5, 0.5, -1, left_leaders[0].first);
            // }
            // if (!right_leaders.empty() && right_leaders[0].second >= 0)
            // {
            //     // cout << "Init Gap " << IDs[i] << " " << right_leaders[0].first << " " << right_leaders[0].second << endl;
            //     Vehicle::openGap(IDs[i], 0.5, right_leaders[0].second + minGap, 0.5, 0.5, -1, right_leaders[0].first);
            // }
            inited[num] = true;
        }
    }
}

void run(int approach, int alpha, int beta, int gamma, int windowSize)
{
    int period = 300;
    vector<vehicle> arrival_A, arrival_B, arrival_C;
    vector<vehicle> schedule_A, schedule_B, schedule_C;
    double countdownX = 0, countdownY = 0;
    bool gA = false, gBX = false, gBY = false, gC = false;
    const double laneLength = Lane::getLength("incoming_0");
    double passTime_dX = 0, passTime_dY = 0;
    vector<string> A_IDs, B_IDs, C_IDs;
    string A_head = "A_1", B_head = "B_1", C_head = "C_1";
    double waitingTime = 0;
    vector<bool> initGap_A(alpha + 1, 0), initGap_B(beta + 1, 0), initGap_C(gamma + 1, 0);
    const double minGap = 3.0;

    while (Simulation::getMinExpectedNumber() > 0)
    {
        for (auto vehID : Simulation::getLoadedIDList())
        {
            Vehicle::setLaneChangeMode(vehID, 0b000000000000);
            // Vehicle::setTau(vehID, W_same);
        }
        Simulation::step();
        // cout << Simulation::getTime() << endl;

        // Schedule
        if (Simulation::getTime() == period)
        {
            initGap_A[1] = true;
            initGap_B[1] = true;
            initGap_C[1] = true;
            if (LaneArea::getLastStepVehicleNumber("dA") and LaneArea::getLastStepVehicleNumber("dB") and LaneArea::getLastStepVehicleNumber("dC"))
            {
                arrival_A = compute_earliest_arrival(laneLength, schedule_A, 'A');
                arrival_B = compute_earliest_arrival(laneLength, schedule_B, 'B');
                arrival_C = compute_earliest_arrival(laneLength, schedule_C, 'C');
                // for (auto a:arrival_A)
                //     cout << a.id << " " << a.time << endl;
                // for (auto a:arrival_B)
                //     cout << a.id << " " << a.time << endl;
                // for (auto a:arrival_C)
                //     cout << a.id << " " << a.time << endl;
                switch (approach)
                {
                case FCFS:
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

        // Detect the passing vehicles
        A_IDs = Lane::getLastStepVehicleIDs("incoming_2");
        B_IDs = Lane::getLastStepVehicleIDs("incoming_1");
        C_IDs = Lane::getLastStepVehicleIDs("incoming_0");
        sort(A_IDs.begin(), A_IDs.end(), sort_id);
        sort(B_IDs.begin(), B_IDs.end(), sort_id);
        sort(C_IDs.begin(), C_IDs.end(), sort_id);
        set_initial_gap(A_IDs, initGap_A, laneLength, minGap, period);
        set_initial_gap(B_IDs, initGap_B, laneLength, minGap, period);
        set_initial_gap(C_IDs, initGap_C, laneLength, minGap, period);

        if (!A_IDs.empty() && A_IDs[0] != A_head)
        {
            // cout << Simulation::getTime() << " " << A_head << " leaves" << endl;
            passTime_dX = Simulation::getTime();
            vector<vehicle>::iterator it = find_if(schedule_A.begin(), schedule_A.end(), find_id(A_head));
            // waitingTime += max(0.0, passTime_dX - it->time);
            schedule_A.erase(it);
            it = find_if(arrival_A.begin(), arrival_A.end(), find_id(A_head));
            waitingTime += max(0.0, Simulation::getTime() - it->time);
            A_head = A_IDs[0];
        }
        else if (A_IDs.empty() && !A_head.empty())
        {
            if (stoi(A_head.substr(A_head.find("_") + 1)) == alpha)
            {
                // cout << Simulation::getTime() << " " << A_head << " leaves" << endl;
                passTime_dX = Simulation::getTime();
                vector<vehicle>::iterator it = find_if(schedule_A.begin(), schedule_A.end(), find_id(A_head));
                // waitingTime += max(0.0, passTime_dX - it->time);
                schedule_A.erase(it);
                it = find_if(arrival_A.begin(), arrival_A.end(), find_id(A_head));
                waitingTime += max(0.0, Simulation::getTime() - it->time);
                A_head = "";
            }
        }

        if (!B_IDs.empty() && B_IDs[0] != B_head)
        {
            // cout << Simulation::getTime() << " " << B_head << " leaves" << endl;
            vector<vehicle>::iterator it = find_if(schedule_B.begin(), schedule_B.end(), find_id(B_head));
            if (Vehicle::getRouteID(it->id) == "route_1")
                passTime_dX = Simulation::getTime();
            else if (Vehicle::getRouteID(it->id) == "route_2")
                passTime_dY = Simulation::getTime();
            // waitingTime += max(0.0, Simulation::getTime() - it->time);
            schedule_B.erase(it);
            it = find_if(arrival_B.begin(), arrival_B.end(), find_id(B_head));
            waitingTime += max(0.0, Simulation::getTime() - it->time);
            B_head = B_IDs[0];
        }
        else if (B_IDs.empty() && !B_head.empty())
        {
            if (stoi(B_head.substr(B_head.find("_") + 1)) == beta)
            {
                // cout << Simulation::getTime() << " " << B_head << " leaves" << endl;
                vector<vehicle>::iterator it = find_if(schedule_B.begin(), schedule_B.end(), find_id(B_head));
                if (Vehicle::getRouteID(it->id) == "route_1")
                    passTime_dX = Simulation::getTime();
                else if (Vehicle::getRouteID(it->id) == "route_2")
                    passTime_dY = Simulation::getTime();
                // waitingTime += max(0.0, Simulation::getTime() - it->time);
                schedule_B.erase(it);
                it = find_if(arrival_B.begin(), arrival_B.end(), find_id(B_head));
                waitingTime += max(0.0, Simulation::getTime() - it->time);
                B_head = "";
            }
        }

        if (!C_IDs.empty() && C_IDs[0] != C_head)
        {
            // cout << Simulation::getTime() << " " << C_head << " leaves" << endl;
            passTime_dY = Simulation::getTime();
            vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
            // waitingTime += max(0.0, passTime_dY - it->time);
            schedule_C.erase(it);
            it = find_if(arrival_C.begin(), arrival_C.end(), find_id(C_head));
            waitingTime += max(0.0, Simulation::getTime() - it->time);
            C_head = C_IDs[0];
        }
        else if (C_IDs.empty() && !C_head.empty())
        {
            if (stoi(C_head.substr(C_head.find("_") + 1)) == gamma)
            {
                // cout << Simulation::getTime() << " " << C_head << " leaves" << endl;
                passTime_dY = Simulation::getTime();
                vector<vehicle>::iterator it = find_if(schedule_C.begin(), schedule_C.end(), find_id(C_head));
                // waitingTime += max(0.0, passTime_dY - it->time);
                schedule_C.erase(it);
                it = find_if(arrival_C.begin(), arrival_C.end(), find_id(C_head));
                waitingTime += max(0.0, Simulation::getTime() - it->time);
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
    }
    cout << max(passTime_dX, passTime_dY) << ", " << waitingTime / (alpha + beta + gamma) << endl;
    Simulation::close();
}
