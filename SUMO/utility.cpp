#include "utility.h"

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

vector<pair<int, int>> grouping(vector<vehicle> &traffic)
{
    int groups = 1;
    int max_groups = 35;
    double grouping_threshold = 1.0;
    double timeStep = 0.5;
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

tuple<tuple<char, int, double>, double> schedule_single_lane(char lane, vector<vehicle> traffic, tuple<char, int, double> prev, vector<vehicle> &schedule)
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
