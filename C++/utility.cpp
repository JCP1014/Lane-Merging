#include "utility.h"

vector<double> read_data(string fileName)
{
    fstream file;
    string line;
    double tmp;
    vector<double> allData;
    file.open(fileName);
    while (getline(file, line))
        allData.push_back(stof(line));
    file.close();
    return allData;
}

vector<double> get_window_by_num(vector<double> &traffic, int carNum)
{
    if (carNum >= traffic.size())
        carNum = traffic.size() - 1;
    vector<double> subtraffic(traffic.begin(), traffic.begin() + carNum + 1);
    traffic.erase(traffic.begin() + 1, traffic.begin() + carNum + 1);
    return subtraffic;
}

vector<pair<int, int>> grouping(vector<double> &traffic)
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
            if (traffic[i] - traffic[i - 1] > grouping_threshold)
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
            grouping_threshold += 0.5;
            groups = 0;
            grouped_index.clear();
        }
        else
        {
            // cout << "threshold: " << grouping_threshold << ",  number of groups: " << groups << endl;
            break;
        }
    }
    // for(auto &g:grouped_index)
    //     cout << g.first << ", " << g.second << endl;
    return grouped_index;
}

vector<pair<int, int>> fixed_threshold_grouping(vector<double> &traffic)
{
    double grouping_threshold = 1;
    double timeStep = 0.5;
    vector<pair<int, int>> grouped_index;

    grouped_index.push_back({0, 0});
    int head = 1, tail = 0;
    for (int i = 2; i < traffic.size(); ++i)
    {
        if (traffic[i] - traffic[i - 1] > grouping_threshold)
        {
            tail = i - 1;
            grouped_index.push_back({head, tail});
            head = i;
        }
    }
    if (head == traffic.size() - 1)
        grouped_index.push_back({head, head});
    else
        grouped_index.push_back({head, traffic.size() - 1});
    return grouped_index;
}

tuple<tuple<char, int, double>, double> schedule_single_lane(char lane, vector<double> traffic, tuple<char, int, double> prev)
{
    char prevLane = get<0>(prev);
    double prevTime = get<2>(prev);
    tuple<char, int, double> last = make_tuple('0', 0, 0.0);
    double wait_time = 0;
    if (prevLane == '0')
    {
        last = make_tuple(lane, 1, traffic[0]);
    }
    else if (lane == prevLane)
    {
        last = make_tuple(lane, 1, max(traffic[0], prevTime + W_same));
    }
    else
    {
        last = make_tuple(lane, 1, max(traffic[0], prevTime + W_diff));
    }
    wait_time += (get<2>(last) - traffic[0]);
    // cout << get<2>(last) << " " << lane << endl;
    for (int i = 1; i < traffic.size(); ++i)
    {
        prevTime = get<2>(last);
        last = make_tuple(lane, i, max(traffic[i], prevTime + W_same));
        wait_time += (get<2>(last) - traffic[i]);
        // cout << get<2>(last) << " " << lane << endl;
    }
    return make_tuple(last, wait_time);
}
