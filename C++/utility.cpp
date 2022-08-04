#include "utility.h"

// Read data from the input file
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

// Get a window of vehicles from a lane
vector<double> get_window_by_num(vector<double> &traffic, int carNum)
{
    if (carNum >= traffic.size())
        carNum = traffic.size() - 1;
    vector<double> subtraffic(traffic.begin(), traffic.begin() + carNum + 1);
    traffic.erase(traffic.begin() + 1, traffic.begin() + carNum + 1);   // Remove from the original vector
    return subtraffic;
}

// Group the vehicles and return the vector of starting index and the ending index of each group
vector<pair<int, int>> grouping(vector<double> &traffic)
{
    int groups = 1; // number of groups
    int max_groups = 35;    // maximum number of groups
    double grouping_threshold = 1.0;    // initial grouping threshold
    double timeStep = 0.5;  // the value added to the grouping threshold per iteration 
    vector<pair<int, int>> group_index;   // vector of pairs of the starting index and ending index of each groups


    while (1)
    {
        group_index.push_back({0, 0});  // group_0 doesn't exist
        int head = 1; // strating index of a group
        int tail = 0; // ending index of a group
        for (int i = 2; i < traffic.size(); ++i)
        {
            // If the headway between two consecutive vehicles is larger than the grouping threshold,
            // Then groups + 1
            if (traffic[i] - traffic[i - 1] > grouping_threshold)
            {
                if (++groups > max_groups)  // if the number of groups exceeds the maximum number of groups
                    break;                  
                tail = i - 1;
                group_index.push_back({head, tail});
                head = i;   // update the starting index for the next group
            }
        }
        if (head == traffic.size() - 1) // the last vehicle
            group_index.push_back({head, head});
        else
            group_index.push_back({head, traffic.size() - 1});
            
        // If the number of groups exceeds the maximum number of groups
        if (groups > max_groups) 
        {
            // Then increase the grouping threshold and clear groups for regrouping
            grouping_threshold += timeStep;
            groups = 0;
            group_index.clear();
        }
        // Else finish grouping
        else
            break;
    }

    return group_index;
}

// Schedule vehicles of a single lane, without inserting any vehicles of other lanes
tuple<tuple<char, int, double>, double> schedule_single_lane(char lane, vector<double> traffic, tuple<char, int, double> prev)
{
    char prevLane = get<0>(prev);   // which lane the previous vehicle comes from
    double prevTime = get<2>(prev); // scheduled entering time of the previous vehicle
    tuple<char, int, double> last = make_tuple('0', 0, 0.0);    // (incoming lane, index, scheduled entering time) of the last vehicle
    double wait_time = 0;   // difference between scheduled entering time and earliest arrival time
    
    // For the first vehicle in the vector
    if (prevLane == '0')    // there is no previous vehicle
    {
        last = make_tuple(lane, 1, traffic[0]);
    }
    else if (lane == prevLane)  // the previous vehicle is from the same incoming lane
    {
        last = make_tuple(lane, 1, max(traffic[0], prevTime + W_same));
    }
    else    // the previous vehicle is from a different incoming lane
    {
        last = make_tuple(lane, 1, max(traffic[0], prevTime + W_diff));
    }
    wait_time += (get<2>(last) - traffic[0]);

    // From the second to the last vehicle
    for (int i = 1; i < traffic.size(); ++i)
    {
        prevTime = get<2>(last);
        last = make_tuple(lane, i, max(traffic[i], prevTime + W_same));
        wait_time += (get<2>(last) - traffic[i]);
    }

    return make_tuple(last, wait_time);
}
