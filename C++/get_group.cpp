#include "get_group.h"

vector<pair<int, int>> grouping(vector<float> &traffic, float timeStep)
{
    int groups = 1;
    int max_groups = 35;
    float grouping_threshold = 1;
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
    return grouped_index;
}

vector<pair<int, int>> fixed_threshold_grouping(vector<float> &traffic, float timeStep)
{
    float grouping_threshold = 1;
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