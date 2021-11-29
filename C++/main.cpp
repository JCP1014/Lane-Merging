
#include "window_dp.h"
#include "group_dp.h"
using namespace std;

int main(int argc, char *argv[])
{
    ios::sync_with_stdio(0);
    cin.tie(0);

    string fileName;
    vector<float> allData;
    float timeStep = 1;
    // float W_same, W_diff;
    int alpha, beta, gamma;
    float p, pA, pB, pC;
    vector<float> a_all, b_all, c_all;

    if (argc >= 5)
    {
        W_same = atof(argv[3]);
        W_diff = atof(argv[4]);
        alpha = atoi(argv[2]);
        beta = atoi(argv[2]);
        gamma = atoi(argv[2]);
        p = atof(argv[1]);
        pA = p / 3;
        pB = p / 3;
        pC = p / 3;
        if (argc >= 6)
            fileName = argv[5];
    }
    else
    {
        cout << "Arguments: lambda, N, W=, W+" << endl;
        return 0;
    }

    if (argc >= 6)
    {
        // Read data from input file
        allData = read_data(fileName);
        alpha = int(allData[0]);
        beta = int(allData[0]);
        gamma = int(allData[0]);
        p = allData[1];
        allData.erase(allData.begin(), allData.begin() + 2);
        a_all.push_back(0);
        for (int i = 0; i < alpha; ++i)
            a_all.push_back(allData[i]);
        allData.erase(allData.begin(), allData.begin() + alpha);
        b_all.push_back(0);
        for (int i = 0; i < beta; ++i)
            b_all.push_back(allData[i]);
        allData.erase(allData.begin(), allData.begin() + beta);
        c_all.push_back(0);
        for (int i = 0; i < gamma; ++i)
            c_all.push_back(allData[i]);
        allData.erase(allData.begin(), allData.begin() + gamma);
    }
    else
    {
        // Generate data
        a_all = generate_traffic(timeStep, alpha, p, 0);
        b_all = generate_traffic(timeStep, beta, p, 1);
        c_all = generate_traffic(timeStep, gamma, p, 2);
    }

    // first_come_first_serve_v1(timeStep, a_all, b_all, c_all, W_same, W_diff);
    first_come_first_serve_v2(a_all, b_all, c_all);
    schedule_by_num_window_v2(a_all, b_all, c_all, 5);
    schedule_by_num_window_v2(a_all, b_all, c_all, 10);
    schedule_by_num_window_v2(a_all, b_all, c_all, 20);
    schedule_by_num_window_v2(a_all, b_all, c_all, 100);
    // schedule_by_reduced_dp(a_all, b_all, c_all, W_same, W_diff);
    // greedy_dp(a_all, b_all, c_all, W_same, W_diff);
    schedule_by_group(a_all, b_all, c_all, timeStep);

    // cout << "a_all = {" << a_all[0];
    // for (int i = 1; i < a_all.size(); ++i)
    //     cout << ", " << a_all[i];
    // cout << "};" << endl;
    // cout << "b_all = {" << b_all[0];
    // for (int i = 1; i < b_all.size(); ++i)
    //     cout << ", " << b_all[i];
    // cout << "};" << endl;
    // cout << "c_all = {" << c_all[0];
    // for (int i = 1; i < c_all.size(); ++i)
    //     cout << ", " << c_all[i];
    // cout << "};" << endl;

    // for (auto &g : grouped_a)
    //         cout << "(" << g.first << ", " << g.second << "), ";
    //     cout << endl;
    //     for (auto &g : grouped_b)
    //         cout << "(" << g.first << ", " << g.second << "), ";
    //     cout << endl;
    //     for (auto &g : grouped_c)
    //         cout << "(" << g.first << ", " << g.second << "), ";
    //     cout << endl;

    return 0;
}