
#include "generate_input.h"
#include "utility.h"
#include "fafg.h"
#include "window_dp.h"
#include "group_dp.h"
#include "dp_2d.h"
#include "pruned_dp.h"
#include "milp.h"
#include "group_milp.h"
using namespace std;

double W_same, W_diff;  // W=, W+

int main(int argc, char *argv[])
{
    string fileName;    // input file
    vector<double> allData; // vector for storing data from input file 
    int alpha, beta, gamma; // number of vehicles
    double p; // trffic density (lambda in Poisson distribution)
    vector<double> A_all, B_all, C_all; // vector of earliest arrival times
    vector<tuple<double, double, double>> res;  // vector of results

    if (argc >= 5)
    {
        W_same = atof(argv[3]); // W=
        W_diff = atof(argv[4]); // W+
        alpha = atoi(argv[2]);  // number of vehicles on Lane A
        beta = atoi(argv[2]);   // number of vehicles on Lane B
        gamma = atoi(argv[2]);  // number of vehicles on Lane C
        p = atof(argv[1]);  // average number of incoming vehicles on each lane per second
        if (argc >= 6)
            fileName = argv[5]; // input file name
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
        A_all.push_back(0); // A_0 = 0
        for (int i = 0; i < alpha; ++i) // A_1 ~ A_alpha
            A_all.push_back(allData[i]);
        allData.erase(allData.begin(), allData.begin() + alpha);
        B_all.push_back(0); // B_0 = 0
        for (int i = 0; i < beta; ++i)  // B_1 ~ B_beta
            B_all.push_back(allData[i]);
        allData.erase(allData.begin(), allData.begin() + beta);
        C_all.push_back(0); // C_0 = 0
        for (int i = 0; i < gamma; ++i) // C_1 ~ C_gamma
            C_all.push_back(allData[i]);
        allData.erase(allData.begin(), allData.begin() + gamma);
    }
    else
    {
        // Generate data
        A_all = generate_traffic(W_same, alpha, p, 0);
        B_all = generate_traffic(W_same, beta, p, 1);
        C_all = generate_traffic(W_same, gamma, p, 2);
    }

    // Applying different appraoches to solve the problem and get results
    res.push_back(first_arrive_first_go(A_all, B_all, C_all));  // FAFG
    // res.push_back(solve_milp(A_all, B_all, C_all)); // MILP
    res.push_back(schedule_by_window_dp(A_all, B_all, C_all, 100));  // 3D DP
    // res.push_back(solve_group_milp(A_all, B_all, C_all));   // Grouping MILP
    res.push_back(schedule_by_group_dp(A_all, B_all, C_all));   // Grouping 3D DP

    res.push_back(schedule_by_pruned_dp(A_all, B_all, C_all)); // Pruned 3D DP
    res.push_back(dp_2d(A_all, B_all, C_all));  // 2D DP
    res.push_back(schedule_by_window_dp(A_all, B_all, C_all, 5));    // Window 3D DP (window size = 5)
    res.push_back(schedule_by_window_dp(A_all, B_all, C_all, 10));   // Window 3D DP (window size = 10)
    res.push_back(schedule_by_window_dp(A_all, B_all, C_all, 20));   // Window 3D DP (window size = 20)

    // Print results
    for (auto &tup : res)
        cout << get<0>(tup) << "," << get<1>(tup) << "," << get<2>(tup) << ",";
    cout << endl;

    return 0;
}