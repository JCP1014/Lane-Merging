
#include "generate_input.h"
#include "fcfs.h"
#include "window_dp.h"
#include "group_dp.h"
#include "dp_2d.h"
#include "reduced_dp.h"
#include "milp.h"
#include "group_milp.h"
using namespace std;

int main(int argc, char *argv[])
{
    ios::sync_with_stdio(0);
    cin.tie(0);

    string fileName;
    vector<double> allData;
    double timeStep = 1;
    // double W_same, W_diff;
    int alpha, beta, gamma;
    double p, pA, pB, pC;
    vector<double> a_all, b_all, c_all;

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

    // a_all = {0, 425, 496, 497, 510, 511, 513, 514, 515, 517, 518};
    // b_all = {0, 410, 436, 529, 530, 532, 533, 535, 536, 537, 539};
    // c_all = {0, 463, 464, 466, 467, 468, 481, 483, 484, 526, 527};
    a_all = {0, 331.746, 374.924, 376.291, 377.658, 379.024, 386.178, 387.548, 388.918, 390.288, 391.658, 393.028, 394.398, 406.196, 407.567, 408.938, 410.308, 411.679, 413.05, 414.42, 415.791, 417.162, 418.532, 419.903, 421.271, 422.625, 424, 425.4, 426.756, 454.035, 455.425, 456.815, 458.205, 459.595, 460.985, 462.375, 463.765, 465.154, 466.544, 467.934, 469.324, 470.714, 472.104, 473.494, 474.884, 476.274, 477.664, 479.054, 480.444, 481.833, 483.224, 484.613, 486.003, 487.393, 488.783, 490.173, 498.443, 506.232, 507.625, 509.017, 510.41, 511.803, 513.196, 514.588, 515.981, 517.374, 518.767, 520.159, 521.552, 522.945, 524.344, 525.729, 527.127, 528.516, 529.906, 531.301, 532.694, 534.087, 535.48, 536.872, 538.265, 539.658, 531.695, 539.356, 540.736, 542.117, 593.662, 595.091, 596.52, 597.949, 599.378, 600.807, 602.236, 603.665, 605.094, 606.522, 607.946, 609.378, 610.807, 612.238, 613.667};
    b_all = {0, 333.042, 335.055, 402.754, 404.145, 405.536, 406.927, 408.318, 409.709, 416.413, 417.807, 419.2, 420.594, 421.988, 423.381, 424.775, 426.169, 427.563, 428.956, 430.35, 431.744, 433.137, 434.531, 435.925, 437.318, 438.712, 440.106, 441.499, 442.893, 444.287, 445.673, 447.073, 448.467, 449.833, 451.256, 452.643, 454.048, 455.436, 456.828, 458.222, 459.617, 461.011, 462.404, 463.798, 465.192, 466.586, 483.506, 484.902, 486.297, 487.693, 489.088, 539.717, 541.161, 542.604, 544.047, 545.49, 546.933, 548.376, 549.819, 551.262, 552.705, 554.149, 555.592, 557.035, 558.478, 559.921, 561.364, 562.807, 564.25, 565.694, 567.137, 568.58, 570.023, 571.466, 572.909, 574.352, 575.795, 577.238, 578.682, 580.125, 581.568, 583.011, 584.454, 585.897, 587.34, 588.783, 590.226, 591.67, 593.113, 594.556, 595.999, 597.442, 598.885, 600.328, 632.102, 633.561, 635.019, 636.478, 637.936, 639.395, 640.853};
    c_all = {0, 327, 349.528, 350.875, 352.221, 353.568, 364.686, 366.037, 367.387, 387.083, 395.965, 397.338, 398.71, 400.083, 401.455, 402.828, 404.2, 405.573, 406.945, 420.145, 421.525, 422.905, 424.285, 425.665, 427.046, 428.426, 429.806, 431.186, 432.566, 433.947, 435.327, 436.707, 438.086, 439.467, 440.847, 442.228, 443.607, 444.988, 446.368, 447.749, 449.129, 450.509, 451.889, 453.27, 454.65, 456.03, 457.41, 458.79, 460.171, 461.551, 462.931, 464.311, 467.382, 468.748, 470.115, 471.481, 472.848, 480.05, 481.42, 482.789, 484.159, 485.529, 486.899, 488.269, 489.639, 491.009, 492.379, 493.749, 494.959, 496.324, 497.689, 499.053, 500.418, 501.783, 503.147, 504.512, 533.628, 535.019, 536.409, 537.8, 539.19, 540.581, 541.971, 543.362, 544.752, 546.143, 547.533, 548.924, 550.314, 596.417, 597.849, 599.281, 600.713, 602.145, 603.577, 605.009, 606.441, 607.873, 609.312, 610.737, 612.17};
    vector<tuple<double, double, double>> res;
    res.push_back(first_come_first_serve_v2(a_all, b_all, c_all));
    // res.push_back(solve_milp(a_all, b_all, c_all));
    // res.push_back(schedule_by_window_dp_v2(a_all, b_all, c_all, 100));
    // res.push_back(solve_group_milp(a_all, b_all, c_all, timeStep));
    // res.push_back(schedule_by_group_dp(a_all, b_all, c_all, timeStep));

    // res.push_back(schedule_by_reduced_dp(a_all, b_all, c_all));
    // res.push_back(greedy_dp(a_all, b_all, c_all));
    // res.push_back(schedule_by_window_dp_v2(a_all, b_all, c_all, 5));
    // res.push_back(schedule_by_window_dp_v2(a_all, b_all, c_all, 10));
    // res.push_back(schedule_by_window_dp_v2(a_all, b_all, c_all, 20));

    for (auto &tup : res)
        cout << get<0>(tup) << "," << get<1>(tup) << "," << get<2>(tup) << ",";
    cout << endl;
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