#include "generate_input.h"
#include "utility.h"
#include "get_vehicle_info.h"
#include "solution.h"
#include "fafg.h"
#include "milp.h"
#include "dp.h"
#include "group_milp.h"
#include "group_dp.h"
#include "control.h"

using namespace std;

double W_same, W_diff;

int main(int argc, char *argv[])
{
    ios::sync_with_stdio(0);
    cin.tie(0);

    double p, pA, pB, pC;
    int N, alpha, beta, gamma;
    vector<double> A, B, C;
    char isNewTest; // whether to generate new test case

    if (argc >= 6)
    {
        p = atof(argv[1]);
        N = atoi(argv[2]);
        W_same = atof(argv[3]);
        W_diff = atof(argv[4]);
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
        string filePath(argv[6]);
        filePath.replace(filePath.begin(), filePath.begin() + 11, "output/output");
        filePath = filePath.substr(0, filePath.find("."));
        
        // FAFG
        Simulation::start({"sumo", "-c", argv[6],
                           "--tripinfo-output", filePath + "_fcfs.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
        run(FAFG, alpha, beta, gamma);
        
        // MILP
        Simulation::start({"sumo", "-c", argv[6],
                           "--tripinfo-output", filePath + "_milp.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
        run(MILP, alpha, beta, gamma);
        
        // 3D DP
        Simulation::start({"sumo", "-c", argv[6],
                           "--tripinfo-output", filePath + "_dp.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
        run(DP, alpha, beta, gamma);
        
        // Grouping MILP
        Simulation::start({"sumo", "-c", argv[6],
                           "--tripinfo-output", filePath + "_groupMILP.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
        run(GROUP_MILP, alpha, beta, gamma);
        
        // Grouping 3D DP
        Simulation::start({"sumo", "-c", argv[6],
                           "--tripinfo-output", filePath + "_groupDP.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
        run(GROUP_DP, alpha, beta, gamma);
    }
    else
    {
        if (isNewTest == 'T' || isNewTest == 't' || isNewTest == '1')
        {
            cout << "Generate a new test" << endl;
            generate_routefile(W_same, N, pA, pB, pC);
        }
        // FAFG
        Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                           "--tripinfo-output", "sumo_data/tripinfo_fcfs.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
        run(FAFG, alpha, beta, gamma);
        
        // MILP
        Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                           "--tripinfo-output", "sumo_data/tripinfo_milp.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
        run(MILP, alpha, beta, gamma);
        
        // 3D DP
        Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                           "--tripinfo-output", "sumo_data/tripinfo_dp.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
        run(DP, alpha, beta, gamma);
        
        // Grouping MILP
        Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                           "--tripinfo-output", "sumo_data/tripinfo_groupMILP.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
        run(GROUP_MILP, alpha, beta, gamma);
        
        // Grouping 3D DP
        Simulation::start({"sumo-gui", "-c", "sumo_data/laneMerging.sumocfg",
                           "--tripinfo-output", "sumo_data/tripinfo_DP.xml",
                           "-S",
                           "--no-step-log", "true",
                           "-W",
                           "--duration-log.disable", "true",
                           "--step-length", "0.5"});
        run(GROUP_DP, alpha, beta, gamma);
    }

    return 0;
}