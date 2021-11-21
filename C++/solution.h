#include <string>
#include <vector>
#include <cmath>
#include <tuple>
using namespace std;

struct Solution
{
    float time[2] = {INFINITY, INFINITY};
    string table = "";
    string lane = "";
    Solution *src = NULL;
};

Solution update_sol(Solution s, float newTimeX, float newTimeY, string newTable, string newLane);
Solution update_sol(Solution s, float newTimeX, float newTimeY, string newTable, string newLane, Solution *newSrc);
Solution choose_best_sol(Solution s, vector<Solution> solVec);
string get_opt_table(vector<Solution> solVec);