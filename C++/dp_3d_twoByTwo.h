#include <stack>
#include "solution.h"
#include "fcfs.h"
#include "get_window.h"

tuple<tuple<char, int, float>, tuple<char, int, float>, double> window_oneSol_dp_v1(vector<float> a, vector<float> b, vector<float> c, float W_same, float W_diff, tuple<char, int, float> last_X, tuple<char, int, float> last_Y);
tuple<float, double> schedule_by_window_dp_v1(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff, int carNum);