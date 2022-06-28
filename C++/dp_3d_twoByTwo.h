#include <stack>
#include "utility.h"
#include "solution.h"
#include "fcfs.h"
#include "get_window.h"

tuple<tuple<char, int, double>, tuple<char, int, double>, double> window_oneSol_dp_v1(vector<double> a, vector<double> b, vector<double> c, double W_same, double W_diff, tuple<char, int, double> last_X, tuple<char, int, double> last_Y);
tuple<double, double> schedule_by_window_dp_v1(vector<double> a_all, vector<double> b_all, vector<double> c_all, double W_same, double W_diff, int carNum);