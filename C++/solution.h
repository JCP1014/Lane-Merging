#include <string>
using namespace std;

class Solution { 
public: 
    Solution(float time[2], string table, string lane);
    float getObj();
    void setValue(float newTime[2], string newTable, string newLane);
    float time[2];
    string table;
    string lane;
};