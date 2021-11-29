#include "read_write.h"

vector<float> read_data(string fileName)
{
    fstream file;
    string line;
    float tmp;
    vector<float> allData;
    file.open(fileName);
    while (getline(file, line))
        allData.push_back(stof(line));
    file.close();
    return allData;
}

void write_result(string fileName, string title, float T_last, float computeTime)
{
    
}