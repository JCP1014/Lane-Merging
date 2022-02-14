#include "read_write.h"

vector<double> read_data(string fileName)
{
    fstream file;
    string line;
    double tmp;
    vector<double> allData;
    file.open(fileName);
    while (getline(file, line))
        allData.push_back(stof(line));
    file.close();
    return allData;
}

void write_result(string fileName, string title, double T_last, double computeTime)
{
    
}