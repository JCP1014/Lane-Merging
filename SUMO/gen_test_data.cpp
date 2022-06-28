#include <bits/stdc++.h>
#include "generate_input.h"

using namespace std;

int main(int argc, char *argv[])
{
    string fileName;
    float timeStep = 1;
    int N;
    float p;

    if (argc == 4)
    {
        p = atof(argv[1]);
        N = atoi(argv[2]);
        fileName = argv[3];
    }
    else
    {
        cout << "Arguments: lambda, N, fileName" << endl;
        return 0;
    }

    generate_routefile(timeStep, N, p, fileName);
    generate_configfile(fileName);
    return 0;
}