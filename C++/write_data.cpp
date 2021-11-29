#include <iostream>
#include <fstream>
#include "generate_input.h"

using namespace std;

int main(int argc, char *argv[])
{
    string fileName;
    ofstream file;
    float timeStep = 1;
    float W_same, W_diff;
    int alpha, beta, gamma;
    float p, pA, pB, pC;
    vector<float> A, B, C;

    if (argc == 6)
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
        fileName = argv[5];
    }
    else
    {
        cout << "Arguments: lambda, N, W=, W+, fileName" << endl;
        return 0;
    }

    // Generate data
    A = generate_traffic(timeStep, alpha, p, 0);
    B = generate_traffic(timeStep, beta, p, 1);
    C = generate_traffic(timeStep, gamma, p, 2);

    // Write files
    file.open(fileName);
    file << alpha << endl;
    file << p << endl;
    for (int i = 1; i < A.size(); ++i)
        file << A[i] << endl;
    for (int i = 1; i < B.size(); ++i)
        file << B[i] << endl;
    for (int i = 1; i < C.size()-1; ++i)
        file << C[i] << endl;
    file << C[gamma];
    file.close();

    return 0;
}