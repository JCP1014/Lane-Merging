#ifndef GENERATE_INPUT_H
#define GENERATE_INPUT_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include <time.h>
using namespace std;

void generate_routefile(double timeStep, int N, double pA, double pB, double pC);
void generate_routefile(double timeStep, int N, double p, string fileName);
void generate_configfile(string fileName);

#endif