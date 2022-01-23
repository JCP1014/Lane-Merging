#include <iostream>
#include <fstream>
#include <random>
#include <time.h>
using namespace std;

void generate_routefile(double timeStep, double p, int N, string fileName)
{
    ofstream file;
    int num_A = 1, num_B = 1, num_C = 1;
    double t = 1.0;
    default_random_engine generator(time(NULL));
    uniform_real_distribution<double> unif(0.0, 1.0);

    file.open("input/" + fileName + ".rou.xml");
    file << "<routes>" << endl;
    file << "    <vType id=\"typeA\" type=\"passenger\" length=\"2\" accel=\"1.5\" decel=\"2\" sigma=\"0.0\" maxSpeed=\"20\" color=\"yellow\"/>" << endl;
    file << "    <vType id=\"typeB\" type=\"passenger\" length=\"2\" accel=\"1.5\" decel=\"2\" sigma=\"0.0\" maxSpeed=\"20\" color=\"blue\"/>" << endl;
    file << "    <vType id=\"typeC\" type=\"passenger\" length=\"2\" accel=\"1.5\" decel=\"2\" sigma=\"0.0\" maxSpeed=\"20\" color=\"magenta\"/>" << endl;
    file << "    <route edges=\"A X\" color=\"yellow\" id=\"route_0\"/>" << endl;
    file << "    <route edges=\"B X\" color=\"yellow\" id=\"route_1\"/>" << endl;
    file << "    <route edges=\"B Y\" color=\"yellow\" id=\"route_2\"/>" << endl;
    file << "    <route edges=\"C Y\" color=\"yellow\" id=\"route_3\"/>" << endl;

    while (num_A <= N || num_B <= N || num_C <= N)
    {
        if (num_A <= N && unif(generator) < p)
            file << "    <vehicle id=\"A_" << num_A++ << "\" type=\"typeA\" route=\"route_0\" depart=\"" << t << "\" departLane=\"0\" departSpeed=\"random\"/>" << endl;
        if (num_B <= N && unif(generator) < p)
            file << "    <vehicle id=\"B_" << num_B++ << "\" type=\"typeB\" route=\"route_1\" depart=\"" << t << "\" departLane=\"0\" departSpeed=\"random\"/>" << endl;
        if (num_C <= N && unif(generator) < p)
            file << "    <vehicle id=\"C_" << num_C++ << "\" type=\"typeC\" route=\"route_3\" depart=\"" << t << "\" departLane=\"0\" departSpeed=\"random\"/>" << endl;
        t += timeStep;
    }
    file << "</routes>" << endl;
    file.close();
}

void generate_configfile(string fileName)
{
    ofstream file;
    file.open("input/" + fileName + ".sumocfg");
    string num = fileName.substr(fileName.find("/") + 1);
    file << "<configuration>" << endl;
    file << "    <input>" << endl;
    file << "        <net-file value=\"../../sumo_data/laneMerging2.net.xml\"/>" << endl;
    file << "        <route-files value=\"" + num + ".rou.xml\"/>" << endl;
    file << "        <additional-files value=\"../../sumo_data/laneMerging.add.xml\"/>" << endl;
    file << "        <gui-settings-file value=\"../../sumo_data/laneMerging.view.xml\"/>" << endl;
    file << "    </input>" << endl;
    file << "</configuration>" << endl;
}

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

    generate_routefile(timeStep, p, N, fileName);
    generate_configfile(fileName);
    return 0;
}
