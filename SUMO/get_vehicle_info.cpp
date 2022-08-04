#include "get_vehicle_info.h"

// Estimate earliest arrival times of vehicles on a lane
vector<vehicle> compute_earliest_arrival(double laneLength, vector<vehicle> &schedule, char lane)
{
    vector<vehicle> arrival_times = {vehicle()};    // vector of earliest arrival times
    double currentTime = Simulation::getTime(); // current time in the simulation
    vector<string> vehicle_ids = LaneArea::getLastStepVehicleIDs(string("d") + lane);   // vector of IDs of vehicles on a lane

    for (auto vehID : vehicle_ids)
    {
        double dist = laneLength - Vehicle::getDistance(vehID); // distance from the vehicle to the merging intersection
        double speed = Vehicle::getSpeed(vehID);    // speed of the vehicle
        double accel = Vehicle::getAcceleration(vehID); // acceleration of the vehicle
        double arrivalTime = 0; // earliset arrival time of the vehicle
        
        if (speed == 0) // if the vehicle stops
        {   // Use the vehicle's scheduled entering time which is computed before as its earliest arrival time
            vector<vehicle>::iterator it = find_if(schedule.begin(), schedule.end(), find_id(vehID));
            if (it != schedule.end())
                arrivalTime = it->time;
        }
        else
        {
            // Estimate the vehicle's earliest arrival time based on its speed, acceleration, and the distance to the merging intersection
            if (accel != 0)
                arrivalTime = currentTime + (sqrt(max(0.0, speed * speed + 2 * accel * dist)) - speed) / accel;
            else
                arrivalTime = currentTime + dist / speed;
                
            vector<vehicle>::iterator it = find_if(schedule.begin(), schedule.end(), find_id(vehID));
            if (it != schedule.end())
                arrivalTime = min(arrivalTime, it->time);
        }
        arrival_times.push_back(vehicle(vehID, arrivalTime));
    }
    sort(arrival_times.begin() + 1, arrival_times.end(), sort_vehicle); // sort the vector based on vehicle's ID
    for (int i = 1; i < arrival_times.size(); ++i)
        arrival_times[i].time = max(arrival_times[i].time, arrival_times[i - 1].time + W_same); // earliest arrival times must satisfy the requirement of W=
    return arrival_times;
}

// Print out the schedule of vehicles on a lane
void print_schedule(vector<vehicle> schedule, string lane)
{
    cout << "---------- schedule_" << lane << " " << schedule.size() << " ----------" << endl;
    for (auto veh : schedule)
    {
        cout << veh.id << " " << veh.time << " " << Vehicle::getRouteID(veh.id) << endl;
    }
}
