#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

float calculate_cost(const Vehicle & vehicle, const map<int, Vehicle> & predictions, const trajectory_t & trajectory);

float distance_cost(const Vehicle & vehicle,  const trajectory_t & trajectory,  const map<int, Vehicle> & predictions, map<string, float> & data);

float inefficiency_cost(const Vehicle & vehicle, const trajectory_t & trajectory, const map<int, Vehicle> & predictions, map<string, float> & data);

//float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);

//map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions);

#endif
