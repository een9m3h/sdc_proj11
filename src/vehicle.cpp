#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
//#include "cost.h"
#include "spline.h"

//planning window defines
#define SAMPLING_TIME 0.02
#define PLANNING_LEN  10 //secs
#define METER_PER_MILE 1609.34 
#define SEC_PER_HOUR 3600
#define TARGET_SPEED 49.5 // MPH
#define IN_RANGE     30.0 // consider cars in this range
#define MAX_ACC      5.0 // metres/sec^2
#define TARGET_DIST_CAR_AHEAD 10.0  //meters

/**
 * Initializes Vehicle
 */


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }



Vehicle::Vehicle(){}

Vehicle::Vehicle(double x, double y, double vx, double vy, double s, double d, string state){

	this->x = x;
	this->y = y;
	this->s = s;
	this->d = d;
	calc_yaw(vx, vy);
	calc_speed(vx, vy);


}

Vehicle::Vehicle(double x, double y , double s , double d, double yaw, double v){

	this->x = x;
	this->y = y;
	this->s = s;
	this->d = d;
	this->yaw = yaw;
	this->v = v;
	this->state = "KL";
}


Vehicle::Vehicle(int lane, float s, float v, float a, string state) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
   // this->
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}

void Vehicle::add_prev_path(vector<double> px, vector<double> py, double end_path_d, double end_path_s){

	this->previous_path_x 	= px;
	this->previous_path_y 	= py;
	this->end_path_d 	= end_path_d;
	this->end_path_s 	= end_path_s;
}

void Vehicle::update_waypoints(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s){

	this->map_waypoints_s = map_waypoints_s;
	this->map_waypoints_x = map_waypoints_x;
	this->map_waypoints_y = map_waypoints_y;
}

void Vehicle::initialize_trajectory(){

	//reference x,y,yaw states
	double ref_x = this->x;
	double ref_y = this->y;
	double ref_yaw = deg2rad(this->yaw);

	int prev_size = previous_path_x.size();

	//if previous size is almoust empty, use the car as starting reference
	if(prev_size < 2)
	{
		//use two points that make tangent to the car
		double prev_car_x = this->x-cos(this->yaw);
		double prev_car_y = this->y-sin(this->yaw);
		ptsx.push_back(prev_car_x);
		ptsx.push_back(this->x);
		ptsy.push_back(prev_car_y);
		ptsy.push_back(this->y);

	}
	//use previous path's endpoint as starting ref
	else
	{
		//Redefine ref state as prev path endpoint
		ref_x = previous_path_x[prev_size-1];
		ref_y = previous_path_y[prev_size-1];

		double ref_x_prev = previous_path_x[prev_size-2];
		double ref_y_prev = previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

		//Use two points that make the path tangent to the previous path's endpoint
		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}


}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}



double Vehicle::calc_speed(double vx, double vy){

	return sqrt(vx*vx+vy*vy);

}

double Vehicle::calc_yaw(double dx, double dy){

	return atan2(dy, dx);
}

double Vehicle::conv_mph_2_mps(double v){
	/*
	 * Converts from MPH to metres/sec
	 * */
	return (v*METER_PER_MILE)/SEC_PER_HOUR;

}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;

    /*for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);*/
    int best_idx = 0;
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    /*string state = this->state;
    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        if (lane != lanes_available - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane != 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }*/
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    //float max_velocity_accel_limit = this->max_acceleration + this->v;
    float new_time_in_acc;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead) && vehicle_ahead.v < TARGET_SPEED) {

  	    float speed_delta = conv_mph_2_mps(vehicle_ahead.v - this->v);
	    float target_dist = vehicle_ahead.s - this->s - TARGET_DIST_CAR_AHEAD; //how much we can catch up to next car
	    if(speed_delta >= 0.0 && target_dist >= 0.0){
		//accelerate to within 10m of vehicle ahead and maintain same speed as vehicle ahead
	    	new_time_in_acc = abs(2*target_dist)/abs(speed_delta);
	    	new_accel 	= speed_delta/new_time_in_acc;
		new_accel 	= new_accel > MAX_ACC ? MAX_ACC : new_accel; //limit accelateration
	        new_time_in_acc = speed_delta/new_accel;	// re-compute time in acc if saturated acc
		new_velocity 	= vehicle_ahead.v;
	    }
	    else if(speed_delta < 0.0 && target_dist >= 0.0){
		//decelerate to achieve car ahead speed at target distance
		new_time_in_acc = abs(2*target_dist)/abs(speed_delta);
	    	new_accel 	= speed_delta/new_time_in_acc;
		new_accel 	= new_accel < -MAX_ACC ? -MAX_ACC : new_accel; //limit accelateration
	        new_time_in_acc = speed_delta/-new_accel;	// re-compute time in acc if saturated acc
		new_velocity 	= vehicle_ahead.v;
	    }
	    else if(speed_delta >= 0.0 && target_dist < 0.0){
	        //allow steady acceleration upto ahead vehicle whilst increating safety distance
		new_time_in_acc = abs(2*target_dist)/abs(speed_delta);
	    	new_accel 	= speed_delta/new_time_in_acc;
		new_accel 	= new_accel > MAX_ACC ? MAX_ACC : new_accel; //limit accelateration
	        new_time_in_acc = speed_delta/new_accel;	// re-compute time in acc if saturated acc
		new_velocity 	= vehicle_ahead.v;
	
	    }
	    else if(speed_delta < 0.0 && target_dist < 0.0){
		//decelerate to hit buffer distance in 1 planning period
		new_time_in_acc = PLANNING_LEN;
		float t1 = ((target_dist*target_dist)*abs(speed_delta))/(new_time_in_acc*2.0 + target_dist*2.0*abs(speed_delta));
		new_accel = speed_delta/t1;
		new_velocity = conv_mph_2_mps(this->v) + new_accel*new_time_in_acc;
			
	    }
   




        /*if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
        } else {
            float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }*/
    } else {
	//maximum acceleration until target speed reached    
        new_velocity 	= TARGET_SPEED; //target speed in MPH
	new_accel    	= this->v >= TARGET_SPEED ? 0.0: MAX_ACC;  //target acc in m/s^2
        new_time_in_acc = min(conv_mph_2_mps(TARGET_SPEED - this->v)/new_accel, 0.0); 	
    }
    
    //new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    //new_position = this->s + new_velocity + new_accel / 2.0;
    return{new_time_in_acc, new_velocity, new_accel};
    
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    float next_pos = position_at(1);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state), 
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    
    /*vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;*/

	vector<float> kinematics = get_kinematics(predictions, this->lane);
    	float new_s = kinematics[0];
    	float new_v = kinematics[1];
    	float new_a = kinematics[2];


	//In Fresnet add evenly 30m spaced points ahread of the starting reference
	vector<double> next_wp0 = getXY(this->s+30, (2+4*this->lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(this->s+60, (2+4*this->lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(this->s+90, (2+4*this->lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_wp0[0]);	
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);	
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for(int i = 0; i < ptsx.size(); i++)
	{
		//shift car ref angle to 0 deg
		double shift_x = ptsx[i]-this->x;
		double shift_y = ptsy[i]-this->x;

		ptsx[i] = (shift_x*cos(0-this->yaw)-shift_y*sin(0-this->yaw));
		ptsy[i] = (shift_x*sin(0-this->yaw)+shift_y*cos(0-this->yaw));

	}
	
	//create spline 
	tk::spline s;

	//set points to the spline
	s.set_points(ptsx, ptsy);

	//Declare actual points
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	//start with all residual points from last plan
	for(int i = 0; i < previous_path_x.size(); i++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);		
	}	

	//calculate how to break up spline points to travel at desired velocity
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

	double x_add_on = 0;

	//fill up rest of the path after adding previous points, output 50 pts
	for(int i = 1; i <= 50-previous_path_x.size(); i++)
	{
		double N = (target_dist/(0.02*this->v/2.24));
		double x_point = x_add_on+target_x/N;
		double y_point = s(x_point);
		
		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		//rotate back to sim co-ords
		x_point = (x_ref*cos(this->yaw)-y_ref*sin(this->yaw));
		y_point = (x_ref*sin(this->yaw)+y_ref*cos(this->yaw));

		x_point += this->x;
		y_point += this->y;
		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);

	}

	
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
        
    } else {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}

void Vehicle::increment(int dt = 1) {
	this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
    return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    float min_distance = IN_RANGE;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
	float curr_distance = (this->s - temp_vehicle.s);
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && curr_distance < IN_RANGE) {
            min_distance = curr_distance;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	float min_distance = IN_RANGE;
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		temp_vehicle = it->second[0];
		float curr_distance = (temp_vehicle.s - this->s);  
		if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && curr_distance < IN_RANGE) {
			min_distance = curr_distance;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}

	return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
	vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      float next_s = position_at(i);
      float next_v = 0;
      if (i < horizon-1) {
        next_v = position_at(i+1) - s;
      }
      predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  	}
    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}

void Vehicle::configure(vector<int> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}
