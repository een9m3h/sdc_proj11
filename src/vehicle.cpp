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
#define PLANNING_LEN  10.0 //secs
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

	std::cout << "func: add_prev_path" << std::endl;

	this->previous_path_x 	= px;
	this->previous_path_y 	= py;
	this->end_path_d 	= end_path_d;
	this->end_path_s 	= end_path_s;

	//initialize first few points for continuity between planned paths
	this->initialize_trajectory();

	std::cout << "func end: add_prev_path" << std::endl;
}

void Vehicle::update_waypoints(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s){

	std::cout << "func: update_waypoints" << std::endl;

	this->map_waypoints_s = map_waypoints_s;
	this->map_waypoints_x = map_waypoints_x;
	this->map_waypoints_y = map_waypoints_y;

	std::cout << "func end: update_waypoints" << std::endl;
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);
	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}
void Vehicle::initialize_trajectory(){

	std::cout << "func: initialize_trajectory" << std::endl;

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

	vector<double> sd_points = getFrenet(ptsx.back(), ptsy.back(), ref_yaw, map_waypoints_x, map_waypoints_y);
	this->end_path_s = sd_points[0];
	this->end_path_d = sd_points[1];

	std::cout << "func end: initialize_trajectory" << std::endl;

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

void Vehicle::choose_next_state(map<int, Vehicle> vehicles, vector<double> &next_x_vals, vector<double> &next_y_vals) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    std::cout << "Func: choose_next_state" << std::endl;


    /*
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    */
    vector<vector<Vehicle>> final_trajectories;
    
    generate_trajectory("KL", vehicles);


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
    //int best_idx = 0;
    //return final_trajectories[best_idx];
   
    next_x_vals = this->next_x_vals;
    next_y_vals = this->next_y_vals;

    std::cout << "func end: choose_next_state" << std::endl;
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */

    std::cout << "Func: successor_states" << std::endl;


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

//vector<Vehicle>
void Vehicle::generate_trajectory(string state, map<int, Vehicle> vehicles) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */

    std::cout << "Func: generate_trajectory" << std::endl;


    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        //trajectory = 
	keep_lane_trajectory(vehicles);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, vehicles);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, vehicles);
    }

    std::cout << "func end: generate_trajectory" << std::endl;
    //return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, Vehicle> vehicles, int lane) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    //float max_velocity_accel_limit = this->max_acceleration + this->v;
    
    std::cout << "Func: get_kinematics" << std::endl;

    
    float new_time_in_acc;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(vehicles, lane, vehicle_ahead) && vehicle_ahead.v < TARGET_SPEED) {


  	    float speed_delta = conv_mph_2_mps(vehicle_ahead.v - this->v);
	    float target_dist = vehicle_ahead.s - this->s - TARGET_DIST_CAR_AHEAD; //how much we can catch up to next car
	    if(speed_delta >= 0.0 && target_dist >= 0.0){
		//accelerate to within 10m of vehicle ahead and maintain same speed as vehicle ahead
  	        std::cout << "accelerate to within 10m of vehicle ahead and maintain same speed as vehicle ahead" << std::endl;
		new_time_in_acc = abs(2*target_dist)/abs(speed_delta);
	    	new_accel 	= speed_delta/new_time_in_acc;
		new_accel 	= new_accel > MAX_ACC ? MAX_ACC : new_accel; //limit accelateration
	        new_time_in_acc = speed_delta/new_accel;	// re-compute time in acc if saturated acc
		new_velocity 	= vehicle_ahead.v;
	    }
	    else if(speed_delta < 0.0 && target_dist >= 0.0){
		//decelerate to achieve car ahead speed at target distance
		std::cout << "decelerate to achieve car ahead speed at target distance" << std::endl;
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
	std::cout << "maximum acceleration until target speed reached" << std::endl;
	new_velocity 	= TARGET_SPEED; //target speed in MPH
	new_accel    	= this->v >= TARGET_SPEED ? 0.0: MAX_ACC;  //target acc in m/s^2
        new_time_in_acc = min(max(conv_mph_2_mps(TARGET_SPEED - this->v)/new_accel, 0.0), PLANNING_LEN); 	
    }


    std::cout << "Func end: get_kinematics" << std::endl;
    
    //new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    //new_position = this->s + new_velocity + new_accel / 2.0;
    return{new_time_in_acc, new_velocity, new_accel};
    
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */

    std::cout << "Func: constant_speed_trajectory" << std::endl;

    float next_pos = position_at(1);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state), 
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}

void Vehicle::keep_lane_trajectory(map<int, Vehicle> vehicles) {
    /*
    Generate a keep lane trajectory.
    */

    std::cout << "Func: keep_lane_trajectory" << std::endl;

    
    /*vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;*/

	vector<float> kinematics = get_kinematics(vehicles, this->lane);
    	float new_t = kinematics[0];
    	float new_v = kinematics[1];
    	float new_a = kinematics[2];

	std::cout << "kinematics: s=" << new_t << ", v=" << new_v << ", a=" << new_a << std::endl;

	//In Fresnet add evenly 30m spaced points ahread of the starting reference
	vector<double> next_wp0 = getXY(this->end_path_s+30, this->end_path_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(this->end_path_s+60, this->end_path_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(this->end_path_s+90, this->end_path_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	std::cout << "Added 30/60/90 points" << std::endl; 

	ptsx.push_back(next_wp0[0]);	
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);	
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);


	for(int i = 0; i < ptsx.size(); i++)
	{
		std::cout << "bf x: " << ptsx[i] << "   bf y: " << ptsy[i] << std::endl;
	}

	for(int i = 0; i < ptsx.size(); i++)
	{
		//shift car ref angle to 0 deg
		double shift_x = ptsx[i]-this->x;
		double shift_y = ptsy[i]-this->y;

		ptsx[i] = (shift_x*cos(0-this->yaw)-shift_y*sin(0-this->yaw));
		ptsy[i] = (shift_x*sin(0-this->yaw)+shift_y*cos(0-this->yaw));

	}
	
	std::cout << "shift points to car's co-ord sys" << std::endl;

	for(int i = 0; i < ptsx.size(); i++)
	{
		std::cout << "bf x: " << ptsx[i] << "   bf y: " << ptsy[i] << std::endl;
	}

	//create spline 
	tk::spline s;

	//set points to the spline
	s.set_points(ptsx, ptsy);

        std::cout << "set points to the spline" << std::endl;

	//Declare actual points
	//vector<double> next_x_vals;
	//vector<double> next_y_vals;

	//start with all residual points from last plan
	for(int i = 0; i < previous_path_x.size(); i++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);		
	}	


        std::cout << "Added all residual points from last plan" << std::endl;

	//calculate how to break up spline points to travel at desired velocity
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

	double x_add_on = 0;


        std::cout << "begin fgilling remaining points" << std::endl;

	double current_speed_mps = conv_mph_2_mps(this->v);
	double current_planning_time = 0.0;

	//fill up rest of the path after adding previous points, output 50 pts
	for(int i = 1; i <= 50-previous_path_x.size(); i++)
	{
		double N; 
		if(current_planning_time < new_t){
			double accel_dist_comp = new_a > 0.0 ? 0.5*SAMPLING_TIME*SAMPLING_TIME*new_a: -0.5*SAMPLING_TIME*SAMPLING_TIME*new_a; 
			N = (target_dist/(SAMPLING_TIME*current_speed_mps + accel_dist_comp));
			current_speed_mps += new_a*SAMPLING_TIME;
		//	std::cout << "accelerating" << std::endl;		
		}else{
			N = (target_dist/current_speed_mps);
		//	std::cout << "const speed" << std::endl;
		}
		current_planning_time += SAMPLING_TIME;
		//std::cout << "Current speed: " << current_speed_mps << " m/s^2   Current time: " << current_planning_time << std::endl;
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

		//std::cout << "next_x_vals: " << x_point << " next_y_vals: " << y_point << std::endl;			

	}

	
        std::cout << "Func end: keep_lane_trajectory" << std::endl;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, Vehicle> vehicles) {
    /*
    Generate a trajectory preparing for a lane change.
    */

    std::cout << "Func: prep_lane_change_trajectory" << std::endl;


    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    vector<float> curr_lane_new_kinematics = get_kinematics(vehicles, this->lane);

    if (get_vehicle_behind(vehicles, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
        
    } else {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(vehicles, new_lane);
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

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, Vehicle> vehicles) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    /*Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = vehicles.begin(); it != vehicles.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<float> kinematics = get_kinematics(vehicles, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    */
    return trajectory;
}

void Vehicle::increment(int dt = 1) {
	this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
    return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, Vehicle> vehicles, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */

    std::cout << "Func: get_vehicle_behind" << std::endl;


    float min_distance = IN_RANGE;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, Vehicle>::iterator it = vehicles.begin(); it != vehicles.end(); ++it) {
        temp_vehicle = it->second;
	float curr_distance = (this->s - temp_vehicle.s);
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && curr_distance < IN_RANGE) {
            min_distance = curr_distance;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, Vehicle> vehicles, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */

   	std::cout << "Func: get_vehicle_ahead" << std::endl;
  

	bool found_vehicle = false;
	Vehicle temp_vehicle;
	float min_distance = IN_RANGE;
	for (map<int, Vehicle>::iterator it = vehicles.begin(); it != vehicles.end(); ++it) {
		std::cout << "Vehicle id: " << it->first << std::endl;
		temp_vehicle = it->second;
		float curr_distance = (temp_vehicle.s - this->s);  
		if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && curr_distance < IN_RANGE) {
			std::cout << "found vehicle" << std::endl;
			min_distance = curr_distance;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}

	std::cout << "Func end: get_vehicle_ahead" << std::endl;


	return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
	vector<Vehicle> vehicles;
    for(int i = 0; i < horizon; i++) {
      float next_s = position_at(i);
      float next_v = 0;
      if (i < horizon-1) {
        next_v = position_at(i+1) - s;
      }
      vehicles.push_back(Vehicle(this->lane, next_s, next_v, 0));
  	}
    return vehicles;

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
