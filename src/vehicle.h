#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

//planning window defines
#define SAMPLES_PER_SEC 50
#define SAMPLING_TIME 0.02
#define PLANNING_LEN  2.0 //secs
#define METER_PER_MILE 1609.34 
#define SEC_PER_HOUR 3600
#define TARGET_SPEED 45.0 // MPH
#define IN_RANGE     60.0 // consider cars in this range
#define MAX_ACC      5.0 // metres/sec^2
#define TARGET_DIST_CAR_AHEAD 5.0  //meters



struct trajectory_t {

	vector<double> 	next_x_vals;
	vector<double> 	next_y_vals;
	double 		final_velocity;
	float 		vehicle_ahead_dist;
	float 		vehicle_behind_dist;
	int 		start_lane;
	int 		end_lane;
};


class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  map<int, Vehicle> vehicles;

  vector<double> previous_path_x;

  vector<double> previous_path_y;

  vector<double> ptsx;  //x-points to give to sim for path planning
  
  vector<double> ptsy;  //y-points to give to sim for path planning

  //Declare actual points to be given to simulator
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;

  double end_path_d;
  double end_path_s; 
  double end_path_x;
  double end_path_y;
  double end_path_yaw;
  double end_path_v;

  double s;

  double d;

  double x;

  double y;

  double yaw;

  float v;

  float a;

  float target_speed;

  int lanes_available;

  float max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, float s, float v, float a, string state="CS");
  Vehicle(double x, double y , double s , double d, double yaw, double v, string state="CS");
  //Vehicle(double x, double y, double vx, double vy, double s, double d);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  int calc_lane_from_d(double d);

  void update_waypoints(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s);

  void initialize_trajectory();

  void add_prev_path(vector<double> px, vector<double> py, double end_path_d, double end_path_s);

  static double calc_speed(double vx, double vy);

  static double calc_yaw(double dx, double dy);

  double conv_mph_2_mps(double v);
 
  double conv_mps_2_mph(double v);



  void choose_next_state(map<int, Vehicle> vehicles, vector<double> &next_x_vals, vector<double> &next_y_vals);

  vector<string> successor_states();

  //vector<Vehicle>
  vector<trajectory_t>  generate_trajectory(string state);

  vector<float> get_kinematics( int lane);

  vector<Vehicle> constant_speed_trajectory();

  trajectory_t keep_lane_trajectory();

  trajectory_t lane_change_trajectory(string state,  int target_lane);

  vector<Vehicle> prep_lane_change_trajectory(string state);

  void increment(float dt);

  float position_at(float t);

  bool get_vehicle_behind( int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead( int lane, Vehicle & rVehicle);

  vector<double> generate_predictions(float horizon = 1.0);

  void realize_next_state(vector<Vehicle> trajectory);

  void configure(vector<int> road_data);

  void update_predictions(int samples);

};

#endif
