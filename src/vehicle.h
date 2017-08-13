#ifndef VEHICLE_H
#define VEHICLE_H
#include <map>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <algorithm>

using namespace std;

class Vehicle {

  // vector to remember the previous sensor fusion data
  vector< vector<double> > sensor_data_prev;

  // array of trajectories
  map<int, vector<double> > predictions;
  vector<double> lane_s;


public:

  struct collider
  {
    bool collision ; // is there a collision?
    int  time; // time collision happens
  };

  // Tunable variables
  int horizon = 2;      // how far we look into the future for collisions
  int buffer = 19;       // min distance between SDC and a bot

  double lane, s, v, a, max_acceleration;


  /*
   * Distance increment determines speed. Rate of change of dist_inc = acceleration
   * Car moves every 20ms, ie 50 times per second. So if dist_inc = 0.5m, then car speed = 50*0.5 = 25m/s
   * Want to limit acceleration to 10m/s^2 = 10*9/4mph = 22.5mph.
   *   ref_v will change by max 22.5mph/s = 22.5/50mph/cycle = 0.45mph/cycle
   */
  const double v_max = 22;    // 22m/s = 49.5mph. Conversion: *4/9
  const double dist_max = 0.44; // dist_max corresponds to the maximum acceleration
  double ref_v = 0.0;         // vehicle starts at 0m/s
  double bot_s = 0.0;       // speed of car in front
  double bot_v = 0.0;       // velocity of car in front

  string state;

  /*
  * Constructor
  */
  Vehicle(double lane, double s, double v, double a);

  /*
  * Destructor
  */
  virtual ~Vehicle();


  vector< vector<double> > generate_vehicle_trajectory(vector<double> previous_path_x, vector<double> previous_path_y,
               vector<double> maps_x, vector<double> maps_y, vector<double> maps_s,
               double car_x, double car_y, double car_s, double car_yaw);


  void bot_predictions(vector< vector<double> > sensor_data, double dt,
                       vector<double> maps_x, vector<double> maps_y);

  void update_state();

  vector<double> state_at(double t);

  bool collides_with(Vehicle other, double at_time);

  collider will_collide_with(Vehicle other, int timesteps);

};

#endif