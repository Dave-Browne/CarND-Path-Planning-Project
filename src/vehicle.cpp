#include "vehicle.h"
#include "helper.h"

Vehicle::Vehicle(double lane, double s, double v, double a) {
    this->lane = lane;    //  0=left, 1=middle, 2=right
    this->s = s;          // in meters
    this->v = v;          // in meters/second
    this->a = a;          // in meters/second^2
    state = "KL";
}

Vehicle::~Vehicle() {}



vector< vector<double> > Vehicle::generate_vehicle_trajectory(vector<double> previous_path_x, vector<double> previous_path_y,
               vector<double> maps_x, vector<double> maps_y, vector<double> maps_s, 
               double car_x, double car_y, double car_s, double car_yaw) 
{
  /*
   * Generate the vehicle trajectory by:
   * 1) Store n points from previous path to new path
   * 2) Get the vehicles current position or last trajectory position
   * 3) Find the closest waypoint
   * 4) Generate Spline trajectory from waypoints
   */
  vector<double> next_x_vals, next_y_vals, next_xy0, next_xy1, next_xy2, ptsx, ptsy;
  double pos_x, pos_y, pos_x2, pos_y2, angle_xy;
  int prev_path_size = previous_path_x.size();


  // Store all points from previous path to new path
  for (int i=0; i<prev_path_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // If there is no previous path, use the vehicles current location,
  if (prev_path_size == 0)
  {
    pos_x = car_x;
    pos_y = car_y;
    angle_xy = deg2rad(car_yaw);

    pos_x2 = pos_x - 1*cos(angle_xy);
    pos_y2 = pos_y - 1*sin(angle_xy);
  }

  // Use the last 2 path points
  else
  {
    pos_x = previous_path_x[prev_path_size-1];
    pos_y = previous_path_y[prev_path_size-1];

    pos_x2 = previous_path_x[prev_path_size-2];
    pos_y2 = previous_path_y[prev_path_size-2];
    angle_xy = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }

  // Use the last 2 points from the previous path for the new trajectory
  ptsx.push_back(pos_x2);
  ptsx.push_back(pos_x);

  ptsy.push_back(pos_y2);
  ptsy.push_back(pos_y);

  // Add 3 future points for the spline
  next_xy0 = getXY(car_s+50, 2+4*lane, maps_s, maps_x, maps_y);
  next_xy1 = getXY(car_s+75, 2+4*lane, maps_s, maps_x, maps_y);
  next_xy2 = getXY(car_s+100, 2+4*lane, maps_s, maps_x, maps_y);

  ptsx.push_back(next_xy0[0]);
  ptsx.push_back(next_xy1[0]);
  ptsx.push_back(next_xy2[0]);

  ptsy.push_back(next_xy0[1]);
  ptsy.push_back(next_xy1[1]);
  ptsy.push_back(next_xy2[1]);

  // Shift the vehicle to (0, 0) and the reference angle to 0 degrees
  for (int i=0; i<ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - pos_x;
    double shift_y = ptsy[i] - pos_y;

    ptsx[i] = (shift_x*cos(0-angle_xy) - shift_y*sin(0-angle_xy));
    ptsy[i] = (shift_x*sin(0-angle_xy) + shift_y*cos(0-angle_xy));
  }

  // Generate a spline tangential to the previous path
  tk::spline spline_xy;
  spline_xy.set_points(ptsx, ptsy);

  // Break up the spline points to desired velocity
  double target_x = 30;           // choose an arbitrary future point
  double target_y = spline_xy(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  // Store remaining points
  for (int i=0; i<50-prev_path_size; i++)
  {
    double dist_inc = ref_v*4/9 * 0.02;   // speed(m/s)*time(s) = meters to travel for desired speed
    double N = (target_dist / dist_inc); // N is the number of points between 0 and target distance
    double x_point = x_add_on + (target_x / N);  // the x point along the spline which will give the desired speed
    double y_point = spline_xy(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Rotate points back to normal
    x_point = x_ref*cos(angle_xy) - y_ref*sin(angle_xy);
    y_point = x_ref*sin(angle_xy) + y_ref*cos(angle_xy);

    x_point += pos_x;
    y_point += pos_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  return {next_x_vals, next_y_vals};
}


void Vehicle::bot_predictions(vector< vector<double> > sensor_data, double dt,
                              vector<double> maps_x, vector<double> maps_y) {
  /*
	 * bot_predictions predicts the movements of other vehicles on the highway
	 * `sensor_data` is the sensor fusion output of the simulator
   * `dt` is the time between simulator cycles
	 * 'maps_x' and 'maps_y' are the waypoint xy points
   * return vehicle predictions in the form {idx: {lane, s, v_s, a_s}}
	 */
  predictions.clear();
  vector<double> convert_sd;
  double x, y, vx, vy, v_s, v_s_prev, a_s, x_, y_, theta, s_, d_, bot_lane;

  // Initialise the previous sensor data variable
  if (sensor_data_prev.size() == 0) {
    sensor_data_prev = sensor_data;
  }

  // Loop through all sensed vehicles on the road
  for (int i=0; i<sensor_data.size(); i++) {
    x  =  sensor_data[i][1];
    y  =  sensor_data[i][2];
    vx =  sensor_data[i][3];
    vy =  sensor_data[i][4];

    // assumption: the sensed vehicle will be travelling in the s direction
    v_s      = sqrt(vx*vx + vy*vy);
    v_s_prev = sqrt( pow(sensor_data_prev[i][3], 2) + pow(sensor_data_prev[i][4], 2) );
    a_s      = (v_s - v_s_prev) / dt;

    // calculate angle for conversion to sd
    x_ = x + vx*dt;
    y_ = y + vy*dt;
    theta = atan2(y_-y, x_-x);

    // convert xy prediction to sd
    convert_sd = getFrenet(x, y, theta, maps_x, maps_y);
    s_ = convert_sd[0];
    d_ = convert_sd[1];

    // using d, calculate the lane the sensed vehicle is in
    if (d_ < 4) bot_lane = 0;
    else if ( (d_ > 4) && (d_ < 8) ) bot_lane = 1;
    else if (d_ > 8) bot_lane = 2;

    // store the predictions in format {idx: {lane, s, v_s, a_s}}
    lane_s = {bot_lane, s_, v_s, a_s};
    predictions[i] = lane_s;
  }
  // Update previous sensor fusion data
  sensor_data_prev = sensor_data;
}



void Vehicle::update_state() {
	/*
    Updates the "state" of the vehicle by assigning one of the following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will change lanes usnig a JMT and then follow longitudinal
       behaviour for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    lane and the SECOND the s position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"lane": 0, "s" : 4,  "v_s" : 2, "a_s" : 0},
        {"lane": 0, "s" : 6,  "v_s" : 2, "a_s" : 0},
        {"lane": 0, "s" : 8,  "v_s" : 2, "a_s" : 0},
        {"lane": 0, "s" : 10, "v_s" : 2, "a_s" : 0},
      ]
    }
  */
  string current_state = state;
  bool collision_kl    = false;
  bool collision_left  = false;
  bool collision_right = false;
  cout << "Current state: " << current_state << endl;

  // Only consider states which can be reached from current FSM state.
  vector<string> possible_successor_states;
  if (current_state == "KL") {
    possible_successor_states.push_back("KL");
    possible_successor_states.push_back("PLCR");
    possible_successor_states.push_back("PLCL");
    cout << "Possible successor states: " << possible_successor_states[0] << " " << possible_successor_states[1] << " " << possible_successor_states[2] << endl;
  }
  else if (current_state == "PLCL") {
    possible_successor_states.push_back("KL");
    possible_successor_states.push_back("PLCL");
    possible_successor_states.push_back("LCL");
    cout << "Possible successor states: " << possible_successor_states[0] << " " << possible_successor_states[1] << " " << possible_successor_states[2] << endl;
  }
  else if (current_state == "PLCR") {
    possible_successor_states.push_back("KL");
    possible_successor_states.push_back("PLCR");
    possible_successor_states.push_back("LCR");
    cout << "Possible successor states: " << possible_successor_states[0] << " " << possible_successor_states[1] << " " << possible_successor_states[2] << endl;
  }
  else if (current_state == "LCL") {
    possible_successor_states.push_back("KL");
    cout << "Possible successor states: " << possible_successor_states[0] << endl;
  }
  else if (current_state == "LCR") {
    possible_successor_states.push_back("KL");
    cout << "Possible successor states: " << possible_successor_states[0] << endl;
  }

  // Calculate the total cost of each successor state  
  vector<double> costs;
  for (int i=0; i<possible_successor_states.size(); i++)
  {

    double cost_kl = 0;         // cost for sitting behind another car
    double cost_d = 0;          // cost for being in an illegal lanes
    double cost_collision = 0;  // the cost for colliding with another vehicle (ie. max cost)
    double cost_tie_breaker = 0;// cost for breaking ties between KL, PLC and LC
    double cost_for_state = 0;  // total cost for the successor state

    // Check for collision on all states
    int lane_change = 0;
    if ( (possible_successor_states[i] == "PLCL") or (possible_successor_states[i] == "LCL") )
    {
      lane_change = -1;
    }
    if ( (possible_successor_states[i] == "PLCR") or (possible_successor_states[i] == "LCR") )
    {
      lane_change = 1;
    }
    // Temporarily update SDC's lane based on successor state
    lane += lane_change;

    map<int, vector<double> >::iterator it = predictions.begin();
    while (it != predictions.end())
    {
      int v_id = it->first;
      vector<double> v_data = it->second;
      double l_ = v_data[0];
      double s_  = v_data[1];
      if ( (l_ == lane) && (s_ > (s-20) ) )   // look a small distance back for cars
      {
        double v_s = v_data[2];
        double a_s = v_data[3];
        Vehicle vehicle = Vehicle(l_, s_, v_s, a_s);
        Vehicle::collider collide;
        collide = will_collide_with(vehicle, horizon);    // collide is boolean
        if (collide.collision == 1)
        {
          // For debugging
//          cout << "+++++++++++++++++++++++++++++COLLISION++++++++++++++++++++++++\n";
//          cout << "+++++++++++++++++++++++++++++COLLISION++++++++++++++++++++++++\n";
//          cout << "+++++++++++++++++++++++++++++COLLISION++++++++++++++++++++++++\n";
          if ( (possible_successor_states[i] == "LCL") or (possible_successor_states[i] == "LCR") )
          {
            // High cost if SDC attempts to change lanes into a collision
            // If KL, SDC slows down
            // If PLC, SDC is staying in the same lane so will slow down to avoid collision
            cost_collision = 100000;
          }

          // Set collision flag
          // In KL state, cars from behind will slow down so no collision is expected
          if ( (possible_successor_states[i] == "KL") and (s_ > s) ) {
            collision_kl = true;
            // Remember the position and speed of the car in front
            bot_s = s_;
            bot_v = v_s;
            
          }
          else if ( (possible_successor_states[i] == "PLCL") or (possible_successor_states[i] == "LCL") ) {
            collision_left = true;
          }
          else if ( (possible_successor_states[i] == "PLCR") or (possible_successor_states[i] == "LCR") ) {
            collision_right = true;
          }
        }
      }
      it++;
    }

    // Cost for a car in front of SDC
    if ( (possible_successor_states[i] == "KL") and (collision_kl == 1) )
    {
      cost_kl = 10;       // cost to stay in lane. Higher than LC but less than illegal lane or turning into a collision
    }

    // Cost for going out of bounds
    if ( ( (lane < 0) and
           ( (possible_successor_states[i] == "PLCL") or (possible_successor_states[i] == "LCL") ) )
      or ( (lane > 2) and 
           ( (possible_successor_states[i] == "PLCR") or (possible_successor_states[i] == "LCR") ) ) )
    {
      cost_d = 1000;
    }

    // Cost penalty for changing lanes to break ties between KL, PLC and LC
    if ( (possible_successor_states[i] == "PLCL") or (possible_successor_states[i] == "PLCR") )
    {
      cost_tie_breaker = 0.2;
      // Decide whether it is better to turn left or right based on predicted collisions
      if ( (possible_successor_states[i] == "PLCL") and (collision_left == 1) ) {
        cost_tie_breaker += 0.1;
      }
      else if ( (possible_successor_states[i] == "PLCR") and (collision_right == 1) ) {
        cost_tie_breaker += 0.1;
      }
    }

    // Cost penalty for changing lanes to break ties between KL, PLC and LC
    if ( (possible_successor_states[i] == "LCL") or (possible_successor_states[i] == "LCR") )
    {
      cost_tie_breaker = 0.1;
    }

    cost_for_state += cost_collision;
    cost_for_state += cost_kl;
    cost_for_state += cost_d;
    cost_for_state += cost_tie_breaker;

    costs.push_back(cost_for_state);

    // Restore SDC's lane to actual state
    lane -= lane_change;

    cout << possible_successor_states[i] << " cost = " << cost_for_state << endl;
  }

  // Find the minimum cost state.
  double min_cost = 9999999;
  double cost = 0;
  cout << "old state " << state << endl;
  for (int i=0; i<possible_successor_states.size(); i++)
  {
    cost  = costs[i];
    if (cost <= min_cost)
    {
      min_cost = cost;
      state = possible_successor_states[i];
    }
  }
  cout << "new state " << state << endl;

  double delta_speed = v_max - v;

  // Actions for new state
  if (state == "KL")
  {
    // If a collision is predicted, reduce speed
    if (collision_kl == 1) {
      if (ref_v > bot_v) ref_v -= dist_max; // maximum deceleration (10m/s^2)
//      if (ref_v > bot_v) ref_v -= dist_max * (ref_v - bot_v) / v_max; // proportional deceleration
    }

    // If no collision is predicted, increase speed 
    else if (delta_speed > 0.2) {
      double v_inc = dist_max/2 * delta_speed * 0.12;
      if (v_inc > dist_max/2) v_inc = dist_max/2;
      ref_v += v_inc;
    }
  }

  else if ((state == "PLCL") or (state == "LCL"))
  {
    if (collision_left == 1) {
      if (ref_v > bot_v) ref_v -= dist_max * (ref_v - bot_v) / v_max; // proportional deceleration
//      if (ref_v > bot_v) ref_v -= dist_max/2;          // reduce speed at 5m/s
    }
  }

  else if ((state == "PLCR") or (state == "LCR"))
  {
    if (collision_right == 1) {
      if (ref_v > bot_v) ref_v -= dist_max * (ref_v - bot_v) / v_max; // proportional deceleration
//      if (ref_v > bot_v) ref_v -= dist_max/2;          // reduce speed at 5m/s
    }
  }

  // If our new state is a lane change, then update lane
  if (state == "LCL") {
    lane -= 1;
    if (delta_speed > 3) ref_v += dist_max/2;
  }
  else if (state == "LCR") {
    lane += 1;
    if (delta_speed > 3) ref_v += dist_max/2;
  }
}


vector<double> Vehicle::state_at(double t) {
	/*
   * Predicts state of vehicle in t seconds (assuming constant acceleration)
   * acceleration removed from equations due to it's inaccuracy
   */
//  double s = this->s + this->v * t + this->a * t * t / 2;
//  double v = this->v + this->a * t;
  double s = this->s + this->v * t;
  double v = this->v;

  return {this->lane, s, v, this->a};
}


bool Vehicle::collides_with(Vehicle other, double at_time) {
	/*
   * Simple collision detection.
   */
  vector<double> check1 = state_at(at_time);
  vector<double> check2 = other.state_at(at_time);

  // For debugging
//  cout << "EGO l s v: " << check1[0] << " " << check1[1] << " " << check1[2] << endl;
//  cout << "veh l s v: " << check2[0] << " " << check2[1] << " " << check2[2] << endl;

  return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= buffer);
}


Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {
  /*
   * Answers the question: Will my SDC collide with another vehicle?
   */
	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1; 

	for (int t=0; t<timesteps; t++)
	{
    if ( collides_with(other, t) )
    {
      collider_temp.collision = true;
			collider_temp.time = t; 
      return collider_temp;
    }
	}
	return collider_temp;
}