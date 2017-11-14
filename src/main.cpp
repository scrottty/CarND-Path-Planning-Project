#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "providedFunctions.h"
#include "spline.h"
#include "Ego.h"
#include "Vehicle.h"
#include "constants.h"
#include "helperFunctions.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // cout.precision(3);

  // enum to control state machine
  Ego ego;
  // vehicle_state ego_state = keep_lane;

  // Start in the middle lane
  int current_lane = 1;
  int new_lane = 1;
  int old_lane = 1;
  int prev_desired_lane = 1;

  // Get close to the speed limit
  double ref_vel = 0.0;
  double target_vel = MAX_VELOCITY;
  double desired_vel = 0.0;

  // Bean counter to sum up the lane choice for lane change confirmation
  int lane_bean_count = 0;
  int safety_bean_count = 0;

  bool lane_changed_initialised = false;

  // The choosen vector after choosing the speed
  vector<vector<double>> choosen_trajectory;

  // Keep driving comfortable
  double max_acc = 8;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
      &ref_vel, &target_vel, &desired_vel, &current_lane, &new_lane, &old_lane, &prev_desired_lane, &ego, &max_acc,
      &lane_bean_count, &safety_bean_count, &lane_changed_initialised, &choosen_trajectory]
      (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	ego.x = j[1]["x"];
          	ego.y = j[1]["y"];
          	ego.s = j[1]["s"];
          	ego.d = j[1]["d"];
          	ego.yaw = j[1]["yaw"];
          	ego.velocity = j[1]["speed"];

          	// Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            int prev_size = previous_path_x.size();
            // Adjust the cars position to be at the end of the planned trajectory
            if (prev_size > 0)
            {
                ego.current_s = ego.s;
                ego.s = end_path_s;
                // car_values[2] = end_path_s;
            }

            // Split vehicles by lane
            vector<vector<Vehicle>> lane_vehicles;
            SplitVehicelsIntoLanes(sensor_fusion, lane_vehicles, ego.s, prev_size);

            // Get the vehicles that are within a safety distance if the car
            // Get the vehicles that within a distance to effect performance decisions
            vector<vector<Vehicle>> safety_vehicles;
            vector<vector<Vehicle>> relevant_vehicles;
            FilterVehicles(lane_vehicles, relevant_vehicles, safety_vehicles);

            int lane_middle = 2 + 4*current_lane;
            bool slow_vehicle = false;

            // Check for potential collision
            bool avoid_vehicle = CheckFrontCollision(lane_vehicles[current_lane], ego);
            // Run the collision detection on the new lane as well to make sure we dont clip the car in front
            if (ego.current_state == lane_change)
                avoid_vehicle = avoid_vehicle || CheckFrontCollision(lane_vehicles[new_lane], ego);

            // Lanes vector. Here because it will be adjusted when changing lanes
            vector<int> lanes;
            lanes.push_back(current_lane);
            lanes.push_back(current_lane);
            lanes.push_back(current_lane);

            switch (ego.current_state)
            {
                case keep_lane:
                {
                    cout << flush << endl;

                    // Choose which lane to be in and confirm it (10 times)
                    new_lane = ChooseLane(relevant_vehicles, target_vel, ref_vel, current_lane, previous_path_x, previous_path_y,
                                        map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    new_lane = ConfirmLaneChange(new_lane, current_lane, prev_desired_lane, lane_bean_count);

                    // Change the state if changing lanes
                    if (new_lane != current_lane)
                    {
                        ego.current_state = prepare_lane_change;
                        break;
                    }

                    // Find the traget velocity is staying in the same lane
                    target_vel = ChooseSpeed(relevant_vehicles, ref_vel, new_lane, current_lane,
                        previous_path_x, previous_path_y, ego, map_waypoints_s, map_waypoints_x, map_waypoints_y, choosen_trajectory);

                    break;
                }
                case prepare_lane_change:
                {
                    // Based on the desired lane change calculate the ideal velocity
                    desired_vel = ChooseSpeed(relevant_vehicles, ref_vel, new_lane, current_lane,
                        previous_path_x, previous_path_y, ego, map_waypoints_s, map_waypoints_x, map_waypoints_y, choosen_trajectory);

                    // Check the lane is safe to move into
                    bool slow_down = false;
                    bool safe = CheckLaneIsSafe(slow_down, safety_vehicles, choosen_trajectory, map_waypoints_s, map_waypoints_x, map_waypoints_y, current_lane, new_lane);
                    if (ConfirmSafe(safe, safety_bean_count))
                    {
                        // If the lane is safe but the vehicle is too close to the vehicle in front, slow down
                        if (slow_down)
                        {
                            target_vel -= 0.112;
                            cout << "--- Slowing Down To Change" << endl;
                        }
                        else // Otherwise change lane and set the new velocity
                        {
                            ego.current_state = lane_change;
                            target_vel = desired_vel;
                            old_lane = current_lane;
                        }

                    }
                    else if (safe == false) // If the lane is not safe go back to finding a new lane
                    {
                        cout << "--- Tried To Change But Not Safe" << endl;
                        ego.current_state = keep_lane;
                    }
                    break;
                }
                case lane_change:
                {
                    // Move to the middle lane first if a double lane change
                    int new_middle;
                    double lane_middle_tol = 1.5;
                    if ( abs(current_lane-new_lane) > 1)
                    {
                        lanes[0] = current_lane + (new_lane-current_lane)/2;
                        lanes[1] = current_lane + (new_lane-current_lane)/2;
                        lanes[2] = current_lane + (new_lane-current_lane)/2;
                        new_middle = 2+4*(current_lane+ (new_lane-current_lane)/2);
                    }
                    else
                    {
                        lanes[0] = new_lane;
                        lanes[1] = new_lane;
                        lanes[2] = new_lane;
                        new_middle = 2+4*new_lane;
                    }

                    // Update the lane
                    if (ego.d>new_middle-lane_middle_tol && ego.d<new_middle+lane_middle_tol)
                        current_lane = GetLane(ego.d);

                    if (current_lane == new_lane)
                    {
                        ego.current_state = keep_lane;
                    }
                    // Go back to prepare lane change if it is a double lane change. Want to make sure the next lane is still safe
                    else if (current_lane != old_lane)
                    {
                        ego.current_state = prepare_lane_change;
                    }

                    break;
                }
                default:
                    break;
            }
            vector<vector<double>> next_xy = CreateTrajectory(lanes,
                target_vel, ref_vel, 50, avoid_vehicle, previous_path_x, previous_path_y,
                ego, map_waypoints_s, map_waypoints_x, map_waypoints_y);

         	msgJson["next_x"] = next_xy[0];
         	msgJson["next_y"] = next_xy[1];

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
