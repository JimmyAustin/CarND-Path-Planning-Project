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
#include "spline.h"


constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.

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

  double target_velocity = 49.5;
  double current_target_velocity = 0;

  double current_lane = 1;
  int packet_tick = 0;

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

  h.onMessage([&packet_tick, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&current_lane,&target_velocity,&current_target_velocity](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
            packet_tick += 1;

            double threshold_front = 30;
            double threshold_back = 20;
            double acceleration_factor = 0.1;
            double deacceleration_factor = 0.3;

          
            // Main car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_speed = j[1]["speed"];

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            if (packet_tick == 1) {
              std::cout << "Previous wiped" << std::endl;
              //previous_path_x = "[]";
              //previous_path_y = "[]";
            }
            // Previous path's end s and d values 
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            
            bool too_close_to_car_in_front = false;
            bool change_to_left_lane = (current_lane > 0);
            bool change_to_right_lane = (current_lane < 2);

            // threshold for s gap between us and other cars before classifying them as being too close
            
            int prev_size = previous_path_x.size();

            int closest_car_ahead = 20;
            int closest_car_ahead_predicted_distance = 10000000;
            int closest_car_ahead_actual_distance = 10000000;

            double predicted_car_s = car_s + ((double)prev_size*.02*car_speed);
            double car_in_front_speed = 10000;

            for(int i = 0; i < sensor_fusion.size(); i++) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double other_car_speed = sqrt(vx*vx*vy*vy);
                double other_car_current_s = sensor_fusion[i][5];
                double other_car_future_s = double(sensor_fusion[i][5]) + ((double)prev_size*.02*other_car_speed);
                float d = sensor_fusion[i][6];
                int other_car_lane = floor((d/4.0));

                bool car_ahead = other_car_current_s > car_s;

                // Use the distance from the car in front in both real terms and predicted terms.
                bool gap_within_threshold = (other_car_future_s - predicted_car_s < threshold_front) and (other_car_future_s - predicted_car_s > 0-threshold_back) or
                                            (other_car_current_s - car_s < threshold_front) and (other_car_current_s - car_s > 0-threshold_back);

                //Print debug information every 10th tick tick.
                if (packet_tick % 10 == 0) {
                    std::cout << i << " - Lane: " << other_car_lane << ", Ahead: " << car_ahead << std::endl;
                    std::cout << "            Location:" << other_car_current_s << std::endl;
                    std::cout << "  Predicted Location:" << other_car_future_s << std::endl;
                    std::cout << "  Gap Within Threshold:" << gap_within_threshold << std::endl;
                    std::cout << "  Predicted Location:" << other_car_future_s << std::endl;
                    std::cout << "  Speed:" << other_car_speed << std::endl;
                    std::cout << "  Predicted Distance:" << other_car_future_s - predicted_car_s << std::endl;
                    std::cout << "  Distance:" << other_car_current_s - car_s << std::endl;
                }
                if (other_car_lane == current_lane) {
                  if (car_ahead) {
                    if (car_in_front_speed < other_car_speed) {
                      car_in_front_speed = other_car_speed;
                    }
                  }
                  if ((other_car_future_s - predicted_car_s) < closest_car_ahead_predicted_distance && car_ahead) {
                    closest_car_ahead = i;
                    closest_car_ahead_predicted_distance = other_car_future_s - predicted_car_s;
                    closest_car_ahead_actual_distance = other_car_current_s - car_s;
                  } 
                }

                if (gap_within_threshold) {

                    // other car in our lane
                    if (other_car_lane == current_lane && car_ahead) {
                        too_close_to_car_in_front = true;
                    }
                    else if (other_car_lane == current_lane - 1) {
                        change_to_left_lane = false;
                    }
                    else if (other_car_lane == current_lane + 1) {
                        change_to_right_lane = false;
                    }
                }
            }

            // Debugging value
            auto action = "Continue";

            if (too_close_to_car_in_front) {
                //If speed to high, reduce to a sensible minimum.
                if (car_in_front_speed > 100) {
                  car_in_front_speed = 20;
                }

                //Drop speed by a constant factor, or to the speed of the car in front, whichever is higher.
                current_target_velocity = max(current_target_velocity - deacceleration_factor, car_in_front_speed);

                if (change_to_left_lane == true) {
                    current_lane -= 1;
                    action = "Left Lane";
                } else if (change_to_right_lane == true) {
                    current_lane += 1;
                    action = "Right Lane";
                } else {                    
                  action = "Slowing";
                }
            }
            else if (current_target_velocity < 49.5) {
                                  action = "Right Lane";
                    action = "Speedup";
                current_target_velocity += acceleration_factor;
            } else {
              action = "Continue";
            }

            // Debugging information, to be print on every 10th packet. 
            // This makes it easier to track the same car in the debugging console.

            if (packet_tick % 10 == 0) {
              cout << "too_close_to_car_in_front: " << too_close_to_car_in_front << endl;
              cout << "change_to_left_lane: " << change_to_left_lane << endl;
              cout << "change_to_right_lane: " << change_to_right_lane << endl;

              cout << "current_target_velocity: " << current_target_velocity << endl;
              cout << "closest_car_ahead: " << closest_car_ahead << endl;
              cout << "closest_car_ahead_predicted_distance: " << closest_car_ahead_predicted_distance << endl;
              cout << "closest_car_ahead_actual_distance: " << closest_car_ahead_actual_distance << endl;

              cout << "Current Lane: " << current_lane << endl;
              cout << "Action: " << action << endl;
              std::cout << "------" << std::endl;
            }


            vector<double> next_x_vals;
            vector<double> next_y_vals;

            vector<pair<double, double>> pst;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);


            if (prev_size < 2) {
                pst.push_back(make_pair(car_x - cos(car_yaw), car_y - sin(car_yaw)));
            } else {
                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];

                double ref_x_prev = previous_path_x[prev_size-2];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                pst.push_back(make_pair(ref_x_prev, ref_y_prev));
                pst.push_back(make_pair(ref_x, ref_y));
            }

            float d = 2 + (4 * current_lane); // Lanes are four meters wide. 

            // We define spacing based on the speed of the vehicle, with a minimum of 20 units. 
            // This allows the car to correct back into a lane faster at lower speeds, 
            // but still allows for less jerk in lane changes at high speeds.

            double spacing = max(car_speed, 20.);//50.;
            vector<double> next_wp0 = getXY(car_s+spacing, (2+4*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+spacing*2, (2+4*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+spacing*3, (2+4*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            pst.push_back(make_pair(next_wp0[0], next_wp0[1]));
            pst.push_back(make_pair(next_wp1[0], next_wp1[1]));
            pst.push_back(make_pair(next_wp2[0], next_wp2[1]));

            // Sort waypoints

            for (int i = 0; i < pst.size(); i++)
            {
                double shift_x = pst[i].first-ref_x;
                double shift_y = pst[i].second-ref_y;

                pst[i].first =  (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                pst[i].second = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }

            // Sort waypoints. This ensures that we wont have have any pstx[x] !< pstx[x-1] errors.
            sort(pst.begin(), pst.end());

            // Turn pairs into position vectors

            vector<double> pstx;
            vector<double> psty;

            for (int i = 0; i <pst.size(); i++)
            {

              pstx.push_back(pst[i].first);
              psty.push_back(pst[i].second);
            }

            // Define spline
            tk::spline spline;
            spline.set_points(pstx, psty);

            for(int i = 0; i < previous_path_x.size(); i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = 30.0;
            double target_y = spline(target_x);
            double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

            double x_addon = 0;

            // Build paths based off spline
            for (int i = 1; i <= 50-previous_path_x.size(); i++) {
                double N = (target_dist/(.02*current_target_velocity/2.24));
                double x_point = x_addon+target_x/N;
                double y_point = spline(x_point);
                
                x_addon = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

            json msgJson;

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

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
