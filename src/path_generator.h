// #ifndef COST_H
// #define COST_H

struct coordinate {
    float x;  
    float y;
};

#include <vector>
#include "world_map.h"

vector<coordinate> generate_paths(vector<double> previous_path_x, 
                                  vector<double> previous_path_y,
                                  float car_x, 
                                  float car_y, 
                                  float car_yaw, 
                                  float car_s,
                                  float car_d,
                                  WorldMap world_map, 
                                  int current_lane, 
                                  int target_lane, 
                                  float target_velocity);

//vector<coordinate> generate_paths(float current_x, float current_y, float car_yaw, WorldMap world_map, int current_lane, int target_lane, float target_velocity);
// #endif