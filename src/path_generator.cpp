#include "path_generator.h"
#include "world_map.h"
#include "spline.h"
#include <math.h>
#include <iostream>
using namespace std;

constexpr double pi() { return M_PI; }
double map_deg2rad(double x) { return x * pi() / 180; }
double map_rad2deg(double x) { return x * 180 / pi(); }

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
                                  float target_velocity) {
    
    vector<double> pstx;
    vector<double> psty;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = map_deg2rad(car_yaw);

    int prev_size = previous_path_x.size();
    if (prev_size < 2) {
        pstx.push_back(car_x - cos(car_yaw));
        pstx.push_back(car_x);

        psty.push_back(car_y - sin(car_yaw));
        psty.push_back(car_y); 
    } else {
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        pstx.push_back(ref_x_prev);
        pstx.push_back(ref_x);

        psty.push_back(ref_y_prev);
        psty.push_back(ref_y); 
    }

    current_lane = 1;
    float d = 2 + (4 * current_lane); // Lanes are four meters wide. 

    vector<double> next_wp0 = world_map.getXY(car_s + 30.0, 6);
    vector<double> next_wp1 = world_map.getXY(car_s + 60.0, 6);
    vector<double> next_wp2 = world_map.getXY(car_s + 90.0, 6);

    pstx.push_back(next_wp0[0]);
    pstx.push_back(next_wp1[0]);
    pstx.push_back(next_wp2[0]);

    psty.push_back(next_wp0[1]);
    psty.push_back(next_wp1[1]);
    psty.push_back(next_wp2[1]);

    // Shift them all to reference

    std::cout << "------" << std::endl;


    for (int i = 0; i <pstx.size(); i++)
    {
        double shift_x = pstx[i]-ref_x;
        double shift_y = psty[i]-ref_y;

        pstx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
        psty[i] = (shift_x * sin(0-ref_yaw)-shift_y*cos(0-ref_yaw));
        std::cout << "--" << std::endl;
        std::cout << i + 1 << std::endl;
        std::cout << "--" << std::endl;
        std::cout << pstx[i] << std::endl;
        std::cout << psty[i] << std::endl;

    //         std::cout << pstx[i] << std::endl;
    // std::cout << psty[i] << std::endl;

    }

    tk::spline spline;

    spline.set_points(pstx, psty);

    // Define the next steps
    vector<coordinate> next_values;

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for (int i = 0; i < previous_path_x.size(); i++)
    {
        coordinate next_coordinate;
        next_coordinate.x = previous_path_x[i];
        next_coordinate.y = previous_path_y[i];
        next_values.push_back(next_coordinate);
    }

    double ref_vel = 45.0;

    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

    double x_addon = 0;

    for (int i = 1; i <= 50-previous_path_x.size(); i++) {
        double N = (target_dist/(.02*ref_vel/2.24));
        double x_point = x_addon+target_x/N;
        double y_point = spline(x_point);
        
        x_addon = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        coordinate next_coordinate;
        next_coordinate.x = x_point;
        next_coordinate.y = y_point;
        next_values.push_back(next_coordinate);
    }


    // float delta = 0.3;

    // for (int i = 1; i < 50; i++) {
    //     coordinate next_coordinate;
    //     std::vector<double> xy_values = world_map.getXY(s + (i * delta), 2);
    //     next_coordinate.x = xy_values[0];
    //     next_coordinate.y = xy_values[1];
    //     next_values.push_back(next_coordinate);
    // };
    return next_values;
};

coordinate offset_point(float car_x, float car_y, float yaw, float distance) {
    coordinate next_coordinate;


    return next_coordinate;
}