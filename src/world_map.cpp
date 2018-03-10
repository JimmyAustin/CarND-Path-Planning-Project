#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <math.h>
#include "world_map.h"
#include <fstream>
#include <chrono>

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <sstream>


/**
 * Initializes Vehicle
 */

constexpr double pi() { return M_PI; }
// double deg2rad(double x) { return x * pi() / 180; }
// double rad2deg(double x) { return x * 180 / pi(); }

WorldMap::WorldMap(){
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line2;
  while (getline(in_map_, line2)) {
    istringstream iss(line2);
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
    this->waypoints_x.push_back(x);
    this->waypoints_y.push_back(y);
    this->waypoints_s.push_back(s);
    this->waypoints_dx.push_back(d_x);
    this->waypoints_dy.push_back(d_y);
  }
}

double WorldMap::distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int WorldMap::ClosestWaypoint(double x, double y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;
    for(int i = 0; i < this->waypoints_x.size(); i++)
    {
        double map_x = this->waypoints_x[i];
        double map_y = this->waypoints_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int WorldMap::NextWaypoint(double x, double y, double theta)
{

    int closestWaypoint = ClosestWaypoint(x,y);

    double map_x = this->waypoints_x[closestWaypoint];
    double map_y = this->waypoints_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == this->waypoints_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> WorldMap::getFrenet(double x, double y, double theta)
{
    int next_wp = NextWaypoint(x,y, theta);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = this->waypoints_x.size()-1;
    }

    double n_x = this->waypoints_x[next_wp]-this->waypoints_x[prev_wp];
    double n_y = this->waypoints_y[next_wp]-this->waypoints_y[prev_wp];
    double x_x = x - this->waypoints_x[prev_wp];
    double x_y = y - this->waypoints_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-this->waypoints_x[prev_wp];
    double center_y = 2000-this->waypoints_y[prev_wp];
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
        frenet_s += distance(this->waypoints_x[i],this->waypoints_y[i],this->waypoints_x[i+1],this->waypoints_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> WorldMap::getXY(double s, double d)
{
    int prev_wp = -1;

    while(s > this->waypoints_s[prev_wp+1] && (prev_wp < (int)(this->waypoints_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%this->waypoints_x.size();

    double heading = atan2((this->waypoints_y[wp2]-this->waypoints_y[prev_wp]),(this->waypoints_x[wp2]-this->waypoints_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-this->waypoints_s[prev_wp]);

    double seg_x = this->waypoints_x[prev_wp]+seg_s*cos(heading);
    double seg_y = this->waypoints_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    std::cout << "--S--" << std::endl;
    std::cout << s << std::endl;
    std::cout << "--D--" << std::endl;
    std::cout << d << std::endl;
    std::cout << "--X--" << std::endl;
    std::cout << x << std::endl;
    std::cout << "--Y--" << std::endl;
    std::cout << y << std::endl;

    return {x,y};
}
