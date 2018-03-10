
#ifndef WORLDMAP_H
#define WORLDMAP_H

 #include <string>

using namespace std;
class WorldMap
{
    // Access specifier
    public:
 
    // Data Members
    std::string geekname;
 
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    /**
    * Constructor
    */
    WorldMap();

    double distance(double x1, double y1, double x2, double y2);
    int ClosestWaypoint(double x, double y);
    int NextWaypoint(double x, double y, double theta);
    vector<double> getFrenet(double x, double y, double theta);
    vector<double> getXY(double s, double d);
};

#endif
