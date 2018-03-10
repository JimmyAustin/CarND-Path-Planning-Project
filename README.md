# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

Video of successful run: https://youtu.be/WwaXwFQXFxQ

### Build and Run Instructions
cd build
cmake .. && make && ./path_planning
   
### Reflection

The code is largely based off the code used in the walkthrough.

We generate a series of waypoints in front of the car. These waypoints are spaces between 20 units apart, or X units where X is the speed of the car. We prepend these generated waypoints either the last two points of the path we generated last tick, or the cars current location if it is the first tick.

We then generate a series of points between these waypoints, with the points being relative to our cars target velocity.

The target velocity begins at zero, and increases by 0.1 every tick. When we need to change lanes, or have another car in front, we decrease the speed by 0.3, to a minimum of either 20 or the speed of the car in front of us.

We detect if we need to change lanes by seeing if the car in front of us is at a safe distance, and if it is not, moving lanes. We determine if a lane is safe to move into if it has no vehicles in it within the threshold. If we have a car in front of us, but no safe lanes to move into, we simply drop the speed.