# Model Documentation
## Path Planning
### Project One - Term 3

---
### Reflection - Path Generation
The code for the path generation is in the `CreateTrajectory` function in `helperFunctions.h`.

#### Function Inputs
* `lanes` - This is a vector of the 3 lanes through which the path will be generated. The lanes are associated to the desired lane at a distance of 30, 60 and 90m
* `target_vel` - This is the target velocity for the vehicle as set by the `ChooseSpeed` function. It is the ideal speed for the vehicle if nothing else is happening
* `ref_vel` - This is essentially a storage of the actual current target speed for the path planning at each time step. It is used to 'lag' the cars velocity behind the target speed so it doesn't accelerate too fast as well as allow the car to slow down in dangerous situations even if the target speed is high
* `steps_forwards` - This is how far forward in time the path is to be calculated. This allows for potential trajectories to be estimated further in front than the current one
* `avoid_vehicle` - This is a flag from the `CheckFrontCollision` function that makes sure the vehicle doesn't collide with any vehicles in front of it. This flag will trigger the `ref_vel` to decrease rapidly
* `previous_path_x` & `previous_path_y` - These are the x and y coordinates for the previous paths set for the vehicle. These are used to pre-populate the new trajectory so it has a smooth transition as the new path is generated
* `ego` - This is the cars current state stored as the class `Ego`
* `map_waypoints_s` & `map_waypoints_x` & `map_waypoints_y` -  The fixed waypoints on the road for the path to be generated against

#### Spline Generation
The spline library in `spline.h` is used to generate a smooth path between the desired points. 5 points are used to create the spline:

1. The second to last point from the previous path that was generated (line 117 or line 132)
2. The last point from the previously generated path (line 120 or line 135)
3. A point in the desired lane at 30m ahead of the end of the original path (line 144)
4. A point in desired lane at 60m ahead (line 145)
4. A point in desired lane at 90m ahead (line 146)

These 5 points are then shifted to the coordinates of the ego vehicles to simplify the later calculations before generating the spline (line 166)

### Previous Path Reuse
The remaining points for the previous path, that haven't yet been 'used' are then loaded into the new path. This ensured the new path follows on smoothly from the previous paths

### New Path Generation
The new path is now created for the remaining points as set by `steps_forwards`. Firstly the reference velocity for the next step is calculated. A simple 'on/off' controller is used to get the `ref_vel` to match the `target_vel`. `ref_vel` is incremented by 0.112 each step until it matches or is greater than `target_vel`. This ensures that maximum accleration and jerk are not exceeded.

The `avoid_vehicle` flag will override this however to stop the vehicle from hitting another vehicle. If the flag is set the reference velocity is reduced until it is 10 mph less than the target velocity. It is reduced at a decrement of 0.4 which is close to the allowable acceleration.

Next a trick is used to calculate the size of the step to produce `ref_vel`. Firstly the direct distance to the point defined by the spline 30m (in the x direction) in front of the vehicle is calculated (line 186). Next this distance is split into equal segments that will produce the `ref_vel` (line 208). From here the distance to move in the x direction is calculated using the length of the segments previously calculated (line 209). This is then passed to the spline to calculate y distance (210).

The `x` and `y` coordinates are then rotated back to the original coordinate system and stored in a vector to be returned.
