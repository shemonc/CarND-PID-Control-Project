# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Describe the Effect of P, I, D components in the implementation

[//]: # (Video clip References)

[video1]: ./P_controlled_self_drive_overshoot_and_drive_along_ref_trajectory.mov "Driving with P component only"
[video2]: ./PD_controlled_self_drive_oscilate_with_increased_CTE.mov "Driving with PD component only"
[video3]: ./PID_controlled_self_drive.mov "Driving with PID component FULL TRACK"

### P Component

Cross Trace Error (CTE) is lateral distance between the vehicle and the so-called reference trajectory.
P controller steers the car proportion(inversely, Tau) to CTE. If the car is far to the right it steers more to the left and if the car is slightly to left, it steers slightly to right. With a higher Tau the car will oscillate faster, i.e. if Tau is set to 0.3 instead of 0.1, the oscillation will increased. 

Video ![alt text][video1] demonstrate overshoot and drive along the reference trajectory from 7 to 15 second of the clip.

### D Component

To avoid this overshot  PD controller is needed. In PD-controller the steering alpha is not just related to the CTE by a factor of Tau-p but all to the temporal derivative of the cross track error,  
  
steering angle, Alpha = -Tau_p*CTE - Tau_d*d/dt(CTE)  

it means when the car is turned enough to reduce the cross track error, it won’t just go shooting for the reference trajectory but it will notice that it is already reducing the error and as the error is becoming smaller overtime, it counter steers. It steers up again, this will allow it to gracefully approach the reference trajectory.

Video ![alt text][video2]  with PD only, the car oscillate , CTE increases constantly, steering is going from positive to negative & positive back again and ultimately overshoot  from 5 to 12 second  

cte: -1.2626 Steering Value: 0.382209 Throttle 0.2 speed 28.6455 angel 9.8254  
cte: -1.2636 Steering Value: 0.37935 Throttle 0.2 speed 28.6276 angel 9.8896  
cte: -1.2545 Steering Value: 0.373893 Throttle 0.2 speed 28.593 angel 9.9916  
cte: -1.2445 Steering Value: 0.37065 Throttle 0.2 speed 28.577 angel 9.9201  
cte: -1.2134 Steering Value: -0.613398 Throttle 0.2 speed 28.5478 angel 9.7837  
cte: -1.1924 Steering Value: -0.600201 Throttle 0.2 speed 28.5339 angel 9.7026  
cte: -1.1494 Steering Value: -0.584701 Throttle 0.2 speed 28.3531 angel -14.8986  
cte: -1.1283 Steering Value: -0.568267 Throttle 0.2 speed 28.2967 angel -14.5687  
cte: -1.0867 Steering Value: -0.553061 Throttle 0.2 speed 28.0996 angel -14.1812  
cte: -1.0662 Steering Value: -0.537142 Throttle 0.2 speed 27.9935 angel -13.7704  
cte: -1.0292 Steering Value: -0.523149 Throttle 0.2 speed 27.8336 angel -13.3902  
cte: -0.9979 Steering Value: -0.506004 Throttle 0.2 speed 27.6959 angel -12.9922  
cte: -0.9858 Steering Value: -0.494787 Throttle 0.2 speed 27.6401 angel -12.6424  
cte: -0.9695 Steering Value: -0.487794 Throttle 0.2 speed 27.5536 angel -12.2138  
cte: -0.9669 Steering Value: -0.482798 Throttle 0.2 speed 27.4898 angel -11.9333  
cte: -0.9709 Steering Value: -0.483011 Throttle 0.2 speed 27.4635 angel -11.7585  
cte: -0.9897 Steering Value: -0.488388 Throttle 0.2 speed 27.4197 angel -11.6336  
cte: -1.0046 Steering Value: -0.496871 Throttle 0.2 speed 27.4002 angel -11.6389  
cte: -1.0455 Steering Value: -0.510243 Throttle 0.2 speed 27.363 angel -11.7734  
cte: -1.0717 Steering Value: -0.527276 Throttle 0.2 speed 27.3447 angel -11.9854  
cte: -1.1019 Steering Value: -0.541253 Throttle 0.2 speed 27.3248 angel -12.3198  
cte: -1.1747 Steering Value: -0.566049 Throttle 0.2 speed 27.2834 angel -12.7456  
cte: -1.2174 Steering Value: -0.109359 Throttle 0.2 speed 27.2627 angel -13.095  
cte: -1.3162 Steering Value: -0.104023 Throttle 0.2 speed 27.2162 angel -13.7149  
cte: -1.3689 Steering Value: -0.121703 Throttle 0.2 speed 27.2069 angel -2.2976  
cte: -1.4228 Steering Value: -0.126731 Throttle 0.2 speed 27.1866 angel -2.1642  
cte: -1.5345 Steering Value: 0.429236 Throttle 0.2 speed 27.1703 angel -2.6062  
cte: -1.5924 Steering Value: 0.429768 Throttle 0.2 speed 27.1654 angel -2.7319  
cte: -1.7056 Steering Value: 0.474139 Throttle 0.2 speed 27.0142 angel 11.1672  
cte: -1.7601 Steering Value: 0.472464 Throttle 0.2 speed 26.9583 angel 11.1805  

### I Component 

A car might have a from wheels not aligned appropriately , in robotics it is called Systematic bias.  Systematic bias will significantly increase the CTE in PD controller, so the differential term will not be able to  compensate for this. This is where the I components come to play which is measured by the integral or the sum of the CTEs over time   
   
steering angle, Alpha = -Tau_p*CTE - Tau_d*d/dt(CTE) - Tau_I*Sum(CTE)  

if there is a constant cross track error of say 0.8 , the sum will increased by each time unit and it will become larger and larger & eventually it will correct the robot motion.

Video ![alt text][video3] Demonstrate a Full Track drive with PID conponent  

## Describe how the final hyperparameters were chosen
Initial hyperparameters was chosen using manual tuning and later use twiddle to fine tune it.

- I start applying twiddle after 100 steps after the car start from zero or the cross track error is 1.9 or more.

- whenever cte goes above 1.2 , apply twiddle

- if the steering angle is within -1 to 1 apply an adaptive throttle based on the current cte, check ApplyAdaptiveThrottle.

- if steering angle is outside of -1 to 1
    - reset the PID coefficient to some empirical value (kp = 0.2, Kd = 5.0, Ki = 0.002)
    - apply an adaptive breaking based on current cte.



---


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

