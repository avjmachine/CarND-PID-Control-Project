# CarND-Controls-PID Project Writeup
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[image1]: ./writeup_images/pid_diagram.png "Block Diagram of a PID Controller"
[image2]: ./writeup_images/pid_equation.png "Equation for PID Control"


## Introduction

The objective of this project is to maneuver a vehicle safely around a track in the simulator using a C++ program, by computing appropriate steering angles based on the cross track error and velocity, which are given as inputs to the C++ program by the simulator. The computed steering angles are given as outputs by the C++ program to the simulator, which in turn moves the vehicle accordingly.


## PID Control for Steering

### PID Control

PID Control is one of the most commonly used control algorithms for maintaining a closed loop system. A closed loop system is one where the process variables such as temperature, flow rate, velocity, etc. are continuously monitored and the error between desired and actual values is tracked. The control action is taken based on a weighted sum in proportion to the error, its derivatives and its integrals. This can be visualized using a control block diagram as shown below:

![alt text][image1]

The final control output is given by the following equation:

![alt text][image2]

A brief description of each of the three terms is given below:

1. **Proportional**

   The proportional term in the PID control compensates proportionately for the error. If the error is more, it raises the compensating control action and if the error is less, it reduces the required control action. If there is no error, there is no corrective response. 
   
   
2. **Integral**

   The integral term accounts for the past values of the error and integrates them over time. If there is steady state error inspite of the proportional control, this term tries to reduce that residual error by acting in proportion to the historic cumulative error.
   
   
3. **Derivative**

   The derivative term acts on reducing the error in future, by estimating the future trend of the error based on its current rate of change. This helps in reducing overshoots and oscillations. In other words, it has a damping effect.
   
The gains or coefficient terms need to be tuned either manually by trial and error or automatically using algorithms.

### Application of PID Control for Steering

In this project, PID control is used for steering the vehicle. Here, the control output is the steering angle of the car. This angle is computed using the PID control equation. 

The error for the proportional term is given by the cross-track error(CTE), which is the difference between the desired lateral position of the vehicle and its actual lateral position along the points on its trajectory. The error for the derivative term is calculated by taking the difference between the current CTE and the CTE of the previous time step, whereas the error for the integral term is calculated as the cumulative sum of the CTE from all the previous time steps.

The tuning of the gain parameters is described in detail in the section below named "Implementation Details"


## Running the Code

Inorder to run this code, the following dependencies need to be installed.

### Dependencies

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
* Simulator. It can be downloaded from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases).

### Basic Build Instructions

This code can be built and run using the following steps:

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## Implementation Details

### Code Description

The code for this project can be found in the `src` directory. There are primarily 4 files that have been used, which are describe below:

1. `json.hpp` - This file contains code which helps in communicating with the simulator using json messages.


2. `PID.h` - This is the header file which contains declarations for the PID class and its attributes.


3. `PID.cpp` - This file contains the function definitions for the PID class member functions such as `Init()` which initializes the PID gain coefficients and their respective errors, and `UpdateError()` which stores and updates at every time step - the error, its delta from the previous time step (derivative) and its historic cumulative sum (integral). It also has a `TotalError()` function that calculates the final control output,i.e., the steering angle value, using a weighted sum of the 3 error terms (the gain coefficients being the weighting factors).


4. `main.cpp` - The main code which interfaces with the simulator can be found here. It reads in the input values (the cross-track error, velocity and steering angle) from the simulator and initializes the above mentioned PID Class object as an instance by calling the `Init()` function. It then calls the `UpdateError()` function to update the proportional, integral and derivative error terms. After this, it calls the `TotalError()` function to get the final control output, i.e., the steering angle and feeds this value along with the throttle value to the simulator. In this implementation of the project, the speed or throttle is not controlled. Hence, the input speed values are unutilized and a constant throttle is given throughout the entire simulation. But, the steering is controlled at every time step using PID control.


### Tuning the parameters

The Kp, Ki and Kd gain coefficients are tuned using an algorithm named Twiddle, that was explained in the Udacity course. This algorithm varies the 3 coefficients in a cyclical manner till a desired level of error or tolerance is reached. It starts with some gain value and increases each gain value by a small amount to see if the average error reduces. The average error is the squared error over all the time steps divided by the number of time steps. If the average error reduces with an increase in the coefficient, it tries once again with a higher increase. If it does not reduce the average error with an increase, it then reduces the coefficient. If the error reduces now, it reduces the coefficient by a higher amount. If the error still does not reduce, it resets the coefficient back to the old value. This step is then repeated for the other 2 coefficients and it is cyclically repeated till the desired error or the tolerance value for the delta in coefficients is reached.

There is one issue with running the Twiddle algorithm in this project. The cross track error can initially be very high with most set of gain coefficients such that the vehicle leaves the track often and the simulator gets stuck. In order to overcome this issue, the coefficients are initially tuned manually by trial and error, and once a set of coefficients is obtained with which the car does not leave the track often, it is used as a set of starting values for the Twiddle algorithm. Now, even if the car leaves the track, an artificially high penalty is added to increase the error value and the car is reset to the initial position on the simulator to try the next set of coefficients. 

Using this Twiddle approach, a final set of parameters was obtained which gave an average error of 0.29. With this final set of parameters, the car does not leave the track and drives for a major part of the time in the centre of the track. The code using this Twiddle algorithm can be found in the `twiddle` branch of this project Github repository. In this code, the simulator is fed one set of parameters, evaluated for one lap and then the position of the car is reset to try another set of parameters. This happens repetitively in a cyclical manner and the simulator was run non-stop for almost 5 hours. 

The set of parameters obtained from the Twiddle algorithm is now used in the final code (master branch of this project repository) for the actual simulation.

### Effect of P, I, D components

As mentioned in the above segment, the 3 gain values are tuned using a hybrid approach of manual trial and error and the Twiddle algorithm. In order to save time and issues associated with the vehicle leaving the track, the parameters are tuned manually initially and then fed as a set of starting values for the Twiddle algorithm to fine-tune. The approach used in the manual trial and error method was to first tweak each parameter individually one after the other to study their effect. The following observations were made during the process of manual tuning:

In the absence of any control, that is, with all the gain values set as 0, it is observed that the car moves straight and veers out of the track since it is unable to make turns ([video link](./writeup_videos/all_gains_0.mp4)). This is because, zero gain values result in a control output (steering angle value) of 0, regardless of the error (CTE) in the system. 

1. Effect of Proportional Gain

When the proportional gain or Kp value is set as -1 (while keeping the other gains at 0), we see that the car starts turning inwards towards the centre of the track but overshoots and again comes back, then again overshoots, moving in an oscillatory manner with ever increasing amplitude, and then it finally becomes too unstable and leaves the track as the speed increases ([video link](./writeup_videos/kp_minus_1.mp4)). 

When this gain is reduced to a smaller value of -0.1, we see an improvement. The oscillations are reduced in amplitude and are slower, but then at higher speeds, the car becomes unstable and leaves the track. ([video_link](./writeup_videos/kp_minus_pt1.mp4))

The Kp value is kept constant at -0.1 while studying the effects of the remaining 2 gain parameters.

2. Effect of Derivative Gain

When a derivative gain or Kd value of -0.1 is tried, we see that the oscillations are reduced compared to the case when only a proportional controller is used, but still the oscillations become unstable at higher speed and the car ultimately veers off from the track([video link](./writeup_videos/kd_minus_pt1.mp4)).
When we use a higher Kd value of -1.0, we observe that the oscillations are no more clearly visible, but we see that the car drifts towards the right side of the lane every now and then after a few seconds and is pulled back to the centre([video link](./writeup_videos/kd_minus_1.mp4)). This drifting effect could be due to the accumulation of errors, the residual or steady state error and can be removed using an integral controller.

3. Effect of Integral Gain

As mentioned above, to reduce the residual error, the integral gain or Ki value is tweaked. With a value of -0.1, it turns sharply to one side and gets off the track at the very beginning ([video link](./writeup_videos/ki_minus_pt1.mp4)). Probably, this gain value is too large. So, it is set to a lower value of -0.01 and the simulation is rerun. It is now observed that the car starts oscillating, but after a certain distance, the oscillations reduce, and the car is stable. There is also no specific drifting to the right side of the track([video link](./writeup_videos/ki_minus_pt01.mp4)). To reduce these initial oscillations, an even smaller value of -0.001 is tried and it is found that there are no significant oscillations observed in the initial segments of the lap, but later on we do see some zig-zag movement at higher speeds([video link](./writeup_videos/ki_minus_pt001.mp4)), where the car sometimes touches the edges of the track during steep turns. 

Finally, this set of manually tuned parameters is fed to the Twiddle algorithm for automatic optimization/tuning.

## Rubric Items
                                                                
### Compilation
                                                                
- The code compiles correctly without errors, using cmake and make.

### Implementation

- The PID procedure follows what was taught in the lessons - 1. Calculation of the CTE, its derivative and its integral 2.Computation of the control output using a weighted sum of these error terms (the weights being the optimized set of gain parameters)

### Reflection

- The effect of each of the P, I, D components in the implementation is described in the above section in the writeup.
- A description of how the final hyperparameters were chosen is also given in the above section in the writeup.
                                                                
### Simulation                 

- The vehicle successfully drives a lap around the track safely. The tyres do not leave the drivable portion of the track surface throughout a lap.