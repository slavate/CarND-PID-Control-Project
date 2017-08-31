# Write-Up for Project CarND-Controls-PID 
Self-Driving Car Engineer Nanodegree Program

---
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it with 4 coefficients as parameters, Kp, Kd, Ki and max speed:   
**windows - `pid.exe 0.1 10 0.0006 20`**  
**ubuntu  - `./pid 0.1 10 0.0006 20`**

## Parameter tuning

At first I used the same values for Kp, Kd, Ki which we used in the class. Afterwards I tried to tune them manually and already on the second attemp th ecar was successfully driving around the track. However it was clear to me that manually find the best parameter combination would be almost unpossible. So I implemented twiddle approach within python script, it can be found in 'twiddle.py'. Moreover I found out that calculating integral error based on all the cte history doesn't make sence, because it will be growing all the time and at some moment the proportional and differential parts will not have any influence on controller. So I defined another hyperparameter htime (main.cpp line 48), which should be tuned. In general the behaviour of Kp and Kd was similar to the class examples. 


