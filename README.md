# smart-robot-car
Project for Medical Robotics and Operation Room Technology 

# Tutorial files of the smart car
https://elegoo-downloads.oss-us-west-1.aliyuncs.com/stem-downloads/02%20Robot%20Car%20Kits/02%20Smart%20Robot%20Car%20V4/ELEGOO%20Smart%20Robot%20Car%20Kit%20V4.0%202024.01.30.zip

# 05/02/2026
The motion and direction control of the four motors were achieved in the move_direction folder.

# 12/02/2026
## Ultrasound Sensor Scan 
In the code in the ultrasound_scan folder, I implemented a method to scan 15-165 degrees using an ultrasound sensor and build a radar map.
## Obstacle
The code I put in the obstacle_move folder implements obstacle detection using the ultrasonic module and enables obstacle avoidance driving.

However, obstacle avoidance algorithm I provide is very rudimentary, have very low driving efficiency, and is prone to errors. Therefore, I I did some research and found the following papers.

- [1] T. T. Cheng and M. N. Mahyuddin, "Implementation of behaviour-based mobile robot for obstacle avoidance using a single ultrasonic sensor," in *2009 Innovative Technologies in Intelligent Systems and Industrial Applications*, 2009, pp. 244–248.

- [2] Q. Liu, Y. G. Lu, and C. X. Xie, "Optimal genetic fuzzy obstacle avoidance controller of autonomous mobile robot based on ultrasonic sensors," in *2006 IEEE International Conference on Robotics and Biomimetics*, Dec. 2006, pp. 125–129.

- [3] M. C. De Simone, Z. B. Rivera, and D. Guida, "Obstacle avoidance system for unmanned ground vehicles by using ultrasonic sensors," *Machines*, vol. 6, no. 2, p. 18, 2018.

- [4] A. Shitsukane, W. Cheruiyot, C. Otieno, and M. Mvurya, "Fuzzy logic sensor fusion for obstacle avoidance mobile robot," in *2018 IST-Africa Week Conference (IST-Africa)*, May 2018, p. 1.

## Pseudocode for software integration (Provided by Ollie)

graph TD
    Start[Start Loop] --> Emergency{Front < 8cm?}
    
    %% Emergency Logic
    Emergency -- Yes --> EmergencyTurn[Set Committed Turn]
    EmergencyTurn --> LockTurn[Lock Turn Timer]
    LockTurn --> CheckSweep
    Emergency -- No --> CheckSweep
    
    %% Sweep Logic
    CheckSweep{Time to Sweep?}
    CheckSweep -- Yes --> Scan[Servo Scan & Update Distances]
    Scan --> FindGaps
    CheckSweep -- No --> FindGaps[Find Open Gaps]
    
    %% Trap Logic
    FindGaps --> Trapped{Gaps Empty?}
    Trapped -- Yes --> TimerCheck{Timeout > 3s?}
    TimerCheck -- Yes --> Stop([STOP ROBOT])
    TimerCheck -- No --> Start
    Trapped -- No --> CalcPath
    
    %% Path Planning
    CalcPath[Find Widest Gap Center] --> CalcSpeed[Calc Speed based on Front Dist]
    
    %% Smoothing
    CalcSpeed --> Override{Emergency Locked?}
    Override -- Yes --> ApplyHardTurn[Apply Hard Turn]
    Override -- No --> SmoothTurn[Apply Smooth Weighted Turn]
    
    %% Output
    ApplyHardTurn --> Motors[Set Motors]
    SmoothTurn --> Motors
    Motors --> Wait[Wait 30-50ms]
    Wait --> Start