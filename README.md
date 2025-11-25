# Fault-Detection-in-DC-Motors-for-a-Mechanical-Piston-System Fault detect 


![MATLAB](https://img.shields.io/badge/MATLAB-0076A8?style=for-the-badge&logo=mathworks&logoColor=white) ![Simulink](https://img.shields.io/badge/Simulink-orange?style=for-the-badge&logo=mathworks&logoColor=white)


---



## 📋 Prerequisites



Before starting this project, make sure you have the following: 

**Knowledge:**

* Basic understanding of DC motor operation with encoder  

* Familiarity with control theory, particularly PID controllers 

* Experience working within the MATLAB/Simulink environment 

* Basic experience with the usage of an Arduino Uno  

* CATIA – Basic 

* Basic electrical/electronic hookups 


 

**Hardware:** 

* A computer running Windows, Linux, or macOS that can support MATLAB/Simulink 

* Arduino Uno 

* 3D Printer 

* H-bridge (L298N)
 
* Motor with an encoder (Pololoo 12 V DC Motor with encoder 30:1 gear ratio)
  
* Current sensor for arduino ( ACS 712 ) 
  


**Software:** 

* MATLAB (recommended version: R2023b or later) 

* Simulink 

* Essential toolboxes/Add On: 

* Simulink support package for Arduino Hardware  

---



## 📖 Introduction



In a piston assembly line, if one of the pistons begins to exhibit any type of malfunction, this project enables real-time fault detection. Such capability contributes directly to the preventive maintenance of the system.
We present here the complete program for fault detection. Specifically, this project provides a comprehensive simulation of fault detection in a DC motor, which serves as the foundation of our digital twin. The digital twin is structured around metrics that allow the identification of deviations, the detection of faults, and the anticipation of potential issues.
Our fault detection framework functions as a practical application of the Digital Twin concept, since it integrates data from the physical system to assess its condition and predict failures through the digital model developed in Simulink.



**Project Objectives:**

- Develop health indicators for a DC motor using MATLAB/Simulink
- Acquire motor signals to calculate RPM and current
- Design the system and implement it in a real-world case
- Create a notification system for each health indicator
- Validate the project’s performance using scopes to obtain graphs of the system’s behavior
- Be able to detect stall, overload and friction


---


## ⚙️ Configuration & Usage

Instructions to run the simulation properly:

1.  **Open the Configuration parameters Ctrl + E**
 
 1.1) Go to solver 
 1.2) On the simulation time, be sure to have the start time at 0.0 and stop time as "inf" 
 1.3) On the solver selection, be sure to have the fixed-step type and the solver in discrete (no continous state)

2.  **Go to the apps section**
   
    2.1) Click on the arrow to show more apps
    2.2) Scroll down to the final and select "Run on hardware board"  *IMAGEN*


3.  **Go to the hardware section:**

    3.1) On the left side should now appear the slection for a the hardware board
    3.2) Select the hardware board (on this case we used an arduino one)
    3.3) Make sure again your stop time is set at infinite by just writing "inf"
    3.4) Finally, to run or try the simulation it should always be played on the "Monitor & Tune" button 


---
## ⚙️ Simulink Control Architecture







---

## 💻 Codes and Programming

This code is the only code used. It needs to be inside of a Mathlab function block which helps us with the alerts by using the filtered RPM the error and the filteres obtained current.

The function helps us to compare the inputs and send signals to the arduino output pins so we can conect our alarms. In this case we connected a series of red LEDS so they turned on when they detect the motor has friction, overload and stall.

Connection in the simulink is as seen here: *IMAGEN*

```matlab
function [fault_overload, fault_friction, fault_stall] = ...
         fault_detector(rpm, err, i)

% ----- Parámetro del filtro (equivalente a ~100 ms) -----
alpha = 0.95;    
% Mientras más alto = más “parecido a delay de 100 ms”

% ----- Variables persistentes ULTRA pequeñas -----
persistent rpm_d err_d i_d

if isempty(rpm_d)
    rpm_d = rpm;
    err_d = err;
    i_d   = i;
end

% ----- Filtro exponencial (simula retraso sin usar RAM) -----
rpm_d = alpha * rpm_d + (1 - alpha) * rpm;
err_d = alpha * err_d + (1 - alpha) * err;
i_d   = alpha * i_d   + (1 - alpha) * i;

% ----- Márgenes definidos como ±15% respecto al valor filtrado -----
i_max   = i_d   * 1.15;
err_max = err_d * 1.15;
rpm_min = rpm_d * 0.85;

% ----- Inicializar -----
fault_overload = false;
fault_friction = false;
fault_stall    = false;

% ----- Stall -----
if (i > i_max) && (err > err_max) && (rpm < 10)
    fault_stall = true;
end

% ----- Overload -----
if (i > i_max) && (err > err_max) && (rpm < rpm_min)
    fault_overload = true;
end

% ----- Friction -----
if (rpm < rpm_min) && (i > 0.7*i_max) && ~(fault_stall || fault_overload)
    fault_friction = true;
end

end
```



---



## ✅ Conclusion



All project objectives were successfully met. We have designed and validated a complete autonomous navigation system for a library robot.



**Key Findings:**

* The combination of **PRM planning** and a **double-loop PI/PI controller** provides a robust and highly accurate solution.

* Simulation results show minimal tracking error for X, Y, and Theta variables.

* This simulation serves as a validated "digital twin," significantly reducing the risk and cost of physical prototyping.


---


## 🔜 Future Improvements


* **Physical Prototype:** Construct a real-world robot based on the parameters validated in this simulation.

* **Dynamic Obstacles:** Implement local path planning (e.g., VFH algorithm) to avoid moving people or objects in real-time.

* **Sensor Integration:** Simulate LiDAR or Camera inputs for SLAM (Simultaneous Localization and Mapping) to replace the ideal map knowledge.


---




## 📚 Additional Resources



* [MathWorks: Mobile Robot Algorithms (Robotics System Toolbox)](https://www.mathworks.com/help/robotics/mobile-robot-algorithms.html)

* [Probabilistic Roadmaps (Kavraki et al., 1996)](https://doi.org/10.1109/70.508439)

* [Springer Handbook of Robotics](https://link.springer.com/book/10.1007/978-3-319-32552-1)


---



## 👥 Authors


**Project Team**
* José Eduardo Espinosa Marinero 
* Santiago Gonzalez Faraco
* Edson Antonio Lagunas Hurtado
* Paul Leonardo Morales Grunauer 
* Gabriel Alejandro Vizcarra Gutierrez


---


