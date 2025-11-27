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

* Essential toolboxes/Add On: "Simulink support package for Arduino Hardware"  

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
- Be able to detect stall, overload and friction in real time


⚡ Electronic Connections

•	Pin 2 -> Encoder (Yellow wire)
•	Pin 3-> Encoder (White wire)
•	Pin 4 -> L298N In1 
•	Pin 5 -> L298N In2 
•	Pin 6 -> L298N Enable PWM 
•	Analog Pin 0 -> Current Sensor ACS712 (Analog voltage output)
•	L298N Out1 -> Encoder (red wire) 
•	L298N Out2 -> Encoder (black wire) 
•	Gnd dc -> Gnd l298N
•	Gnd dc-> Encoder (green wire)
•	5v dc -> Encoder (white wire)
•	GND-> ACS712 Gnd Pin 
•	5Vdc -> ACS712 Vcc Pin
•	12Vdc -> ACS712 (IP+)
•	ACS712 (IP-) -> input voltaje L298N


## ⚙️ Configuration & Usage

Instructions to run the simulation properly:

1.  **Open the Configuration parameters Ctrl + E**
 
- 1.1) Go to solver 
- 1.2) On the simulation time, be sure to have the start time at 0.0 and stop time as "inf" 
- 1.3) On the solver selection, be sure to have the fixed-step type and the solver in discrete (no continous state)

2.  **Go to the apps section**

- 2.1) Click on the arrow to show more apps
- 2.2) Scroll down to the final and select "Run on hardware board"  *IMAGEN*


4.  **Go to the hardware section:**

   - 3.1) On the left side should now appear the slection for a the hardware board
   - 3.2) Select the hardware board (on this case we used an arduino one)
   - 3.3) Make sure again your stop time is set at infinite by just writing "inf"
   - 3.4) Finally, to run or try the simulation it should always be played on the "Monitor & Tune" button 

⚡ Electronic Connections

•	Pin 2 -> Encoder (Yellow wire)
•	Pin 3-> Encoder (White wire)
•	Pin 4 -> L298N In1 
•	Pin 5 -> L298N In2 
•	Pin 6 -> L298N Enable PWM 
•	Analog Pin 0 -> Current Sensor ACS712 (Analog voltage output)
•	L298N Out1 -> Encoder (red wire) 
•	L298N Out2 -> Encoder (black wire) 
•	Gnd dc -> Gnd l298N
•	Gnd dc-> Encoder (green wire)
•	5v dc -> Encoder (white wire)
•	GND-> ACS712 Gnd Pin 
•	5Vdc -> ACS712 Vcc Pin
•	12Vdc -> ACS712 (IP+)
•	ACS712 (IP-) -> input voltaje L298N

---
## ⚙️ Simulink Control Architecture
In Simulink, these block diagrams were needed to get everything worked correctly. We are going to explain each one of them. 

### _ENCODER TO FILTER RPM_  

LINK TO THE FOLDER IMAGE OF THAT 

With this block sequence we convert the pulses from the encoder into RPM’s 

### _LOW PASS FILTER_ 

<img src="SIMULINK_ARCHITECTURE/LOW_PASS_FILTER.png" width="500" height="350">  

You will get the RPM’s with some noise, so we implemented a Low filter pass to get a cleaner view of the RPM signal, this will be very helpful, for you too. 

### _SUBSYTEM COMPARING THE TARGET WITH THE FILTERED RPM's_ 

<img src="SIMULINK_ARCHITECTURE/SUBSYSTEM.png" width="500" height="350">

You will need to set the values of the RPM’s you want so in order to do that in the Target variable, now inside of the Subsystem, what we did is that the Target variable is compared to the filtered RPM’s.  Here the subsystem uses the error to generate the P, I, D control signals within a plus/minus 8 bit range. 

P, I and D inside of the Subsystem

<img src="SIMULINK_ARCHITECTURE/SIGNAL GENERATOR_OF_P_I_and_D.jpg" width="500" height="350">

Signal processing to output

<img src="SIMULINK_ARCHITECTURE/SIGNALPROCESSING_TO_OUTPUT.jpg" width="500" height="350">

Then it adds the P, I, D signals to obtain the PWM signal also within a plus/minus 8 bit range. 

<img src="SIMULINK_ARCHITECTURE/HOW_TO_GET_THE_PWM_SIGNAL.jpg" width="500" height="350">

Then the PWM signal is processed  (AGREGAR MAS TEXTO)

### _CURRENT DETECTION_ 

<img src="SIMULINK_ARCHITECTURE/Current_Detection.jpg" width="500" height="350">

Here is the part where you can process the analog input of the Arduino that is connected to the ASC Current Sensor. It transforms the analog input into the actual value of the current based on the relationship of voltage to bits (5/1023), the voltage offset (2.515), and the volts/amperes resolution (0.66). It uses a Low Pass Filter to smooth out the inputs like we did in the RPM case. 

### _FAUL DETECTORS_ 

<img src="SIMULINK_ARCHITECTURE/FAULT_DETECTORS.jpg" width="500" height="350"> 

PARTE DEL CODIGO (COMO VEAN) 

[Code for the fault detector in real time](SIMULINK_ARCHITECTURE/FaultDetectionBlockCode.txt)

Then finally, our main purpose and the heart of this project is to get the Faults in real time, so we wrote a code capable of detect the faults (Overload, Friction and Stall) in real time, we put our values (Filtered RPM’s, Error and Filtered Current) and our Digital Outputs can be seen connecting some LED’s to the pins you declared in the blocks (later this repository we will explain about the physical connections). 






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



All project objectives were successfully met. We have designed and validated a complete dc fault motor





---





## 👥 Authors


**Project Team**
* José Eduardo Espinosa Marinero 
* Santiago Gonzalez Faraco
* Edson Antonio Lagunas Hurtado
* Paul Leonardo Morales Grunauer 
* Gabriel Alejandro Vizcarra Gutierrez


---


