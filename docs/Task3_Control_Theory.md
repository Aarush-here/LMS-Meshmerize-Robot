



**Role of P, I,D:**

Proportional (P) : Error - it is how much far is robot from line and how much should it turn to get back.

Integral(I): accumulation of past error which is recorded and rectified to avoid steady drift of robot out of path

Derivative (D): rate of change or error which dampens effect of P , so that robot doesn't wobble or oscillates violently.



steering constant = Kp(e) + Kd(de/dt) + Ki(integral(e.dt))



this constant is added to one side motor and removes from one which adds a overall turning effect .



**Reference : https://youtu.be/QoNkpnpvEqc?si=rDJJV8hCisfkpPbk**



**Selected Configuration: PD Control**

For speed purposes , the integral term is dropped as due to constant curves and turns , the term might overcorrect the steering constant. Also , 90 degree terms would result in high integral term . Proportional term would be sufficient to act for curves and to avoid the overcorrecting effect of I term , it is simply dropped. Also , PD is preffered because stability, fast dynamic response, and noise reduction matter more than eliminating tiny steady-state errors



**Sensor-to-Motor Conversion Effect:**

1.Error Calculation: IR array outputs a weighted centroid(from -2.0 to 2.0). perfectly aligned =  0. Hence, error (e) = 0.0 - centroid position. 

2.Sterring constant calculation: steering constant = (Kp(e))+(Kd(de/dt))

3.Applying it to motor = Now speed of motors are changed as so to perform turning effect , Left Motor = BaseSpeed+SteeringConstant and Right Motor = BaseSpeed-SteeringConstant. Example ,if robot drifts right (positive error is accumulated), the left motor speeds up and the right motor slows down to pull it left.



**Handling Oscillation and Lost-Line Conditions**

**Oscillation**:If the robot is violently wobbling left and right, the P-term is too high or the D-term is too low. The response is to first increase Kd to add more damping. If oscillation persists, decrease Kp.(it is more or less hit and miss , to fix it change Kp and Kd until perfect values are found.)

**Lost-Line Condition:** If the robot is out of track (e.g., all sensor readings drop to 0), the centroid function returns our set value of 11(which we chose in code because it is out of -2 and 2 range). The control loop detects this value and immediately commands to what to do next, we could either add some code so robot tries to get back to track , eg , move back or move left , or just simply hault by changing both motor speeds to 0 



**PD Control Pseudocode Sketch**

cpp

float current\_position = get\_weighted\_centroid();



// Lost-line safety check

if (current\_position == 11.0) {

&#x20;   stop\_motors();

&#x20;   return;

}



float error = 0.0 - current\_position;

float derivative = error - previous\_error;

float correction = (Kp \* error) + (Kd \* derivative);

previous\_error = error;



int left\_pwm = clamp(BASE\_SPEED + correction, MAX\_PWM);

int right\_pwm = clamp(BASE\_SPEED - correction, MAX\_PWM);



set\_motors(left\_pwm, right\_pwm);



\#coding was done with help of AI as I am not very comfortable in Cpp.

