**TB6612FNG**

**1.Parameters and Design Constraints:**

operating voltage = 15V max (hence it can easily operate on our 11.1 V battery)

peak current =  3.2A(we chose motors whose peak stall current is very less than 3.2 so our driver can handle max possible load without any issues)

continuous current = 1.2A

smaller and suitable for techfest



Because we are using **TB6612FNG ,** we need a dedicated buck converter as it does not feature inbuild 5V regulator**.**

**2. Condition for damage and prevention**

if motors physically lock up , then it would draw maximum current(stall current) , if it exceeds the peak threshold , it would cause overheat and permanent damage



to fix this we use motors with less peak stall current and also we can implement software timeout in esp32 that cuts power when such scenario occurs(eg if stuck for >1sec)



**Source of Specifications = https://robu.in/product/motor-driver-tb6612fng-module-performance-ultra-small-volume-3-pi-matching-performance-ultra-l298n/**

