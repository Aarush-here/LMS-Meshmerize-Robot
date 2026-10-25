**Identification of Faulty Sensor -** For physical bumps , faulty sensor will jump reading to 0 or something, whereas other sensors will be steady. By simply observing abnormalities in data ,we can find faulty sensor.





1. **Protocol Selection -**

Wi-Fi via UDP

Latency and speed - It fires continuous packets without waiting from response .This ensures lowest latency , so tracking occurs fast. This will ensure fast response of robot to commands.



Drawback - Wifi consumes more power than Bluetooth , but out 11.1V battery in more than sufficient to afford this.



Ease of use- esp32 can host its own local network so robot can directly be connected to phone or laptop without any external router.

