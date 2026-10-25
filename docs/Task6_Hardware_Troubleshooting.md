**Fault -** Esp32 reset and loses its wi-fi connection whenever motors accelerate from start.



**Cause-** "BROWNOUT" - when dc motors start they draw massive currents which are called "stall current" . If this requirement is not fulfilled then voltage or entire system drops. If voltage of esp32 drops below its operational volage(2.5V-3.0V) , then esp32 triggers a safety reboot.



**Diagnostic Check -** Connect a multimeter across esp32's 5v and ground pin. then command robot to drive at full speed from rest. If a sudden drop is recorded(e.g. 5V to 4V) , we have confirmed the power sag and reason for reset.



**Fix -** Add a capacitor between Motor and Motor Driver . Capacitor feeds the initial current spike to so battery voltage remains stable for all components . The fix is confirmed when robot can high achieve speed from sudden start without resetting esp32 or disconnecting wifi.

