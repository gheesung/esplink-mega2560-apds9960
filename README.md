# esplink-mega2560-apds9960

This sketch get the uses the following library.

## SparkFun APDS9960 library
Sparkfun library (SparkFun_APDS9960.h) has a checking for the valid device ID. For compatible module bought from China, this value has to change according. 

```C
// Acceptable device IDs 
#define APDS9960_ID_1           0xA8
#define APDS9960_ID_2           0x9C 
```

## ELClient library
This library (https://github.com/jeelabs/el-client) managed the communication between ESP-link and the Mega2560.

More details about the setup can be found at http://iotdiary.blogspot.sg/2017/11/esp-link-on-atmega2560esp8266-board.html
