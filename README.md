# Adra Arduino SDK

    Test passed on MEGA2560, DUE.

 >Sdk is use the HardwareSerial for serial comminucate, you must config the correct **serial_port** is **Serial1**, **rtsPin** is **4** pin. you can init like this.

    #include <adra_api.h>
    AdraApi adraApi(1,4);

<img src="./doc/line1.png" style="width:600px">

>AdraApi Constructor is 

    AdraApi(uint8_t serial_port, uint8_t rts);


>After that, you need to set the baud for connect. for example, baud is 115200. (**if you need to change the baud rate, you can refer [change the baud rate](./doc/change_id.md) with [Adra Assistant](https://www.umbratek.com/download-center).**)

    adraApi.connect(115200);

>Then, you can comminucate with adra. for example, get the adra's current position.

    float buf;
    int8_t ret = adraApi.get_pos_current(1,&buf); //1 is the id of adra
    Serial.println(ret);
    Serial.println(buf);

> All the api is in the [adra_api.h](./src/adra_api.h)

# Install SDK to Arduino

Download the SDK ,and unzip it to the libraries path of Arduino. Then restart Arduino IDE. Then you can see the sdk library in **Sketch->Include Library->ADRA_SDK_ARDUINO**

<!-- <img src="./doc/sdk.png" style="width:600px"> -->

# Reference example

You can reference example in the sdk. Find the exmaple in **File->Examples->ADRA_SDK_ARDUINO**

<!-- <img src="./doc/example.png" style="width:600px"> -->