# Max300003  
ESP32 focussed software and experimental documentation  

## Software  
This software derives, in parts, from the original work at  
* [MAX30001-EVSYS](https://os.mbed.com/teams/MaximIntegrated/code/MAX30001-MAX32630FTHR-ECG-Bioz-EVKIT/shortlog/)  (Emre.Eken)   
* [HeartyPatch](https://github.com/patchinc/heartypatch)  

Note that HeartyPatch/Protocentral has announced commercialisation of some of their products. Any reference to their commercial name, **HeartyPatch**, will be replaced with **HeartSensor** in order to avoid any potential commercial issues. As required, any software will retain acknowledgements of the original code and licences.  
   
Relevant software acknlowledgements are included, and if not, are meant to be included.  
It has been changed to work within the *Arduino ESP32* framework rather than the _esp-idf_ framework.  
As such, it is not meant to be production-grade, rather coded for experimenters.  
  
The code base comprises  
1. a version to support MAX30001-EVSYS  
1. a serial interface that sends data to a serial plotter over bluetooth  
1. an OTA initial load (requires your SSID/PASS)   
  
It is a work-in-progress and has been modified by an engineer rather than a real software programmer.  

The code is being developed using VSCODE on Windows10. However, some modules work using the Arduino IDE.  
  
### Important improvements include: 
* Ability to use MAX30001-EVSYS for register setting evaluation  
* Updated register initialization, rather than the original 'calibration mode'  
* [OTA upload](https://lastminuteengineers.com/esp32-ota-updates-arduino-ide/)  
* Serial bluetooth for use with Arduino monitor or [SerialPlot](https://bitbucket.org/hyOzd/serialplot/src)  
  
## Hardware  
In the hardware section there are details of  
* connecting the common mode ground for use with the USB cable  
* attaching a three lead system for more stable measurements  
* (Future chest strap)  

## ToDo
Extensive !!  
