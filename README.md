# VitalSignsBabyIncubator
# مشروع حاضنة ذكية لمراقبة صحة الأطفال الخدج (Smart Baby Incubator Project)

## Brief Description (وصف موجز)

This project is a prototype of a smart baby incubator designed to monitor the vital signs and environmental conditions crucial for premature infants. It integrates several sensors to measure SpO2, heart rate, skin temperature, incubator temperature, and humidity. The system also includes movement detection with an alarm to alert caregivers to potential issues. The incubator features automatic and manual temperature control, ensuring a stable and safe environment for the baby.

This project was developed as an academic endeavor to explore the application of microcontroller technology in healthcare settings. It is not intended for medical use.

يهدف هذا المشروع إلى تصميم نموذج أولي لحاضنة ذكية للأطفال الخدج، قادرة على مراقبة العلامات الحيوية والظروف البيئية الهامة. يدمج المشروع مجموعة من الحساسات لقياس تشبع الأكسجين في الدم (SpO2)، ومعدل ضربات القلب، ودرجة حرارة الجلد، ودرجة حرارة الحاضنة، والرطوبة. يتضمن النظام أيضًا خاصية اكتشاف حركة الطفل مع إنذار لتنبيه مقدمي الرعاية إلى أي مشكلات محتملة. تتميز الحاضنة بنظام تحكم آلي ويدوي في درجة الحرارة، مما يضمن بيئة مستقرة وآمنة للطفل.

تم تطوير هذا المشروع كجهد أكاديمي لاستكشاف تطبيق تكنولوجيا المتحكمات الدقيقة في البيئات الصحية. وهو ليس مخصصًا للاستخدام الطبي.

## Features (الميزات)

* **SpO2 and Heart Rate Monitoring (مراقبة تشبع الأكسجين في الدم ومعدل ضربات القلب):** Measures the baby's blood oxygen saturation and heart rate using a MAX30100 sensor.
* **Skin Temperature Measurement (قياس درجة حرارة الجلد):** Monitors the baby's skin temperature using a DS18B20 sensor.
* **Incubator Temperature and Humidity Control (التحكم في درجة حرارة ورطوبة الحاضنة):** Measures and controls the incubator environment using a DHT11 sensor.
* **Movement Detection with Alarm (اكتشاف الحركة مع إنذار):** Detects significant baby movements using an MPU6050 gyroscope and triggers an audible and visual alarm.
* **Automatic Temperature Control (التحكم الآلي في درجة الحرارة):** Automatically adjusts the heater and fan to maintain the incubator temperature around a default target.
* **Manual Temperature Control (التحكم اليدوي في درجة الحرارة):** Allows the user to set a specific target temperature using a keypad.
* **Data Display and Status Reporting (عرض البيانات والإبلاغ عن الحالة):** Provides real-time feedback on sensor readings and system status.

## Hardware Components (المكونات الصلبة)

* Microcontrollers:

    * Arduino Nano IoT 33 (الرئيسي)
    * Arduino Uno (لـ MAX30100)
* Sensors:

    * MAX30100 Pulse Oximeter and Heart Rate Sensor
    * DS18B20 Waterproof Temperature Sensor
    * DHT11 Temperature and Humidity Sensor
    * MPU6050 Accelerometer and Gyroscope
* Actuators:

    * Relay Module (for heater and fan control)
    * Heater (AC 220V)
    * Fan
    * Buzzer
    * LED
* Input:

    * 4x4 Keypad
* Other:

    * Assorted wires, resistors, breadboard, power supplies, etc.
    * AutoCAD file for incubator maquette (for laser cutting)

## Wiring Instructions (تعليمات التوصيل)

Detailed wiring diagrams and instructions should be included here. Since I cannot create visual diagrams, I'll provide a textual description. It is highly recommended to create a Fritzing diagram or similar and include it in your GitHub repository.

**Important Notes:**

* Ensure all power is disconnected before making any connections.
* Double-check all connections to avoid short circuits or damage to components.
* Use appropriate gauge wire for each connection.
* Refer to the datasheets for each component for detailed pinout information.

**Arduino Nano IoT 33 Connections:**

* **DHT11:**

    * VCC to 3.3V
    * Data to Digital Pin 2
    * GND to GND
* **DS18B20:**

    * VCC to 3.3V
    * Data to Digital Pin 4 (with a 4.7kΩ pull-up resistor to 3.3V)
    * GND to GND
* **MPU6050:**

    * VCC to 3.3V
    * GND to GND
    * SCL to A5
    * SDA to A4
    * INT (if used) to a digital pin (e.g., Digital Pin 3)
* **Keypad:**

    * Row pins (13, 12, 11, 10)
    * Column pins (A3, A2, A1, A0)
* **Buzzer:**

    * Positive to Digital Pin 8
    * Negative to GND
* **LED:**

    * Anode (long leg) to Digital Pin 9 (with a 220Ω resistor in series)
    * Cathode (short leg) to GND
* **Heater Relay:**

    * Control pin to Digital Pin 6
    * Connect the relay's common (COM), normally open (NO), and normally closed (NC) terminals to control the AC heater. **(Caution: High Voltage - Exercise extreme care!)**
* **Fan Relay:**

    * Control pin to Digital Pin 7
    * Connect the relay's common (COM), normally open (NO), and normally closed (NC) terminals to control the fan.
* **Serial Communication with Arduino Uno:**

    * Nano IoT 33 RX to Arduino Uno TX
    * Nano IoT 33 TX to Arduino Uno RX
    * Connect the GND pins of both Arduino boards.

**Arduino Uno Connections:**

* **MAX30100:**

    * VCC to 5V
    * GND to GND
    * SCL to A5
    * SDA to A4
    * INT to Digital Pin 2 (or other digital pin, adjust code accordingly)
* **Serial Communication with Arduino Nano IoT 33:**

    * Uno RX to Nano IoT 33 TX
    * Uno TX to Nano IoT 33 RX
    * Connect the GND pins of both Arduino boards.

## How to Use (كيفية الاستخدام)

1.  **Power Up:** Connect the power supplies to the Arduino boards and the heater/fan circuit. **Exercise extreme caution with mains voltage!**
2.  **Initial Setup:**

    * The system starts in AUTO mode.
    * The incubator will attempt to maintain the default target temperature (35°C).
    * The system will start reading sensor data.
3.  **Automatic Mode:**

    * The heater will turn on if the incubator temperature is below the target temperature minus the hysteresis (0.5°C).
    * The fan will turn on if the incubator temperature is above the target temperature plus the hysteresis.
4.  **Manual Mode:**

    * Press '\*' on the keypad to enter manual temperature setting mode. The system will prompt you to enter the desired target temperature.
    * Use the keypad to enter the target temperature (e.g., "36.5").
    * Press '#' to confirm the entered temperature. The system will switch to MANUAL mode and use this new target temperature.
    * The heater and fan will operate as in automatic mode, but using the manually set target temperature.
5.  **Return to Auto Mode:**

    * Press 'C' on the keypad to return to automatic mode. The system will revert to the default target temperature.
6.  **Movement Detection:** If the MPU6050 detects significant movement, the buzzer will sound, and the LED will flash.
7.  **Skin Temperature Monitoring:** If the baby's skin temperature goes outside the safe range (25°C to 37.5°C), a warning message will be displayed.
8.  **Data Display:** The system will print sensor readings and system status to the serial monitor every 2 seconds.

## Limitations (القيود)

* **Prototype Only:** This is a prototype and has not undergone medical device certification. It is not intended for clinical use.
* **Sensor Accuracy:** The accuracy of the sensors may vary. Calibration may be required for more precise measurements.
* **Power Supply Complexity:** The system requires multiple power supplies with different voltage levels, which can be complex and potentially hazardous (especially the AC heater).
* **Sensor Vulnerability:** The sensors are exposed and may be susceptible to damage.
* **Software Serial Limitations:** SoftwareSerial can sometimes have timing issues, especially at higher baud rates or when other interrupts occur.
* **I2C Address Conflict:** The MAX30100 sensor's fixed I2C address requires using a separate microcontroller, increasing system complexity.

## Disclaimer (إخلاء المسؤولية)

**This project is for educational and experimental purposes only. It is not a medical device and should not be used for any actual medical treatment or diagnosis. The authors are not responsible for any consequences resulting from the use or misuse of this project. Users assume all risks associated with building and operating this system.**

**هذا المشروع مخصص للأغراض التعليمية والتجريبية فقط. إنه ليس جهازًا طبيًا ولا ينبغي استخدامه لأي علاج أو تشخيص طبيعي. المؤلفون ليسوا مسؤولين عن أي عواقب ناتجة عن استخدام أو إساءة استخدام هذا المشروع. يتحمل المستخدمون جميع المخاطر المرتبطة ببناء وتشغيل هذا النظام.**

## Credits/Acknowledgements (شكر وتقدير)

We would like to thank the following individuals for their contributions to this project:

* Nourhan Atef
* Mohamed Osama
* Hager Abdelrahman
* Yehia Mohamed
* Mohaned Yasser
* The code for this project was developed with the assistance of Gemini LLM.

