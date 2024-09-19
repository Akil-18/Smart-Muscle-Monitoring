![image](https://github.com/user-attachments/assets/c9f50694-41e5-4b65-bc89-ea9a9235157b)


**(Ongoing Project)**




**1. AIM**

_(I) Purpose_

   * The purpose of the Smart Muscle Monitoring Project is to monitor muscle activity in real-time using sensors like the ICM20948 (9-DOF IMU) and MyoWare EMG sensors.
  
   * The project aims to analyze muscle movements and provide feedback on exercise form, which can help in optimizing workout routines and preventing injuries.
  
   * Additionally, the project incorporates AWS IoT connectivity to store and analyze the data in the cloud.


_(II) Inputs_

  1. ICM20948 (9-DOF IMU Sensor): Provides accelerometer, gyroscope, and magnetometer data.
    
  2. MyoWare EMG Sensor: Measures muscle activity by detecting the electrical potential generated by muscle cells.


_(III) Outputs_

  1. Cloud Storage and Analysis: Sensor data stored in AWS IoT for further analysis.
    
  2. ML-Based Feedback:
     
   * Alerts when muscle fatigue is detected during exercises.

   * Real-time feedback on exercise form to help improve technique.
  
   * Actionable insights delivered through a mobile app.




**2. FUNCTIONAL REQUIREMENTS**

_(I) Sensor Initialization_

   * Acquire continuous data from ICM20948 and EMG Sensor


_(II) Data Acquisition_

   * Continuously acquire and buffer data from the ICM20948 and MyoWare sensors.
  
   * Store data temporarily in RAM when Wi-Fi is connected.


_(III) Buffering Strategy_

   * Wi-Fi Connected: Buffer data in internal RAM for quick access and periodic flushing to the cloud.
  
   * Wi-Fi Lost: Shift to SRAM (23LC512) for buffering data when Wi-Fi connectivity is lost.
  
   * Wi-Fi Restored: Flush the buffered data from SRAM to the cloud once Wi-Fi is reconnected, then clear SRAM and resume buffering in internal RAM.


_(IV) Data Transmission_

  * Send the buffered data to AWS IoT when Wi-Fi is connected.
  
  * Ensure seamless transition between Wi-Fi states without data loss.


_(V) Sensor Fusion_

   * Implement Sensor Fusion algorithms to combine data from the accelerometer, gyroscope, and magnetometer to produce accurate and stable orientation data (roll, pitch, yaw).
  
   * Use Kalman Filter or Madgwick Filter for sensor fusion.


_(VI) Machine Learning Integration_

   *  Utilize ML models hosted on AWS to analyze sensor data in real-time.
  
   * Detect muscle fatigue and evaluate exercise form based on sensor data.
  
   * Provide real-time feedback to users based on the ML analysis.


**3. SCHEMATIC DIAGRAM**

![image](https://github.com/user-attachments/assets/b129d03f-13da-4436-a5fd-f8e6e673fc4c)

