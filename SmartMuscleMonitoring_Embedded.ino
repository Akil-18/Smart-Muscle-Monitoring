//WiFi and AWS 
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

//Communication Protocols - SPI and I2C
#include <Wire.h>
#include <SPI.h>

//Sparkfun 9DOF ICM20948 Library
#include "ICM_20948.h"
#define AD0_VAL 1

//WiFi and AWS Credentials 
#include "secrets.h"

// 23LC512 SRAM setup
#define SRAM_CS 5
#define SRAM_SIZE 65536
#define BUFFER_SIZE 512
#define ICM_LED 25
#define WIFI_LED 33
#define WIFI_TIMEOUT_MS 20000

//Creating an Object of Class ICM_20948_I2C
ICM_20948_I2C myICM;

//Buffer1 is the Internal RAM buffer. It buffers sensor sensor data for 1 second and sends to AWS
float buffer1[BUFFER_SIZE];
int buffer1Index = 0;
bool usingBuffer1 = true;

//sramBuffer is the 23LC512 External SRAM Buffer. Sensor data is buffered in SRAM when WiFi is disconnected
float sramBuffer[BUFFER_SIZE];
int sramBufferIndex = 0;

//WiFi Connection Flag
bool wifiConnected = false;

//Creating a Mutex for buffers for Synchronization
SemaphoreHandle_t bufferMutex;

WiFiClientSecure net;
PubSubClient client(net);

//Function Description: To blink status LEDs for easy debugging
void blink(uint8_t PinVal){
  digitalWrite(PinVal, 1);
  delay(250);
  digitalWrite(PinVal, 0);
  delay(250);
}

//Function Description: To Connect to AWS IoT Core
void connectAWS() {
    if (WiFi.status() != WL_CONNECTED) 
        return;
        
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);

    while (!client.connected()) {
        Serial.print("Connecting to AWS IoT...");

        if (client.connect(AWS_IOT_CLIENT_ID)) {
            Serial.println("Connected to AWS IoT");
        }

        if (WiFi.status() != WL_CONNECTED) 
            break; //Tries to connect to WiFi
        else {
            Serial.print("Failed. Error state=");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

//Function Description: To publish data in cloud
void publishToAWS(float *icmData) {
    String payload = "{";
    payload += "\"accX\":" + String(icmData[0]) + ",";
    payload += "\"accY\":" + String(icmData[1]) + ",";
    payload += "\"accZ\":" + String(icmData[2]) + ",";
    payload += "\"gyrX\":" + String(icmData[3]) + ",";
    payload += "\"gyrY\":" + String(icmData[4]) + ",";
    payload += "\"gyrZ\":" + String(icmData[5]) + ",";
    payload += "\"magX\":" + String(icmData[6]) + ",";
    payload += "\"magY\":" + String(icmData[7]) + ",";
    payload += "\"magZ\":" + String(icmData[8]);
    payload += "}";

    if (client.publish(AWS_IOT_TOPIC, payload.c_str())) {
        Serial.println("Data published to AWS IoT");
    } else {
        Serial.println("Failed to publish data");
    }
}

//Function Description: Calls publishToAWS function when the buffer has data. Then, the buffer index is cleared
void flushBufferToAWS() {
    for (int i = 0; i < buffer1Index; i += 9) {
        publishToAWS(&buffer1[i]);
        delay(100); // Avoid flooding the network
    }
    buffer1Index = 0;
}

//Function Description: SRAM buffer is used when WiFi is not connected. Once WiFi restores the function calls publishToAWS function as long as there is data in Buffer. Then Buffer is cleared
void flushSramBufferToAWS() {
    Serial.println("Flushing SRAM buffer to AWS...");
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SRAM_CS, LOW);
    for (int i = 0; i < sramBufferIndex; i += 9) {
        publishToAWS(&sramBuffer[i]);
        delay(100); // Avoid flooding the network
    }
    digitalWrite(SRAM_CS, HIGH);
    SPI.endTransaction();
    sramBufferIndex = 0;
}

//Function Description: Initialises Sparkfun 9DOF ICM20948
void Sparkfun_9DOF_Initialization() {
    bool initialized = false;
    while (!initialized)
    {
       myICM.begin(Wire, AD0_VAL);
       if(myICM.status != ICM_20948_Stat_Ok)
       {
          Serial.println("Sensor initialization failed. Retrying...");
          blink(ICM_LED);
          delay(500);
       }

       else
       {
          //Sets the ICM_LED HIGH which indicates that the sensor is initialized and working
          initialized = true;
          digitalWrite(ICM_LED, HIGH);
          Serial.println("Sensor initialization successful");
       }
    }
}


//Functional Description: Periodically checks if WiFi is connected, else attempts to connect
void KeepWifiAlive(void * Parameters) {
    while (1) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!wifiConnected) {
                Serial.println("WiFi Connected: " + WiFi.localIP().toString());
                wifiConnected = true;
                digitalWrite(WIFI_LED, HIGH);
                xSemaphoreTake(bufferMutex, portMAX_DELAY);
                if (!usingBuffer1) {
                    flushSramBufferToAWS();
                }
                xSemaphoreGive(bufferMutex);
            }
            client.loop();
            vTaskDelay(10000 / portTICK_PERIOD_MS);
        } 
        
        else {
            blink(WIFI_LED);
            Serial.println("Connecting to WiFi...");
            WiFi.mode(WIFI_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            wifiConnected = false;
            unsigned long startAttemptTime = millis();

            while ((WiFi.status() != WL_CONNECTED) && (millis() - startAttemptTime < WIFI_TIMEOUT_MS)) {
                Serial.println("Trying to connect...");
                blink(WIFI_LED);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }

            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("WiFi Connected: " + WiFi.localIP().toString());
                wifiConnected = true;
                digitalWrite(WIFI_LED, HIGH);
                connectAWS();
            } else {
                Serial.println("Failed to connect to WiFi");
                digitalWrite(WIFI_LED, LOW);
            }
        }
    }
}

//Functional Description: Continuously gets data from ICM20948 and buffers it accordingly.
void SparkfunReadings(void * Parameters) {
    while (1) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            float icmData[9] = {
                myICM.accX(), myICM.accY(), myICM.accZ(),
                myICM.gyrX(), myICM.gyrY(), myICM.gyrZ(),
                myICM.magX(), myICM.magY(), myICM.magZ()
            };

            xSemaphoreTake(bufferMutex, portMAX_DELAY);

            if (wifiConnected) {
                if (buffer1Index < BUFFER_SIZE - 9) {
                    for (int i = 0; i < 9; i++) {
                        buffer1[buffer1Index++] = icmData[i];
                    }
                } else {
                    flushBufferToAWS();
                    for (int i = 0; i < 9; i++) {
                        buffer1[buffer1Index++] = icmData[i];
                    }
                }
            } else {
                if (sramBufferIndex < BUFFER_SIZE - 9) {
                    for (int i = 0; i < 9; i++) {
                        sramBuffer[sramBufferIndex++] = icmData[i];
                    }
                } else if(sramBufferIndex == BUFFER_SIZE){
                    // SRAM buffer is full
                        Serial.print(sramBufferIndex);
                        Serial.println("Buffer Full. Data will not be backed up unless WiFi is restored!");
                }
            }
            xSemaphoreGive(bufferMutex);
        }

        else{
          Sparkfun_9DOF_Initialization();
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup() {

    //Initialize Serial Communication and set baud rate
    Serial.begin(115200);

    //Initialize WiFi Setup
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    //Initialize I2C Setup
    Wire.begin();

    //Initialize SPI Setup
    SPI.begin();
    
    //To set Pin Mode
    pinMode(ICM_LED, OUTPUT);
    pinMode(WIFI_LED, OUTPUT);
    pinMode(SRAM_CS, OUTPUT);
    digitalWrite(SRAM_CS, HIGH);

    //To create Buffer Mutex for Buffer Handling (Synchronization)
    bufferMutex = xSemaphoreCreateMutex();
    if (bufferMutex == NULL) {
        Serial.println("Buffer mutex creation failed");
        while (1);
    }

    //Initialization call for Sparkfun 9DOF ICM20948
    Sparkfun_9DOF_Initialization();

    //FreeRTOS Tasks
    xTaskCreatePinnedToCore(SparkfunReadings, "SparkfunReadings", 10000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(KeepWifiAlive, "KeepWifiAlive", 10000, NULL, 1, NULL, 1);
}

void loop() {}
