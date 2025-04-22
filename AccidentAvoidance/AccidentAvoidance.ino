// Libraries for WiFi, AWS IoT, ICM20948, and GPS
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include "ICM_20948.h"

// WiFi and AWS Credentials
#include "secrets.h" // File that includes WiFi and AWS credentials

// Pin Definitions
#define SRAM_SIZE 65536  // 64KB SRAM (adjust if different for your SRAM model)

#define SRAM_CS 5
#define ICM_LED 25
#define WIFI_LED 33
#define BUZZER_PIN 13
#define GPS_RX 16
#define GPS_TX 17
#define WIFI_TIMEOUT_MS 20000
#define AD0_VAL 1

// Struct for storing sensor and GPS data
struct SensorData {
    float accX, accY, accZ;
    float gyrX, gyrY, gyrZ;
    float magX, magY, magZ;
    float latitude, longitude;
    float speed;
};

// Constants
#define BUFFER_SIZE 512
#define EARTH_RADIUS 6371.0 // Radius of Earth in km
bool usingBuffer1 = true;

WiFiServer server(80); // Create a server on port 80

// Buffers
SensorData buffer1[BUFFER_SIZE];
int buffer1Index = 0;
bool wifiConnected = false;
SemaphoreHandle_t bufferMutex;

// GPS, ICM20948, and AWS client setup
TinyGPSPlus gps;
ICM_20948_I2C myICM;
WiFiClientSecure net;
PubSubClient client(net);
HardwareSerial gpsSerial(1);

// Function to blink status LEDs
void blink(uint8_t pin) {
    digitalWrite(pin, HIGH);
    delay(250);
    digitalWrite(pin, LOW);
    delay(250);
}   

// Function to initialize the buzzer pin
void initializeBuzzer() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer initially
}

//Function to initialize SRAM 23LC512
void initializeSRAM(){
    pinMode(SRAM_CS, OUTPUT);
    digitalWrite(SRAM_CS, HIGH);
}

//Function to initialize status LEDs
void initializeStatusLeds(){
    pinMode(ICM_LED, OUTPUT);
    digitalWrite(ICM_LED,0);
    pinMode(WIFI_LED, OUTPUT);
    digitalWrite(WIFI_LED,0);
}

// AWS IoT Callback to handle messages
void callback(char* topic, byte* payload, unsigned int length) {
    String incomingMessage;
    for (unsigned int i = 0; i < length; i++) {
        incomingMessage += (char)payload[i];
    }
    
    Serial.print("Received message: ");
    Serial.println(incomingMessage);

    // Check for overspeed or rash driving alerts
    if (incomingMessage == "OVERSPEED" || incomingMessage == "RASH_DRIVING") {
        digitalWrite(BUZZER_PIN, HIGH); // Activate buzzer
        delay(500);
        digitalWrite(BUZZER_PIN, LOW);  // Deactivate buzzer
    }
}

// Function Description: Initialises Sparkfun 9DOF ICM20948
void initializeICM20948() {
    bool initialized = false;
    while (!initialized) {
        myICM.begin(Wire, AD0_VAL);
        if (myICM.status != ICM_20948_Stat_Ok) {
            Serial.println("Sensor initialization failed. Retrying...");
            blink(ICM_LED);
            delay(500);
        } else {
            //Sets the ICM_LED HIGH which indicates that the sensor is initialized and working
            initialized = true;
            digitalWrite(ICM_LED, HIGH);
            Serial.println("Sensor initialization successful");
        }
    }
}

// Connect to AWS IoT Core
void connectAWS() {
    if (WiFi.status() != WL_CONNECTED) 
        return;
        
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(callback); //AWS callback to handle incoming messages

    while (!client.connected()) {
        Serial.print("Connecting to AWS IoT...");

        if (client.connect(AWS_IOT_CLIENT_ID)) {
            Serial.println("Connected to AWS IoT");
            client.subscribe("alert"); // Subscribe to the alert topic
        }

        if (WiFi.status() == WL_CONNECTED) 
            break;
        else {
            Serial.print("Failed. Error state=");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

void publishToAWS(SensorData data) {
    String payload = "{";
    payload += "\"accX\":" + String(data.accX, 6) + ",";  // Added precision
    payload += "\"accY\":" + String(data.accY, 6) + ",";  // Added precision
    payload += "\"accZ\":" + String(data.accZ, 6) + ",";  // Added precision
    payload += "\"gyrX\":" + String(data.gyrX, 6) + ",";  // Added precision
    payload += "\"gyrY\":" + String(data.gyrY, 6) + ",";  // Added precision
    payload += "\"gyrZ\":" + String(data.gyrZ, 6) + ",";  // Added precision
    payload += "\"magX\":" + String(data.magX, 6) + ",";  // Added precision
    payload += "\"magY\":" + String(data.magY, 6) + ",";  // Added precision
    payload += "\"magZ\":" + String(data.magZ, 6) + ",";  // Added precision
    payload += "\"latitude\":" + String(data.latitude, 6) + ",";  // Added precision
    payload += "\"longitude\":" + String(data.longitude, 6) + ",";  // Added precision
    payload += "\"speed\":" + String(data.speed, 6);  // Added precision and removed the trailing comma
    
    payload += "}";

    // Debugging the payload before publishing
    Serial.println(payload);

    if (client.publish(AWS_IOT_TOPIC, payload.c_str())) {
        Serial.println("Data published to AWS IoT");
    } else {
        Serial.println("Failed to publish data");
    }
}


// Speed Calculation using GPS coordinates
float calculateSpeed(float lat1, float lon1, float lat2, float lon2, unsigned long deltaTime) {
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    
    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float distance = EARTH_RADIUS * c; // Distance in kilometers
    float speed = (distance * 1000) / (deltaTime / 1000.0); // Speed in m/s
    return speed;
}

// Read GPS data and calculate speed
void readGPSData(SensorData &data) {
    static float lastLat = 0.0, lastLon = 0.0;
    static unsigned long lastTime = millis();

    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated()) {
            data.latitude = gps.location.lat();
            data.longitude = gps.location.lng();

            unsigned long currentTime = millis();
            if (lastLat != 0.0 && lastLon != 0.0) {
                data.speed = calculateSpeed(lastLat, lastLon, data.latitude, data.longitude, currentTime - lastTime);
            }
            lastLat = data.latitude;
            lastLon = data.longitude;
            lastTime = currentTime;
        }
    }
}

// Flush buffer to AWS IoT
void flushBufferToAWS() {
    for (int i = 0; i < buffer1Index; i++) {
        publishToAWS(buffer1[i]);
        delay(100); // Avoid network flooding
    }
    buffer1Index = 0;
}

// WiFi Management Task
void KeepWifiAlive(void *Parameters) {
    int wifiRetryCount = 0; // Counter for WiFi connection attempts

    while (1) {
        // If WiFi is connected, process MQTT and manage buffers
        if (WiFi.status() == WL_CONNECTED) {
            wifiRetryCount = 0; // Reset retry counter if connected

            if (!wifiConnected) {
                Serial.println("WiFi Connected: " + WiFi.localIP().toString());
                wifiConnected = true;
                digitalWrite(WIFI_LED, HIGH); // Indicate WiFi connection

                // Flush data from SRAM to AWS when WiFi reconnects
                Serial.println("Flushing data from SRAM to AWS...");
                flushSRAMToAWS(); // Ensure data in SRAM is sent to the cloud

                // Synchronize buffers and flush remaining data to AWS
                xSemaphoreTake(bufferMutex, portMAX_DELAY);
                if (!usingBuffer1) {
                    flushBufferToAWS(); // Flush remaining buffer1 data to AWS
                }
                xSemaphoreGive(bufferMutex);
            }

            // Keep the MQTT client active
            client.loop(); // Ensure MQTT client processes messages
            vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay for 10 seconds

        } else {  // If WiFi is not connected, attempt to reconnect
            Serial.println("Attempting to connect to WiFi...");
            blink(WIFI_LED); // Blink LED to indicate connection attempt
            WiFi.mode(WIFI_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Start WiFi connection
            wifiConnected = false;
            unsigned long startAttemptTime = millis();

            // Retry connecting to WiFi within timeout period
            while ((WiFi.status() != WL_CONNECTED) && (millis() - startAttemptTime < WIFI_TIMEOUT_MS)) {
                Serial.println("Trying to connect...");
                blink(WIFI_LED);
                vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for 2 seconds before retrying
            }

            // If WiFi is connected, proceed with connection and AWS IoT
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("WiFi Connected: " + WiFi.localIP().toString());
                wifiConnected = true;
                digitalWrite(WIFI_LED, HIGH); // Indicate WiFi connected
                connectAWS(); // Reconnect to AWS IoT
            } else {  // If WiFi connection fails after retries, handle SRAM buffering
                wifiRetryCount++; // Increment retry counter
                Serial.println("Failed to connect to WiFi");
                digitalWrite(WIFI_LED, LOW); // Indicate WiFi disconnected

                // If max retries are reached, switch to SRAM buffering mode
                if (wifiRetryCount >= 3) {
                    Serial.println("Max WiFi retries reached. Switching to SRAM buffering.");
                    xSemaphoreTake(bufferMutex, portMAX_DELAY);
                    usingBuffer1 = false; // Indicate that data will be stored in SRAM
                    xSemaphoreGive(bufferMutex);
                    break; // Exit WiFi retry loop, indicating to switch to SRAM mode
                }
            }
        }
    }
}



// Read ICM20948 and GPS data
void readSensorData(void *param) {
    Serial.println("Sensor reading task started...");
    
    while (true) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            SensorData data = {
                myICM.accX(), myICM.accY(), myICM.accZ(),
                myICM.gyrX(), myICM.gyrY(), myICM.gyrZ(),
                myICM.magX(), myICM.magY(), myICM.magZ(),
                0.0, 0.0, 0.0 // Initialize GPS values
            };

            readGPSData(data); // Read GPS data and calculate speed

            xSemaphoreTake(bufferMutex, portMAX_DELAY);
            if (wifiConnected) {
                if (buffer1Index < BUFFER_SIZE) {
                    buffer1[buffer1Index++] = data;
                } else {
                    flushBufferToAWS();
                    buffer1[buffer1Index++] = data;
                }
            } else {
                if (storeDataToSRAM(data)) {
                    Serial.println("No WiFi. Data stored in SRAM.");
                } else {
                    Serial.println("SRAM is full. Data might be lost.");
                }
            }
            xSemaphoreGive(bufferMutex);
        } else {
            Serial.println("ICM20948 not ready. Reinitializing...");
            initializeICM20948();
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}


void flushSRAMToAWS() {
    Serial.println("Checking SRAM for unsent data...");
    uint8_t data[sizeof(SensorData)];
    int sramIndex = 0;
    bool dataFlushed = false; // Track if any data was flushed

    while (true) {
        // Activate SRAM and issue read command
        digitalWrite(SRAM_CS, LOW);
        SPI.transfer(0x03); // Read command
        SPI.transfer((sramIndex >> 8) & 0xFF); // Address high byte
        SPI.transfer(sramIndex & 0xFF);        // Address low byte
        
        for (int i = 0; i < sizeof(SensorData); i++) {
            data[i] = SPI.transfer(0x00);
        }
        digitalWrite(SRAM_CS, HIGH);

        // Check if this is valid data
        SensorData* sramData = reinterpret_cast<SensorData*>(data);
        if (sramData->latitude == 0 && sramData->longitude == 0) {
            if (!dataFlushed) {
                Serial.println("No valid data found in SRAM.");
            }
            break; // Exit if no more valid data is found
        }

        // Publish the data to AWS
        publishToAWS(*sramData);
        dataFlushed = true;
        Serial.print("Flushed data from SRAM index: ");
        Serial.println(sramIndex);

        // Mark this block as sent by zeroing it out
        digitalWrite(SRAM_CS, LOW);
        SPI.transfer(0x02); // Write command
        SPI.transfer((sramIndex >> 8) & 0xFF);
        SPI.transfer(sramIndex & 0xFF);
        for (int i = 0; i < sizeof(SensorData); i++) {
            SPI.transfer(0x00);
        }
        digitalWrite(SRAM_CS, HIGH);

        // Move to the next block in SRAM
        sramIndex += sizeof(SensorData);
    }

    if (dataFlushed) {
        Serial.println("All data flushed from SRAM to cloud.");
    }
}

bool storeDataToSRAM(SensorData data) {
    static int sramWriteIndex = 0; // Tracks where to write next in SRAM
    const int maxSRAMIndex = SRAM_SIZE - sizeof(SensorData); // Maximum index in SRAM

    // Convert the SensorData struct to a byte array
    uint8_t* dataBytes = reinterpret_cast<uint8_t*>(&data);

    // Check if there's enough space in SRAM to store new data
    if (sramWriteIndex >= maxSRAMIndex) {
        // If SRAM is full, overwrite the first data (circular buffer)
        sramWriteIndex = 0;
    }

    // Retry mechanism to ensure successful write
    int retryCount = 3;
    while (retryCount-- > 0) {
        // Select SRAM chip to start communication
        digitalWrite(SRAM_CS, LOW);

        // Send write command and address to SRAM
        SPI.transfer(0x02); // Write command
        SPI.transfer((sramWriteIndex >> 8) & 0xFF); // Address high byte
        SPI.transfer(sramWriteIndex & 0xFF);        // Address low byte

        // Send the actual data bytes to SRAM
        for (int i = 0; i < sizeof(SensorData); i++) {
            if (SPI.transfer(dataBytes[i]) == 0) {
                // Retry if transfer failed
                continue;
            }
        }

        // Deselect SRAM chip to end communication
        digitalWrite(SRAM_CS, HIGH);

        // Increment SRAM write index for the next write operation
        sramWriteIndex += sizeof(SensorData);

        return true; // Successfully stored the data in SRAM
    }

    // If retries are exhausted, print error and return false
    Serial.println("Error: SRAM write failed after retries.");
    return false;
}


void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); //Start WiFi
    Wire.begin(); 
    SPI.begin(); //Start SPI
    server.begin(); // Start the server

    initializeStatusLeds(); //Initialize Status LEDs
    initializeSRAM(); //Initialize SRAM
    initializeBuzzer(); // Initialize the buzzer
    initializeICM20948(); //Initialize ICM-20948

    //Mutex creation for SRAM Buffer writing
    bufferMutex = xSemaphoreCreateMutex();
    if (!bufferMutex) {
        Serial.println("Mutex creation failed");
        while (1);
    }

    //FreeRTOS Tasks
    xTaskCreatePinnedToCore(KeepWifiAlive, "keepWifiAlive", 60000,NULL,1,NULL,1);
    xTaskCreatePinnedToCore(readSensorData, "readSensorData", 40000, NULL, 2 , NULL, 0);
}

void loop(){}
