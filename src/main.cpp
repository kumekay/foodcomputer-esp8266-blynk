#include <Arduino.h>
// Configuration
#include <credentials.h>
// copy src/credentials_example.h to src/credentials.h and put your data to the file

#include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino
#include <BlynkSimpleEsp8266.h> // https://github.com/blynkkk/blynk-library
#include <ESP8266httpUpdate.h>  // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266httpUpdate
#include <Ticker.h>             // https://github.com/esp8266/Arduino/blob/master/libraries/Ticker

#include <Wire.h>
#include <SI7021.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#define DEBUG_SERIAL Serial

// Use I2C
#define I2C_SDA D7 // Yellow
#define I2C_SCL D6 // White

// Config
const uint16_t blynk_port{8442};
const char device_id[] = "foodcomputer_blynk_esp8266";
const char fw_ver[] = "0.0.1";

bool shouldSendData{false};
bool shouldCheckWatering{false};
Ticker dataSender;
Ticker wateringStateChecker;

// Power output
const uint8_t configPins[]{D3, D4};
uint8_t out1Pin{D1};
uint8_t out2Pin{D2};
uint8_t lightingPin{D0};

// Humidity and temperature
SI7021 si7021;

// Water temperature
OneWire ow(D5);
DallasTemperature ds(&ow);
DeviceAddress waterTemperatureAddr;

void setSendDataFlag()
{
        shouldSendData = true;
}

void sendData()
{
        float airHumdity{0};
        float airTemperature{0};
        float waterTemperature{0};

        ds.requestTemperatures();
        waterTemperature = ds.getTempC(waterTemperatureAddr);

        airHumdity = si7021.getHumidityPercent();
        airTemperature = si7021.getCelsiusHundredths() / 100.0;

        Blynk.virtualWrite(V0, airTemperature);
        Blynk.virtualWrite(V1, airHumdity);
        Blynk.virtualWrite(V2, waterTemperature);

        DEBUG_SERIAL.print("Air Humidity: ");
        DEBUG_SERIAL.println(airHumdity);

        DEBUG_SERIAL.print("Air Temperature: ");
        DEBUG_SERIAL.println(airTemperature);

        DEBUG_SERIAL.print("Water Temperature: ");
        DEBUG_SERIAL.println(waterTemperature);
        shouldSendData = false;
}

void scanI2C()
{
        byte error, address;
        int nDevices;

        DEBUG_SERIAL.println("Scanning...");

        nDevices = 0;
        for (address = 1; address < 127; address++)
        {
                // The i2c_scanner uses the return value of
                // the Write.endTransmisstion to see if
                // a device did acknowledge to the address.
                Wire.beginTransmission(address);
                error = Wire.endTransmission();

                if (error == 0)
                {
                        DEBUG_SERIAL.print("I2C device found at address 0x");
                        if (address < 16)
                                DEBUG_SERIAL.print("0");
                        DEBUG_SERIAL.print(address, HEX);
                        DEBUG_SERIAL.println("  !");

                        nDevices++;
                }
                else if (error == 4)
                {
                        DEBUG_SERIAL.print("Unknown error at address 0x");
                        if (address < 16)
                                DEBUG_SERIAL.print("0");
                        DEBUG_SERIAL.println(address, HEX);
                }
        }
        if (nDevices == 0)
                DEBUG_SERIAL.println("No I2C devices found\n");
        else
                DEBUG_SERIAL.println("done\n");
}

void setup()
{
        DEBUG_SERIAL.begin(115200);

        Wire.begin(I2C_SDA, I2C_SCL);
        scanI2C();

        // Connections
        DEBUG_SERIAL.println("Connecting to WiFI");
        Blynk.connectWiFi(WIFI_SSID, WIFI_PASS);

        DEBUG_SERIAL.println("\nConnecting to Blynk server");
        Blynk.config(BLYNK_AUTH, BLYNK_SERVER, blynk_port);
        while (Blynk.connect() == false)
        {
                delay(500);
                DEBUG_SERIAL.print(".");
        }

        DEBUG_SERIAL.println("\nReady");

        // Devices
        for (uint8 i = 0; i <= 1; i++)
        {
                pinMode(configPins[i], OUTPUT);
                digitalWrite(configPins[i], HIGH);
        }
        pinMode(out1Pin, OUTPUT);
        digitalWrite(out1Pin, LOW);
        pinMode(out2Pin, OUTPUT);
        digitalWrite(out2Pin, LOW);
        pinMode(lightingPin, OUTPUT);
        digitalWrite(lightingPin, HIGH);

        // Init sensors
        if (!ds.getAddress(waterTemperatureAddr, 0))
        {
                Serial.println("DS18B20 not found");
        }

        if (!si7021.begin(I2C_SDA, I2C_SCL))
        {
                DEBUG_SERIAL.println("SI7021 not found");
        }

        // Timers;
        dataSender.attach(5.0, setSendDataFlag);
}

void loop()
{
        Blynk.run();
        if (shouldSendData)
        {
                sendData();
        }
}

// Sync state on reconnect
BLYNK_CONNECTED()
{
        Blynk.syncAll();
}

// Update FW
BLYNK_WRITE(V22)
{
        if (param.asInt() == 1)
        {
                DEBUG_SERIAL.println("FW update request");

                char full_version[34]{""};
                strcat(full_version, device_id);
                strcat(full_version, "::");
                strcat(full_version, fw_ver);

                t_httpUpdate_return ret = ESPhttpUpdate.update(FW_UPDATE_URL, full_version);
                switch (ret)
                {
                case HTTP_UPDATE_FAILED:
                        DEBUG_SERIAL.println("[update] Update failed.");
                        break;
                case HTTP_UPDATE_NO_UPDATES:
                        DEBUG_SERIAL.println("[update] Update no Update.");
                        break;
                case HTTP_UPDATE_OK:
                        DEBUG_SERIAL.println("[update] Update ok.");
                        break;
                }
        }
}

BLYNK_WRITE(V10)
{
        analogWrite(lightingPin, param.asInt());
}