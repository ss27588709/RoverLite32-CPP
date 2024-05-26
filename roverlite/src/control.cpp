#include <Arduino.h>
#include <controller_index.h>
#include <Wifi.h>
#include <ESPAsyncWebServer.h>
#include <motor.h>
#include <MecanumWheelRobot.h>
#include <iostream>
#include <sstream>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Adafruit_PWMServoDriver.h>
#include "PinDefines.h"

void setupLedFlash(int pin);

const char *ssid = "TP-LINK_000000_2.4";
const char *password = "19640215";

AsyncWebServer server(80);
AsyncWebSocket wsRoverCmd("/cmd");
uint32_t streamClientId = 0;
uint8_t lampState = 0;
uint8_t lastPercentage = 100;
uint8_t batPercentage = 100;
const TickType_t xDelay1ms = pdMS_TO_TICKS(1);

// // OV2640 Camera
// Camera ov2640;

// Motors
Motor *motorFL = new Motor(-1);
Motor *motorFR = new Motor(1);
Motor *motorBL = new Motor(1);
Motor *motorBR = new Motor(-1);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Robot
MecanumWheelRobot roverLite(0.0465, 0.02626, 0.0295);

// Function Declaration
void handleRoot(AsyncWebServerRequest *request);
void handleNotFound(AsyncWebServerRequest *request);
void notifyClients();
void onRoverCmdWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void batteryMonitorLoop();
void sendCameraImage();
void streamTask(void *parameter);


void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();
    // Serial.printf("maxTurnOmega: %d vel_Max: %d\n", roverLite.maxTurnOmega, roverLite.vel_Max);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    pinMode(BJT_LAMP, OUTPUT);
    digitalWrite(BJT_LAMP, LOW);

    pinMode(RUN_PIN_1, OUTPUT);
    pinMode(RUN_PIN_2, OUTPUT);
    pwm.begin();
    pwm.setPWMFreq(50); // Analog servos run at ~60 Hz updates

    // forward, 1 to self.AIN2, self.BIN1, 1,self.CIN1,1,self.DIN2,1
    // forward value is 1, -1,-1, 1.
    //  021,534,687,11-17-16

    // go, 1,3,8,16, 1 -1 -1 1
    // left -1 1 -1 1 -> 0 0 1 1
    // right -1 1 -1 1 -> 0 0 1 1
    // left 0 1 0 1 -> 2,3,7,16 -> -1 -1 -1 1
    motorFL->initialize(pwm, 0, 2, 1);
    motorFR->initialize(pwm, 5, 3, 4); // 78冲突
    motorBL->initialize(pwm, 6, 8, 7);
    motorBR->initialize(pwm, 11, RUN_PIN_2, RUN_PIN_1); // should be 11, gpio 25, 24, however 24 -> esp32 16, 25->17,
    roverLite.attachMotor(motorFL, motorFR, motorBL, motorBR);
    roverLite.standBy();
    // camera_init();
    // startCameraServer();

    // WiFi.softAP(ssid, password);

    // ov2640.startCameraServer();
    

    server.on("/", HTTP_GET, handleRoot);
    server.onNotFound(handleNotFound);

    // wsCameraStream.onEvent(onStreamWebSocketEvent);
    // server.addHandler(&wsCameraStream);

    wsRoverCmd.onEvent(onRoverCmdWebSocketEvent);
    server.addHandler(&wsRoverCmd);
    server.begin();
    Serial.println("HTTP server started");

    // xTaskCreatePinnedToCore(streamTask, "streamTask", 10000, NULL, 1, &StreamTaskHandle, 1);
    // Serial.println("Streaming task begin!");
}

void loop()
{
    // wsCameraStream.cleanupClients();
    // wsRoverCmd.cleanupClients();
    // sendCameraImage();
}
void handleRoot(AsyncWebServerRequest *request)
{
//       AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", controller_index_html_gz, controller_index_html_gz_len);
//   response->addHeader("Content-Encoding", "gzip");
//   request->send(response);
  request->send_P(200, "text/html", indexHtml); // 响应请求
}

void handleNotFound(AsyncWebServerRequest *request)
{
    request->send_P(404, "text/plain", "File Not Found");
}

void notifyClients()
{
    const uint8_t size = JSON_OBJECT_SIZE(1);
    StaticJsonDocument<size> json;
    json["percentage"] = batPercentage;
    char buffer[17];
    size_t len = serializeJson(json, buffer);
    wsRoverCmd.textAll(buffer, len);
}

void onRoverCmdWebSocketEvent(AsyncWebSocket *server,
                              AsyncWebSocketClient *client,
                              AwsEventType type,
                              void *arg,
                              uint8_t *data,
                              size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;
    case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        roverLite.standBy();
        break;
    case WS_EVT_DATA:
        AwsFrameInfo *info;
        info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
        {
            /* std::string cmdData = "";
            cmdData.assign((char*)data, len);
            std::istringstream ss(cmdData);
            std::string L_XPos, L_YPos, R_XPos, R_YPos, Lamp_SW;
            Serial.printf("LXPos [%s] LYPos [%s] RXPos [%s] R_YPos [%s] Lamp [%s]\n",
                          L_XPos.c_str(), L_YPos.c_str(), R_XPos.c_str(), R_YPos.c_str(), Lamp_SW.c_str());

            int L_X = atoi(L_XPos.c_str());
            int L_Y = atoi(L_YPos.c_str());
            int R_X = atoi(R_XPos.c_str());
            int R_Y = atoi(R_YPos.c_str()); */

            StaticJsonDocument<128> json;
            DeserializationError err = deserializeJson(json, data);
            if (err)
            {
                Serial.print(F("deserializeJson() failed with code "));
                Serial.println(err.c_str());
                return;
            }

            if (json.containsKey("Lamp_SW"))
            {
                int8_t Lamp_SW = json["Lamp_SW"];

                if (lampState != Lamp_SW)
                {
                    lampState = Lamp_SW;
                    digitalWrite(BJT_LAMP, lampState);
                    Serial.printf("receive Lamp_SW: %d", Lamp_SW);
                }
            }
            else
            {
                int8_t L_X = json["L_XPos"];
                int8_t L_Y = json["L_YPos"];
                int8_t R_X = json["R_XPos"];
                R_X = -1 * R_X;
                L_X = 1 * L_X;
                // Serial.printf("receive L_X L_Y R_X: %d, %d, %d", L_X, L_Y, R_X);
                // Serial.println();

                roverLite.parseCommand(L_X, L_Y, R_X);
                roverLite.move();
            }

            if (abs(lastPercentage - batPercentage) > 3)
            {
                lastPercentage = batPercentage;
                notifyClients();
            }
        }
        break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
        break;
    default:
        break;
    }
}


void batteryMonitorLoop()
{
    int batValue = 0;
    float adcVoltage = 0;
    float batVoltage = 0;

    batValue = analogRead(BAT_ADC);
    // Serial.println(batValue);
    adcVoltage = batValue / 4096.0 * 2.6;
    // Serial.println(adcVoltage);
    batVoltage = adcVoltage * 3;
    Serial.printf("Battery Voltage: [%f]", batVoltage);

    batPercentage = batVoltage * 100 / 8.4;

    delay(10000);
}