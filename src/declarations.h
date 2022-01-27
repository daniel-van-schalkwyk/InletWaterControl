#ifndef declarations_h
#define declarations_h
// Import all necessary libraries
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <U8g2lib.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <wifiDetails.h>
// Define all necessary pins and ports used by Arduino
#define mainControllerChannel       Serial1
#define geyserPowerSetPin           7
#define geyserPowerResetPin         8
#define freezerSetPin               12
#define freezerResetPin             13
#define tempBusPin                  2
#define tempCalibratorPin           6
#define geyserValveFeedbackPin      5
#define mainWaterValveFeedbackPin   4
#define preInletValveFeedbackPin    3
#define geyserWaterTempPin          A1
#define servoPosFeedbackPin         A6
#define encoderDtPin                10
#define encoderClkPin               11
#define encoderSwPin                9
// Define parameters for interrupt timer
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
// Define enumerations
enum actuatorStates{Off, On};
enum valveState{Closed, Open};
enum valveNumber{geyserValve, mainSupplyValve, PreInletValve, mixingValve};
enum PID_out_type{angle_, PWM};
enum highPowerLoad{geyser, freezer};
enum encoderDirection : uint8_t {CCW, CW, still};
enum systemStates : int {idle, cooling, heating, discharging, tempSelect};
// Define parameters used by OLED screen
#define SCREEN_WIDTH            128     // OLED display width, in pixels
#define SCREEN_HEIGHT           64      // OLED display height, in pixels
#define OLED_RESET              -1      // Reset pin # (or -1 if sharing Arduino reset pin)
// Declare all instances 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
U8G2_SH1106_128X32_VISIONOX_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Adafruit_PWMServoDriver servoPWM = Adafruit_PWMServoDriver(0x40); // PWM servo motor driver object for all servo motors
OneWire SystemTemp(tempBusPin);
OneWire CalibratorTemp(tempCalibratorPin);
DallasTemperature calTempBus(&CalibratorTemp);
DallasTemperature systemTempBus(&SystemTemp);
const DeviceAddress mainInletWaterSensorAddress = {0x28, 0xA7, 0xB3, 0x79, 0xA2, 0x00, 0x03, 0xA6};
const DeviceAddress localOutletSensorAddress = {0x28, 0xE9, 0x12, 0x75, 0xD0, 0x01, 0x3C, 0xB7};
const DeviceAddress freezerTempSensorAddress = {};
// Define all general variables
int systemState = idle;
bool geyserLatchFlag = false;
bool freezerLatchFlag = false;
bool regulationFlag = false;
bool timerSampleFlag = false;
bool firstTempRequest = true;
volatile bool encoderClkFlag = false;
volatile bool encoderSwFlag = false;
volatile bool encoderDtFlag = false; 
volatile int geyserTempUpdateCounter = 0;
volatile int freezerTempUpdateCounter = 0;
const double geyserThermistorDisconnected = -1;
const double geyserWaterDeadband = 1.5;
const double inletTempSatisfactionMargin = 0.5;
bool inDeadBand = false;
double tempAccuracyMargin = 0.5;
int tempResolution = 10;    // This is necessary to have a 5 Hz update frequency 
const int servoFREQUENCY = 50;
const int disconnectDS18B20 = DEVICE_DISCONNECTED_C;
const unsigned long serialSpeed = 115200;
double inletTempMeas = 0.00;
double inletTempCal = 0.00;
double inletSetTemp = 20;
double geyserSetTemp = 40;  // This may change a abit according to the required outlet target
// Define local temperature measurement variables
double localOutletTemp = 0.00;
double freezerChamberTemp = 0.00;
double geyserWaterTemp = 0.00;
double sourceWaterTemp = 0.00;
// Define the absolute valve boundaries of the servo motors
const int MIN_GV_servo = 160;   // 0 deg
const int MAX_GV_servo = 300;   // 90 deg
const double feedback90 = 1.52; // voltage feedback from feedbak pin for 90 degrees
const double feedback0  = 0.58; // voltage feedback from feedbak pin for 0 degrees
int16_t ServoPwmTick = MIN_GV_servo;
const uint8_t servoChannel = 0;
int servoUpdateFrequency = 5; // Frequency of servo control system in Hz
bool dirFlag = true;
volatile int encoderCounter = 0;
volatile unsigned long encoderClkTick = 0;
volatile unsigned long encoderSwTick = 0;
bool previousStateCLK;
// Declare Wifi stuff...
int status = WL_IDLE_STATUS;
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
String mqttReceivedMessage = "";
const char broker[] = "test.mosquitto.org";
int port = 1883;
const char topicReceive[] = "commandsUI";
const char topicSend[] = "testBenchResponse";

struct PID_params
{
  double Kp;
  double Ki;
  double Kd;
  double e_sum;
  double e_prev;
}servoPIDout;
// Define variables for thermistor circuit and code
struct paramsThermistorNTC
{
  double voltDividerR;
  double thermistorResistance;
  double a2;
  double b2;
  double c2;
};
double max12BitNum = 4095;
double kelvinToC_const = 273.15;
paramsThermistorNTC geyserWiseThermistorMain, geyserWiseThermistorPre;
// Declare all function prototypess
void actuatePower(bool elementState, bool type = geyser);
void configurePins();
void controlInletEnvironment(double MainInletSetTemp = 25);
void controlGeyserElement(double geyserWaterTemp, double geyserSetTemp);
void oneWireSetup();
void calibrateServos();
void actuateServo(int valveNum = mainSupplyValve, double servoAngle = 0);
double getServoAngle();
uint16_t getServoPulsePeriod(int servo, int angle);
void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();
void displaySetup();
uint8_t findOneWireDevices(int pin);
void getAllTemperatures();
double getGeyserThermistorTemp();
void setThermistorProperties(paramsThermistorNTC *thermistorStruct);
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
void servoMotorSetup();
double calcPIDoutput(double inletTempError, bool typeOut = angle_);
void readSerialMessage(String serialMessage);
String getSubString(String data, char separator, int index);
void connectToWifi();
void receivedMqttMessage(int messageSize);
void connectToMQTTbroker();
void configureInterrupts();
uint8_t checkEncoderDirection();
void encoderSwHandler();
void encoderDtHandler();
void encoderClkHandler();
void setTemperatureMenu();
void controlChestFreezerPower();
void controlServoValve();
void updateDisplay();
#endif