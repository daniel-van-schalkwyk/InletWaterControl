#ifndef declarations_h
#define declarations_h
// Import all necessary libraries
#include <DallasTemperature.h>
#include <OneWire.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <wifiDetails.h>
// Define all necessary pins used by Arduino
#define geyserPowerSetPin           10
#define geyserPowerResetPin         11
#define freezerSetPin               12
#define freezerResetPin             13
#define tempBusPin                  2
#define tempCalibratorPin           6
#define geyserValveFeedbackPin      5
#define mainWaterValveFeedbackPin   4
#define preInletValveFeedbackPin    3
#define geyserWaterTempPin          A1
#define servoPosFeedbackPin         A6
// Define parameters for interrupt timer
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
// Define enumerations
enum systemStates{Off, On};
enum valveState{Closed, Open};
enum valveNumber{geyserValve, mainSupplyValve, PreInletValve, mixingValve};
enum PID_out_type{angle_, PWM};
enum highPowerLoad{geyser, freezer};
// Define parameters used by OLED screen
#define SCREEN_WIDTH            128     // OLED display width, in pixels
#define SCREEN_HEIGHT           64      // OLED display height, in pixels
#define OLED_RESET              -1      // Reset pin # (or -1 if sharing Arduino reset pin)
// Declare all instances 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_PWMServoDriver servoPWM = Adafruit_PWMServoDriver(0x40); // PWM servo motor driver object for all servo motors
OneWire SystemTemp(tempBusPin);
OneWire CalibratorTemp(tempCalibratorPin);
DallasTemperature calTempBus(&CalibratorTemp);
DallasTemperature systemTempBus(&SystemTemp);
// Define all general variables
bool geyserLatchFlag = false;
bool freezerLatchFlag = false;
bool regulationFlag = true;
bool timerSampleFlag = false;
bool firstTempRequest = true;
int tempResolution = 12;    // This is necessary to have a 5 Hz update frequency 
const int servoFREQUENCY = 50;
const unsigned long serialSpeed = 9600;
double inletTempMeas = 0.00;
double inletTempCal = 0.00;
double inletSetTemp = 20;
double tempAccuracyMargin = 0.5;
const DeviceAddress inletWaterSensorAddress = { 0x28, 0xE9, 0x12, 0x75, 0xD0, 0x01, 0x3C, 0xB7 };
// Define the absolute valve boundaries of the servo motors
const int MIN_GV_servo = 160;   // 0 deg
const int MAX_GV_servo = 300;   // 90 deg
const double feedback90 = 1.52; // voltage feedback from feedbak pin for 90 degrees
const double feedback0  = 0.58; // voltage feedback from feedbak pin for 0 degrees
int16_t ServoPwmTick = MIN_GV_servo;
const uint8_t servoChannel = 0;
int servoUpdateFrequency = 5; // Frequency of servo control system in Hz
bool dirFlag = true;
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
void controlMainGeyserInletTemp(double inletSetTemp = 25);
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
#endif