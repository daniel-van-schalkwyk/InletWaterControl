#include <declarations.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(serialSpeed);
  // configurePins();
  // oneWireSetup();
  // servoMotorSetup();
  // startTimer(servoUpdateFrequency);
  displaySetup();
  // // connectToWifi();
  // previousStateCLK = digitalRead(encoderClkPin);
  // Serial.print("Setup Complete");

}

void loop() {
  findOneWireDevices(4);
  delay(2000);
  // put your main code here, to run repeatedly:
  // if(Serial.available() > 0)
  // {
  //   readSerialMessage(Serial.readStringUntil(0x10));
  // }
  // else
  // {
  //   // calibrateServos();
  //   // servoPWM.setPWM(0, 0, MIN_GV_servo);
  //   // delay(100);
  //   // Serial.println(getServoAngle());
  //   // controlMainGeyserInletTemp(inletSetTemp);

  //   if(encoderSwFlag && (millis() - encoderSwTick >= 50)) // Include debounce for switch
  //   {
  //     Serial.println("Switch triggered...");
  //     systemState = systemStates::tempSelect;
  //     setTemperatureMenu();
  //   }
  // }
}

void readSerialMessage(String serialMessage)
{
  regulationFlag = (bool)(getSubString(serialMessage, ':', 0).toInt());
  inletSetTemp = getSubString(serialMessage, ':', 1).toDouble();
  Serial.write("Regulation: ");
  Serial.write(regulationFlag);
  Serial.write(" and inlet set temp: ");
  Serial.write(inletSetTemp);
}

/** Function description
 * \brief This function is used separate a string variable using delimeters
 * \param data  The entire string to be separated
 * \param separator The character delimiter used to split the data string
 * \param index The location of the string to be extracted from the data string
 * \return The indexed string delimited by the separator paramter
 */
String getSubString(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
      if (data.charAt(i) == separator || i == maxIndex) {
          found++;
          strIndex[0] = strIndex[1] + 1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/*! Function description
  @brief  This function is used to toggle the state (open or close) of the geyser heating element.
  \param elementState determines whether the heating element switches on or off. If true, the element switches ON and vice versa.
*/
void actuatePower(bool loadState, bool loadType)
{
  switch(loadType)
  {
    case geyser:
    {
      if(loadState)
      {
        geyserLatchFlag = On;
        digitalWrite(geyserPowerResetPin, LOW);
        digitalWrite(geyserPowerSetPin, HIGH);
        delay(10);
        digitalWrite(geyserPowerSetPin, LOW);
      }
      else
      {
        geyserLatchFlag = Off;
        digitalWrite(geyserPowerResetPin, HIGH);
        digitalWrite(geyserPowerSetPin, LOW);
        delay(10);
        digitalWrite(geyserPowerResetPin, LOW);
      }
      break;
    }
    case freezer:
    {
      if(loadState)
      {
        freezerLatchFlag = On;
        digitalWrite(freezerResetPin, LOW);
        digitalWrite(freezerSetPin, HIGH);
        delay(10);
        digitalWrite(freezerSetPin, LOW);
      }
      else
      {
        freezerLatchFlag = Off;
        digitalWrite(freezerResetPin, HIGH);
        digitalWrite(freezerSetPin, LOW);
        delay(10);
        digitalWrite(freezerResetPin, LOW);
      }
      break;
    } 
  }
}

/*! Function description
  @brief  This function is used to configure all pins used in the overall system.
*/
void configurePins()
{
  // Configure all OUTPUT pins (Default is INPUT)
  pinMode(geyserPowerSetPin, OUTPUT);
  pinMode(geyserPowerResetPin, OUTPUT);
  pinMode(freezerSetPin, OUTPUT);
  pinMode(freezerResetPin, OUTPUT);
  pinMode(geyserValveFeedbackPin, INPUT);
  pinMode(mainWaterValveFeedbackPin, INPUT);
  pinMode(preInletValveFeedbackPin, INPUT);
  pinMode(servoPosFeedbackPin, INPUT);
  analogReadResolution(12); // Set analgue pin resolution to 12 bits
  configureInterrupts();
}

/*! Function description
  @brief  This function is used to configure the appropriate interrupt pins of the overall system.
*/
void configureInterrupts() 
{
  pinMode(encoderClkPin, INPUT);
  pinMode(encoderDtPin, INPUT);
  pinMode(encoderSwPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderSwPin), encoderSwHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderDtPin), encoderDtHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderClkPin), encoderClkHandler, CHANGE);
}

/*! Function description
  @brief  This function is used to configure all pins used in the overall system.
*/
void servoMotorSetup()
{
  servoPWM.begin();
  servoPWM.setPWMFreq(servoFREQUENCY);
}

/** Function description
 * \brief This function is used to regulate the inlet temperature of the 150L geyser water if this is required.
 */
void controlMainGeyserInletTemp(double inletSetTemp)
{
  double geyserWaterTemp = 20.00;
  if(regulationFlag && timerSampleFlag)
  {
    timerSampleFlag = false;
    servoPWM.setPWM(servoChannel, 0, ServoPwmTick);
    double currentServoAngle = getServoAngle();
    Serial.println(currentServoAngle);
    if(firstTempRequest)
    {
      systemTempBus.requestTemperatures();
      firstTempRequest = false;
    }
    else
    {
      inletTempCal = systemTempBus.getTempC(inletWaterSensorAddress);
      geyserWaterTemp = getGeyserThermistorTemp();
      systemTempBus.requestTemperatures();
      double inletTempError = inletSetTemp - inletTempCal;
      if((inletTempError > 0.00) && (abs(inletTempError) > tempAccuracyMargin))
      {
        // The measured inlet water temperature is below the setpoint margin
        // Water needs to increase in temperature
        actuateServo(geyserValve, calcPIDoutput(inletTempError, angle_));
      }
      else if((inletTempError < 0.00) && (abs(inletTempError) > tempAccuracyMargin))
      {
        // The measured inlet water temperature is above the setpoint margin
        // Water needs to decrease in temperature
        actuateServo(geyserValve, calcPIDoutput(inletTempError, angle_));
      }
      else
      {
        // The measured inlet temperature is within the temperature margin and is ready
      }
    }
    // print to LED screen
    // display.clearDisplay();
    // display.println("Temperature: " + String(inletTempCal));
    // display.setCursor(0, 10);
    // display.println("Geyser Temp: " + String(geyserWaterTemp));
    // display.display();
    u8g2.clearDisplay();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.firstPage();
    do 
    {
      u8g2.setCursor(0, 10);
      u8g2.print("Temperature = " + String(inletTempCal));
      u8g2.setCursor(0, 20);
      u8g2.println("Geyser Temp: " + String(geyserWaterTemp));
    } 
    while ( u8g2.nextPage() );
  }
}

double calcPIDoutput(double inletTempError, bool typeOut)
{
  servoPIDout.Kp = 8;
  servoPIDout.Ki = 0.5;
  servoPIDout.Kd = 0.2;
  double errorDiff = servoPIDout.e_prev - inletTempError;
  double PID_out = 0.00;
  if(!typeOut)
  {
    PID_out = servoPIDout.Kp*inletTempError + servoPIDout.Ki*servoPIDout.e_sum + servoPIDout.Kd*errorDiff;
    if(PID_out >= 90) PID_out = 90;
    else if(PID_out <= 90) PID_out = 0;
  }
  else
  {
    if(PID_out >= MAX_GV_servo) PID_out = MAX_GV_servo;
    else if(PID_out <= MIN_GV_servo) PID_out = MIN_GV_servo;
  }
  servoPIDout.e_sum += inletTempError;
  servoPIDout.e_prev = inletTempError;
  return PID_out;
}

/** Function description
 * \brief This is a setup function and is used to configure all the onewire sensors in the system
 *        It instantiates an array of DallasTemperature objects and configures all elements to the
 *        same specifications. 
 *        It sets the resolution of the DS18B20 sensors, sets the "wait for convesion" flag to false 
 *        and checks that all sensors are connected to their busses - otherwise it will display an error message.
 */
void oneWireSetup() 
{
  // Initialise temperature bus for inlet water control
  systemTempBus.begin();
  systemTempBus.setResolution(tempResolution);
  systemTempBus.setWaitForConversion(false);
  // Initialise calibration temperature bus for inlet water control
  calTempBus.begin();
  calTempBus.setResolution(tempResolution);
  calTempBus.setWaitForConversion(false);
}

void calibrateServos()
{
  if(ServoPwmTick >= MIN_GV_servo && ServoPwmTick < MAX_GV_servo && dirFlag)
  {
    dirFlag = true;
    ServoPwmTick++;
  }
  else
  {
    dirFlag = false;
    if(ServoPwmTick <= MIN_GV_servo)
      dirFlag = true;
    else  
      ServoPwmTick--;
  }
  delay(100);
  servoPWM.setPWM(0, 0, ServoPwmTick);
}

double getServoAngle()
{
  double adcInServo = analogRead(servoPosFeedbackPin);
  adcInServo *= 3.30/max12BitNum;
  return mapDouble(adcInServo, feedback0, feedback90, 0, 90);
}

void actuateServo(int valveNum, double servoAngle)
{
  servoPWM.setPWM(valveNum, 0, getServoPulsePeriod(valveNum, servoAngle));
}

/** Description
 * \brief This function is used to convert the required angle into a PWM pulse width
 *        that can be used by the servo shield library
 * \param servo The enumerated servo name / channel on the PWM shield.
 * \param angle The required anglar position the servo needs to actuate to.
 * \return The unint16_t 12-bit pulse value required by the PWM servo motor object.
 */
uint16_t getServoPulsePeriod(int servo, int angle) 
{
  uint16_t servo_max = 0; 
  uint16_t servo_min = 0;
  uint16_t pulsePeriod;
  // Cap the angle to prevent servo damage
  if(angle > 90 && servo != PreInletValve)  angle = 90; else if(angle < 0)  angle = 0;
  // Start the switch statement for various servos
  servo_max = MAX_GV_servo;
  servo_min = MIN_GV_servo;
  pulsePeriod = (uint16_t)mapDouble((double)angle, (double)0, (double)90, (double)servo_min, (double)servo_max);
  return pulsePeriod;
}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  // Serial.println(TC->COUNT.reg);
  // Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync
  TcCount16* TC = (TcCount16*) TC3;
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  setTimerFrequency(frequencyHz);
  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;
  NVIC_EnableIRQ(TC3_IRQn);
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) 
  {
    TC->INTFLAG.bit.MC0 = 1;
    // Write callback here!!!
    timerSampleFlag = true;
  }
}

/** Description
 * \brief This function is used to calibrate a sensor bus to an accurate temperature value
 *        A simple linear calibration technique is used to calibrate the sensors
 *        T_calibrated = m_slope * T_measured + intercept
 */
void calibrateSensorBus()
{
  // read 
}

/*! Function description
  @brief  
*/
void displaySetup()
{
  if(u8g2.begin())
  {
    u8g2.clearDisplay();
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.firstPage();
    do 
    {
      u8g2.setCursor(45, 10);
      u8g2.print("Welcome!");
      u8g2.setCursor(0, 20);
      u8g2.println("Inlet water controller :)");
    } 
    while ( u8g2.nextPage() );
  }
  
  // if(display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  // { 
  //  // Clear the buffer
  //   display.clearDisplay();
  //   delay(1);
  //   display.setTextSize(1);
  //   display.setTextColor(WHITE);
  //   display.setCursor(0, 0);
  //   // Display static text
  //   display.println("OLED screen ready!");
  //   display.setCursor(0, 10);
  //   display.println("Current State: IDLE");
  //   display.display();
  //   delay(1);
  // }
}

uint8_t findOneWireDevices(int pin)
{
  OneWire ow(pin);
  uint8_t address[8];
  uint8_t count = 0;
  if (ow.search(address))
  {
    Serial.print("\nuint8_t pin");
    Serial.print(pin, DEC);
    Serial.println("[][8] = {");
    do {
      count++;
      Serial.println("  {");
      for (uint8_t i = 0; i < 8; i++)
      {
        Serial.print("0x");
        if (address[i] < 0x10) Serial.print("0");
        Serial.print(address[i], HEX);
        if (i < 7) Serial.print(", ");
      }
      Serial.println("  },");
    } while (ow.search(address));

    Serial.println("};");
    Serial.print("// nr devices found: ");
    Serial.println(count);
  }
  else{
    Serial.println("Nothing found");
  }
  return count;
}

/*! Function description
  @brief Captures the thermistor temperature reading of the geyserWise thermistor
  \return The bulk water temperature of the geyser water measured by the geyserWise thermistor
*/
double getGeyserThermistorTemp()
{
  analogWriteResolution(12);
  setThermistorProperties(&geyserWiseThermistorMain);
  geyserWiseThermistorMain.thermistorResistance = geyserWiseThermistorMain.voltDividerR*((max12BitNum)/(double)analogRead(geyserWaterTempPin) - 1);
  double thermR = geyserWiseThermistorMain.thermistorResistance;
  double temp = (1.0/(geyserWiseThermistorMain.a2 + geyserWiseThermistorMain.b2*log(thermR) + geyserWiseThermistorMain.c2*(log(thermR))*(log(thermR))*(log(thermR))) - kelvinToC_const);
  return temp;
}

/*! Function description
  @brief Captures the thermistor temperature reading of the geyserWise thermistor
  \param  thermistorStruct The thermistor paramater structure to that you want to set
*/
void setThermistorProperties(paramsThermistorNTC *thermistorStruct)
{
  // Set the geyserWise thermistor properties
  thermistorStruct->voltDividerR = 9610.00;
  thermistorStruct->thermistorResistance = 12000.00;
  thermistorStruct->a2 = 0.0008544974;
  thermistorStruct->b2 = 0.0002882289;
  thermistorStruct->c2 = -1.8781118715e-7;
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*! Function description
  @brief 
*/
void connectToMQTTbroker()
{
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // set the message receive callback
  mqttClient.onMessage(receivedMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(topicReceive);
  Serial.println();
  // subscribe to a topic
  mqttClient.subscribe(topicReceive);
  Serial.print("Topic: ");
  Serial.println(topicReceive);
  Serial.println();
}

/*! Function description
  @brief 
*/
void connectToWifi()
{
  // attempt to connect to Wi-Fi network:
  while (status != WL_CONNECTED) 
  {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // wait 5 seconds for connection:
    delay(5000);
  }
  // you're connected now, so print out the data:
  Serial.println("Device connected to " + String(ssid));
  Serial.println("---------------------------------------");
}

/*! Function description
  @brief Callback function for when a Wifi message is received 
  \param  messageSize
*/
void receivedMqttMessage(int messageSize)
{
  // we received a message, print out the topic and contents
  Serial.println("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    mqttReceivedMessage += (char)mqttClient.read();
  }
  Serial.println(mqttReceivedMessage);
  Serial.println();
  Serial.println();
}

uint8_t checkEncoderDirection()
{
  uint8_t direction = still;
  // Read the current state of inputCLK
  bool currentStateCLK = digitalRead(encoderClkPin);
  // If the previous and the current state of the inputCLK are different then a pulse has occured
  if (currentStateCLK != previousStateCLK)
  { 
    // If the inputDT state is different than the inputCLK state then 
    // the encoder is rotating counterclockwise
    if (digitalRead(encoderDtPin) != currentStateCLK) { 
      encoderCounter--;
      direction = CCW;
      
    } else {
      // Encoder is rotating clockwise
      encoderCounter++;
      direction = CW;
    }
  } 
  // Update previousStateCLK with the current state
  previousStateCLK = currentStateCLK; 
  return direction;
}

void encoderClkHandler()
{
  encoderClkFlag = true;
  encoderClkTick = millis();
}

void encoderDtHandler()
{
  encoderDtFlag = true;
}

void encoderSwHandler()
{
  encoderSwFlag = true;
  encoderSwTick = millis();
}

void setTemperatureMenu()
{
  if(encoderSwFlag)
  {
    if (encoderClkFlag && (millis() - encoderClkTick >= 2))
    {
      if(digitalRead(encoderDtPin) != digitalRead(encoderClkPin)) 
      { 
        encoderCounter++;
      } 
      else // Encoder is rotating clockwise
      {
        encoderCounter--;
      }
      encoderClkFlag = false;
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Adjust inlet set temp:");
    display.setCursor(0, 10);
    display.println("Inlet Temp: " + String(encoderCounter) + " degC");
    display.display();
  }
  encoderSwFlag = false;
}
