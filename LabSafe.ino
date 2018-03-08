// Import librarys
// Utilities
#include "DebugUtils.h"

// Temperture
#include <OneWire.h>
#include <DallasTemperature.h>

// SMS
#include "Adafruit_FONA.h"


// TEMPERATURE
//Temperature Data Wire plugged into 1022 (I2C SCL - Pin 36)
#define ONE_WIRE_BUS 22
//Setup to communicate with any OneWire Devices
OneWire oneWire(ONE_WIRE_BUS);
//Pass OneWire reference to Dallas Temperature
DallasTemperature sensors(&oneWire);
volatile double RefreshedTemp1;

float InitialTemp1;
float Temp1Set;
float Temp1Min;
float Temp1Max;

// Second timer
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// LCD
hw_timer_t * timer2 = NULL;
volatile SemaphoreHandle_t timer2Semaphore;
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;

// Temperature
hw_timer_t * timer3 = NULL;
volatile SemaphoreHandle_t timer3Semaphore;
portMUX_TYPE timer3Mux = portMUX_INITIALIZER_UNLOCKED;

// FLOW
volatile double FlowRate = 0;
// Flow sensor input IO35 (GPIO_35 - Pin 7)
byte sensorInterrupt = 35;
// Total pulse count
volatile byte flowSensorCount;
// Last time the flow sensor's flow rate was calculated
volatile unsigned long lastFlowRateSample = millis();

// Calculated info on flow rate
float InitialFlowRate;
unsigned int InitialFlowMilliLitres;
//unsigned long totalMilliLitres;


float SetFlowRate;
float FlowRateMin;
float FlowRateMax;

// SMS
// Set reset pin
#define FONA_RST 25
// PhoneNumber to send SMS message
#define PhoneNo "07745139107"
// Harware serial to communicate with simcom
HardwareSerial Serial2(2);
HardwareSerial *fonaSerial = &Serial2;
// set the simcom reset pins with the fona instance
Adafruit_FONA fona = Adafruit_FONA (FONA_RST);
// Create a buffer to store the text messages
char replybuffer[255];
// Declare the read text message function
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
// Notifications from the FONA library
char fonaInBuffer[64];


// OTHER
unsigned long RefreshRate;
unsigned long LastRefresh;


// Button
int button = 4;
// Save the time the button was first detected as pressed
unsigned long firstPress = -1;
// Save the state of the button (declare here so we dont redeclare every loop)
int buttonState;

// LED'S and Siren
int LEDRed = 33;
int LEDBlue = 27;
int LEDRed2 = 32;
int Siren = 12;

#include <SoftwareSerial.h>
SoftwareSerial swSer(18, 19, false, 256);


// Buffer for flow and temperature
#include <vector>
using namespace std;
vector<double> temperatureContainer;


/**
 * Setup program
 */
void setup(void)
{
    delay(2000);

    // Start lcd serial
    swSer.begin(9600);
    
    updateFlowRate("Wait");
    updateTemperature("Wait");
      
      
    //Start Serial Port
    Serial.begin(115200);
    Serial.println();
    Serial.println("Kerris Labsafe Version 1");

    // Setup the temperature sensor library
    sensors.begin();
    // Print the number of devices found
    DEBUG_APP_PRINT("Locating temperature sensors...");
    DEBUG_APP_PRINT("Found ");
    DEBUG_APP_PRINT(sensors.getDeviceCount(), DEC);
    DEBUG_APP_PRINTLN(" sensors.");
    // Update the class buffer for temperature variables
    sensors.requestTemperatures();
    // Get the current temperature
    for(int i=0; i<5; i++)
      getTemperature();
    // Make the library work in async mode
    sensors.setWaitForConversion(false);

    // Setup flow rate timer
    timerSemaphore = xSemaphoreCreateBinary();
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &calculateFlowRate, true);
    timerAlarmWrite(timer, 1000000, true);
    
    // Set up lcd timer
    timer2Semaphore = xSemaphoreCreateBinary();
    timer2 = timerBegin(1, 80, true);
    timerAttachInterrupt(timer2, &updateLcd, true);
    timerAlarmWrite(timer2, 2000000, true);

    // Set up temperature
    timer3Semaphore = xSemaphoreCreateBinary();
    timer3 = timerBegin(2, 80, true);
    timerAttachInterrupt(timer3, &calculateTemperatures, true);
    timerAlarmWrite(timer3, 150000, true);

    timerAlarmEnable(timer);
    timerAlarmEnable(timer2);
    timerAlarmEnable(timer3);

    // LED's and Siren
    //Set pin mode
    pinMode(LEDRed, OUTPUT);
    pinMode(LEDBlue, OUTPUT);
    pinMode(LEDRed2, OUTPUT);
    pinMode(Siren, OUTPUT);

    // Turn Red LED on
    ledcSetup(1, 10000, 8); // PWM Channel, base frequency, resolution
    ledcAttachPin(LEDRed, 1); // GPIO Pin, PWM Channel
    ledcWrite(1, 20);

    // Initially flash the blue LED
    ledcSetup(0, 10000, 8); // PWM Channel, base frequency, resolution
    ledcAttachPin(LEDBlue, 0); // GPIO Pin, PWM Channel
    ledcWrite(0, 128); // PWM Channel, Duty Cycle

    // Set up 2 Red LEDs and vibrator
    ledcSetup(2, 10000, 8);
    ledcAttachPin(LEDRed2, 2);
    ledcWrite(2, 0);

    // Set up Siren
    ledcSetup(3, 3100, 8); // (Timer Channel, Base Frequency, Resolution)
    ledcAttachPin(12, 3);   // (Pin, Timer Channel)
    ledcWrite(3, 0); // Turn siren    

    // FLOW
    // Set up the flow controller input
    pinMode(sensorInterrupt, INPUT);

    // Default the variables
    InitialFlowRate = 0.0;
    InitialFlowMilliLitres = 0;


    // Setup interrupt and attach a handling function
    attachInterrupt(digitalPinToInterrupt(sensorInterrupt), flowSensorHandler, FALLING);

    // SMS
    // Setup the fona / simcom 808 serial inerface
    fonaSerial->begin(9600); //4800

    // If the serial interface failed state that it did
    if (!fona.begin(*fonaSerial))
    {
        DEBUG_APP_PRINTLN(F("Couldn't find SIMCOM808"));
        while (1);
    }

    // State everything went ok
    DEBUG_APP_PRINTLN(F("SIMCOM808 is connected"));

    // 16 character buffer for IMEI!
    char imei[15] = {0};

    // Delete message in slot 1 to make sure there is always space
    fona.deleteSMS(1);

    // State that the SIMCOM is now setup and we're moving to the main program loop
    DEBUG_APP_PRINTLN("SIMCOM808 Ready");

    // OTHER
    RefreshRate = 5000; //5 seconds for not just for test
    LastRefresh = 0;

    // Button
    // Set button pin to input with internal pullup
    pinMode(button, INPUT_PULLUP);

    delay(1000); // wait to settle

    Serial.println("LabSafe Ready");
}

unsigned long flashLED = 0;
bool flashLEDState = true;



/**
 * Runs program
 */
void loop()
{
  
    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {

        // Read the current state of the button
        buttonState = digitalRead(button);

        // Get the current time stamp
        unsigned long currentTime = millis();

        // Flash LED every second
        if (currentTime - flashLED > 1000)
        {
            // Cycle LED
            (flashLEDState) ? ledcWrite(0, 0) : ledcWrite(0, 10) ;
            // Save current state of LED
            flashLEDState = !flashLEDState;
            // Update last time the LED was cycled
            flashLED = currentTime;
        }

      
        // Check to see if the any press conditions match
        if (buttonState == LOW && firstPress == -1)
        {
            // Detect the inital press
            DEBUG_APP_PRINTLN("Initial press");


            // Set the first time press value
            firstPress = currentTime;
        }
        // On button click less than 2 AND greater than 0.5
        if (buttonState == HIGH && firstPress != -1 && ((currentTime - firstPress) < 2000) && ((currentTime - firstPress) > 500))
        {

            
            // Detect a short press
            DEBUG_APP_PRINTLN("Short press");

            ledcWrite(0, 20);  // turn the Blue LED on all the time

            // FLOW:
            portENTER_CRITICAL(&timerMux);
            SetFlowRate = (float) FlowRate;
            portEXIT_CRITICAL(&timerMux);
            Serial.print("The Set Flow Rate is: ");
            Serial.print(SetFlowRate);
            Serial.print("ml/Sec");
            Serial.println();

            // Tollerance
            FlowRateMin = SetFlowRate - 20;
            FlowRateMax = SetFlowRate + 20;

            Serial.print("Minimum Flow Rate: ");
            Serial.print(FlowRateMin);
            Serial.print("ml/Sec");
            Serial.println();
            Serial.print("Maximum Flow Rate: ");
            Serial.print(FlowRateMax);
            Serial.print("ml/Sec");
            Serial.println();

            // TEMPERATURE:
            // Reading Temperature
//            DEBUG_APP_PRINTLN("Reading Temperature... ");
//            sensors.requestTemperatures();
//            DEBUG_APP_PRINT("Read ");

//            // Printing Read Temperature
//            DEBUG_APP_PRINTLN("Temperature 1 is: ");
//            DEBUG_APP_PRINT(sensors.getTempCByIndex(0));
//
//            // Read Temperature
//            InitialTemp1 = sensors.getTempCByIndex(0);
//            DEBUG_APP_PRINTLN("Temperature 1 Read is: ");
//            DEBUG_APP_PRINT(InitialTemp1);
//            DEBUG_APP_PRINTLN();

            // Set Temperature
            
  
              portENTER_CRITICAL_ISR(&timerMux);
                  Temp1Set = (float) RefreshedTemp1;
              portEXIT_CRITICAL_ISR(&timerMux);
            
            Serial.print("Set Temperature is: ");
            Serial.print(Temp1Set);
            Serial.println();

            // Set Tolerance
            Temp1Min = Temp1Set - 6;
            Temp1Max = Temp1Set + 6;
            Serial.print("Temperature 1 min is: ");
            Serial.print(Temp1Min);
            Serial.println();
            Serial.print("Temperature 1 Max is: ");
            Serial.print(Temp1Max);
            Serial.println();

        }
        else if (buttonState == HIGH && firstPress != -1 && ((currentTime - firstPress) > 5000))
        {
            // Detect a long press 10sec
            DEBUG_APP_PRINTLN("Long press");
            DEBUG_APP_PRINTLN();
            Serial.println("Resetting Microcontroller");

            // Reboot the ESP
            ESP.restart();
        }


        while (buttonState == HIGH && firstPress != -1 && ((currentTime - firstPress) < 2000) && ((currentTime - firstPress) > 500))
        {
                    
            if ((millis() - LastRefresh) > RefreshRate)
            {
                //float RefreshedTemp1;

                // Reading and Printing Temp every Refresh rate
//                sensors.requestTemperatures();
//                RefreshedTemp1 = sensors.getTempCByIndex(0);
//                Serial.print("Current Temp 1 is: ");
//                Serial.print(RefreshedTemp1);
//                Serial.println();

                portENTER_CRITICAL_ISR(&timerMux);
                  float temperature = (float) RefreshedTemp1;
                portEXIT_CRITICAL_ISR(&timerMux);

                // Checking if temp is within min/max
                if  ((temperature < Temp1Min) || (temperature > Temp1Max))
                {
                    errorEvent();
                }

                //READ FLOW
                portENTER_CRITICAL(&timerMux);
                  unsigned long tempLastTime = lastFlowRateSample;
                  float tempFlowRate = (float) FlowRate;
                portEXIT_CRITICAL(&timerMux);
                Serial.print("Current Flow is: ");
                Serial.print(tempFlowRate);
                Serial.print("ml/Sec");
                Serial.println();
                if ((tempFlowRate < FlowRateMin) || (tempFlowRate > FlowRateMax))
                {
                    errorEvent();
                }


                LastRefresh = millis();
            }


        }

    }

}


void errorEvent()
{
    Serial.println();
    Serial.print("error");
    Serial.println();


    ledcWrite(0, 0); // Turn Blue LED off
    ledcWrite(2, 128); // Flash 2 Red LEDs/ Vibrate
    ledcWrite(3, 128); // Turn siren on

    // If temperature is out of tolerance SEND SMS
    fona.sendSMS(PhoneNo, "An error has occured");

    // Flash LED & Buzzer
    unsigned long flashError = 0;
    bool flashErrorState = true;

    // Reset last time press
    bool errorState = true;
    unsigned long firstErrorAccept = -1;

    while (errorState)
    {
        // Read button
        buttonState = digitalRead(button);

        // Check to see if the any press conditions match
        if (buttonState == LOW && firstErrorAccept == -1)
        {
            // Set the first time press value
            firstErrorAccept = millis();
        }
        else if (buttonState == HIGH && firstErrorAccept != -1)
        {
            firstErrorAccept = -1;
        }

        if (buttonState == LOW && ((millis() - firstErrorAccept) > 5000))
        {
            // Reboot the ESP
            ESP.restart();

            errorState = false;
        }

        // Flash LED every second
        if (millis() - flashError > 1000)
        {
            // Cycle LED
            if (flashErrorState)
            {
                ledcWrite(2, 0);
                ledcSetup(3, 3100, 8);
                ledcWrite(3, 128);
            }
            else
            {
                ledcWrite(2, 128);
                ledcSetup(3, 1000, 8);
                ledcWrite(3, 128);
            }

            // Save current state of LED
            flashErrorState = !flashErrorState;
            // Update last time the LED was cycled
            flashError = millis();
        }
    }
}


void updateFlowRate(String value)
{
    swSer.write(0xff);
    swSer.write(0xff);
    swSer.write(0xff);
    // Serial1.print("v1.txt=\"52\"");
    swSer.print("v2.txt=\""+value+"\"");
    swSer.write(0xff);
    swSer.write(0xff);
    swSer.write(0xff);
}

void updateTemperature(String value)
{
    swSer.write(0xff);
    swSer.write(0xff);
    swSer.write(0xff);
    // Serial1.print("v1.txt=\"52\"");
    swSer.print("v1.txt=\""+value+"\"");
    swSer.write(0xff);
    swSer.write(0xff);
    swSer.write(0xff);
}


/**
 * Gets the new temperature value, adds to container, 
 * 
 * @return double 
 *          average of the container
 */
double getTemperature()
{
    // Remove value if size is 5+
    if(temperatureContainer.size() >= 5)
        temperatureContainer.erase(temperatureContainer.begin());

    // Add value to container
    temperatureContainer.push_back(sensors.getTempCByIndex(0));

    // Create a container total value
    double average = 0;
    for(auto x:temperatureContainer)
        average += x;

    // Return the average of the five previous results    
    return (average / 5);
}


/**
 * Increments the counter for the flow sensor
 * 
 * @param i magic variable
 * @return void
 */
void flowSensorHandler()
{
    // Locking access to ISR
    portENTER_CRITICAL_ISR(&timerMux);
      // Increment the pulse counter
      flowSensorCount++;
    portEXIT_CRITICAL_ISR(&timerMux);
}


void IRAM_ATTR updateLcd()
{
      noInterrupts();
      portENTER_CRITICAL_ISR(&timer2Mux);
          // Update flow rate
          portENTER_CRITICAL_ISR(&timerMux);
            double flowRate = FlowRate;
            updateFlowRate(String(flowRate));
          portEXIT_CRITICAL_ISR(&timerMux);

          // Update temperature
          portENTER_CRITICAL_ISR(&timer3Mux);
            // Caluclate the temperature from the class buffer
            double currentTemp = getTemperature();
            // Save the temperature to the global variabel
            RefreshedTemp1 = currentTemp;
            // Update the lcd display
            updateTemperature(String(currentTemp));
          portEXIT_CRITICAL_ISR(&timer3Mux);
      portEXIT_CRITICAL_ISR(&timer2Mux);
     interrupts();
  xSemaphoreGiveFromISR(timer2Semaphore, NULL);
}


void IRAM_ATTR calculateTemperatures()
{
    portENTER_CRITICAL_ISR(&timer3Mux);
        // Request for new temperature information 
        sensors.requestTemperatures();
    portEXIT_CRITICAL_ISR(&timer3Mux);
    xSemaphoreGiveFromISR(timer3Semaphore, NULL);
}

/**
 * Calculating flow rate, triggered every 1 second
 */
void IRAM_ATTR calculateFlowRate()
{
    portENTER_CRITICAL_ISR(&timerMux);
      // The hall-effect flow sensor outputs approximately 4.5 pulses per second per litre/minute of flow.
      double flowUnformatted = ((1000.0 / (millis() - lastFlowRateSample)) * flowSensorCount) / 4.5;
      flowSensorCount = 0;
      lastFlowRateSample = millis();
      FlowRate = (flowUnformatted / 60) * 1000;
    portEXIT_CRITICAL_ISR(&timerMux);
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

