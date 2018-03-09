// Utility macros
#include "DebugUtils.h"

// Vector library
#include <vector>

// I2C library
#include <OneWire.h>

// Temperture library
#include <DallasTemperature.h>

// Software serial library
#include <SoftwareSerial.h>

// Simcom 808 library
#include "Adafruit_FONA.h"


// Setup software serial for lcd 
SoftwareSerial swSer(18, 19, false, 256);

// Setup temperature library
#define ONE_WIRE_BUS 22
//Setup to communicate with any OneWire Devices
OneWire oneWire(ONE_WIRE_BUS);
//Pass OneWire reference to Dallas Temperature
DallasTemperature sensors(&oneWire);
volatile double RefreshedTemp1;

// Define global variables for experiment tolerances
float InitialTemp1;
float Temp1Set;
float Temp1Min;
float Temp1Max;
float SetFlowRate;
float FlowRateMin;
float FlowRateMax;

// Setup timer & misc for flow
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Setup LCD & misc for flow
hw_timer_t * timer2 = NULL;
volatile SemaphoreHandle_t timer2Semaphore;
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;

// Setup temperature & misc for flow
hw_timer_t * timer3 = NULL;
volatile SemaphoreHandle_t timer3Semaphore;
portMUX_TYPE timer3Mux = portMUX_INITIALIZER_UNLOCKED;

// Save the calculated flow rate
volatile double FlowRate = 0;
// Where is the sensor connected to?
byte sensorInterrupt = 35;
// Total pulse count
volatile byte flowSensorCount;
// Last time the flow sensor's flow rate was calculated
volatile unsigned long lastFlowRateSample = millis();

// Calculated info on flow rate
float InitialFlowRate;
unsigned int InitialFlowMilliLitres;


// Set reset pin for sim com
#define FONA_RST 25
// PhoneNumber to send SMS message
String number = "+447745139107";
// Harware serial to communicate with simcom
HardwareSerial Serial2(2);
HardwareSerial *fonaSerial = &Serial2;
// set the simcom reset pins with the fona instance
Adafruit_FONA fona = Adafruit_FONA (FONA_RST);

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
unsigned long flashLED = 0;
bool flashLEDState = true;


// Buffer for flow and temperature
using namespace std;
vector<double> temperatureContainer;
vector<double> flowContainer;


/**
 * Setup program
 */
void setup(void)
{
    //Set pin mode for leds, siren and vibrator
    pinMode(LEDRed, OUTPUT);
    pinMode(LEDBlue, OUTPUT);
    pinMode(LEDRed2, OUTPUT);
    pinMode(Siren, OUTPUT);

    // Turn Red LED on pwm
    ledcSetup(1, 10000, 8); // PWM Channel, base frequency, resolution
    ledcAttachPin(LEDRed, 1); // GPIO Pin, PWM Channel
    ledcWrite(1, 20);

    // Initially flash the blue LED pwm
    ledcSetup(0, 10000, 8); // PWM Channel, base frequency, resolution
    ledcAttachPin(LEDBlue, 0); // GPIO Pin, PWM Channel
    ledcWrite(0, 128); // PWM Channel, Duty Cycle

    // Set up 2 Red LEDs and vibrator pwm
    ledcSetup(2, 10000, 8);
    ledcAttachPin(LEDRed2, 2);
    ledcWrite(2, 0);

    // Set up Siren pwm
    ledcSetup(3, 3100, 8); // (Timer Channel, Base Frequency, Resolution)
    ledcAttachPin(12, 3);   // (Pin, Timer Channel)
    ledcWrite(3, 0); // Turn siren    

    delay(2000);

    // Start lcd serial
    swSer.begin(9600);


    // Tell the user to wait
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
    // Get and fill the current temperature into the buffer container
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

    // Set up temperature timer
    timer3Semaphore = xSemaphoreCreateBinary();
    timer3 = timerBegin(2, 80, true);
    timerAttachInterrupt(timer3, &calculateTemperatures, true);
    timerAlarmWrite(timer3, 1500000, true);

    // Enable all our hardware timers
    timerAlarmEnable(timer);
    timerAlarmEnable(timer2);
    timerAlarmEnable(timer3);

    // Set up the flow sensor to an input
    pinMode(sensorInterrupt, INPUT);

    // Default the variables
    InitialFlowRate = 0.0;
    InitialFlowMilliLitres = 0;

    // Setup interrupt and attach a handling function for the flow sensor pulses
    attachInterrupt(digitalPinToInterrupt(sensorInterrupt), flowSensorHandler, FALLING);

    // Setup the fona / simcom 808 serial inerface
    fonaSerial->begin(9600); //4800

    // If the serial interface failed state that it did
    if (!fona.begin(*fonaSerial))
    {
        // Delay
        delay(500);
      
        // Flush buffer
        while(fonaSerial->available())
        {
            fonaSerial->read();
        }

        // Wait for a while
        delay(1000);

        // Try connecting one more time
        if (!fona.begin(*fonaSerial))
        {
            DEBUG_APP_PRINTLN(F("Couldn't find SIMCOM808"));
            while (1);
        }
    }

    // State everything went ok
    DEBUG_APP_PRINTLN(F("SIMCOM808 is connected"));

    // 16 character buffer for IMEI!
    char imei[15] = {0};

    // Delete message in slot 1 to make sure there is always space
    fona.deleteSMS(1);

    // OTHER
    RefreshRate = 5000; //5 seconds for not just for test
    LastRefresh = 0;
    
    // Set button pin to input with internal pullup
    pinMode(button, INPUT_PULLUP);

    // Settling time
    delay(1000);

    // State the lab safe is ready
    Serial.println("LabSafe Ready");
}





/**
 * Runs main program
 */
void loop()
{
    // Make sure the flow sensors has been read at least one
    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {

        // Read the current state of the button
        buttonState = digitalRead(button);

        // Get the current time stamp
        unsigned long currentTime = millis();

        // Flash LED every second
        if (currentTime - flashLED > 100)
        {
            // Cycle LED
            (flashLEDState) ? ledcWrite(0, 0) : ledcWrite(0, 10) ;
            // Save current state of LED
            flashLEDState = !flashLEDState;
            // Update last time the LED was cycled
            flashLED = currentTime;
        }

      
        // Set the first time the button is held down
        // Which can then be used to calculate the total time
        // The button has been held down
        if (buttonState == LOW && firstPress == -1)
        {
            // Detect the inital press
            DEBUG_APP_PRINTLN("Initial press");

            // Set the first time press value
            firstPress = currentTime;
        }
        
        
        // Start the experiment and set tolerances
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


            // Set Temperature
              portENTER_CRITICAL_ISR(&timerMux);
                  Temp1Set = (float) RefreshedTemp1;
              portEXIT_CRITICAL_ISR(&timerMux);
            
            Serial.print("Set Temperature is: ");
            Serial.print(Temp1Set);
            Serial.println();

            // Set Tolerance
            Temp1Min = Temp1Set - 3;
            Temp1Max = Temp1Set + 3;
            Serial.print("Temperature 1 min is: ");
            Serial.print(Temp1Min);
            Serial.println();
            Serial.print("Temperature 1 Max is: ");
            Serial.print(Temp1Max);
            Serial.println();

        }

        // Check experiment within tolerance
        while (buttonState == HIGH && firstPress != -1 && ((currentTime - firstPress) < 2000) && ((currentTime - firstPress) > 250))
        {
                    
            if ((millis() - LastRefresh) > RefreshRate)
            {
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


/**
 * Runs when an error has occured
 */
void errorEvent()
{
    // Convert number to required type
    char sendto[number.length()+1];
    number.toCharArray(sendto, number.length()+1);

    ledcWrite(0, 0); // Turn Blue LED off
    ledcWrite(2, 128); // Flash 2 Red LEDs/ Vibrate
    //ledcWrite(3, 128); // Turn siren on

    // IfSa temperature is out of tolerance SEND SMS
    noInterrupts();
    DEBUG_APP_PRINT("Signal Strength: ");
    DEBUG_APP_PRINTLN(fona.getRSSI());
    DEBUG_APP_PRINT("Network Status: ");
    DEBUG_APP_PRINTLN(fona.getNetworkStatus());
    if(!fona.sendSMS(sendto, "An error has occured"))
        DEBUG_APP_PRINTLN("Could not send sms");
    interrupts();
    
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

        if (buttonState == LOW && ((millis() - firstErrorAccept) > 2000))
        {
            // Turn off siren & leds
            ledcWrite(3, 0);
            ledcWrite(2, 0);
            
            // Reboot the ESP
            // ESP.restart();
            firstPress = -1;
            break;

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

    // Wait for finger to be removed
    delay(3000);
}


/**
 * Update lcd flow rate variable
 */
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


/**
 * Update lcd temperature variable
 */
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
 * Gets the current flow rates and adds to container 
 * 
 * @return 
 *    The container avaerage
 */
double getFlow()
{
  double flow = FlowRate;
    // Remove value if size is 5+
    if(flowContainer.size() >= 5)
        flowContainer.erase(flowContainer.begin());

    // add value to container
    flowContainer.push_back(flow);

    // Calculate container value
    double Total = 0;
    for(auto x:flowContainer)
      Total+= x;

    // Divide total by 5 to give average
    return (Total/5);
    
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
    if(temperatureContainer.size() >= 3)
        temperatureContainer.erase(temperatureContainer.begin());

    // Add value to container
    temperatureContainer.push_back(sensors.getTempCByIndex(0));

    // Create a container total value
    double average = 0;
    for(auto x:temperatureContainer)
        average += x;

    // Return the average of the five previous results    
    return (average / 3);
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
            double flowRate = getFlow();
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

