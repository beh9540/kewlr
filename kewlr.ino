#include <DallasTemperature.h>

#include <OneWire.h>

#include <PID_v1.h>

// time in millis between state flips
#define WINDOW_SIZE 5000
// setup pin locations
#define ONE_WIRE_BUS 2
#define TEC_1_PIN 6
// serial port setup
#define SERIAL_BAUD_RATE 9600
#define SERIAL_TIMEOUT 10
#define SERIAL_BUFFER 200
// initial setpoint
#define DEFAULT_SETPOINT 50


String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean run = false; //whether or not we run

long windowStartTime;
double setPoint1, temperatureProbe1, output1;
//setup the PID with the links and initial tuning parameters
PID pid1(&temperatureProbe1, &output1, &setPoint1,2,5,1, DIRECT);
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);  

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  //set the timeout low, we don't want to block any longer than we
  //have to, so we can keep the run loop going
  Serial.setTimeout(SERIAL_TIMEOUT);
  inputString.reserve(SERIAL_BUFFER);
  
  windowStartTime = millis();

  //initialize the starting setpoint
  setPoint1 = DEFAULT_SETPOINT;

  //tell the PID to range between 0 and the full window size
  pid1.SetOutputLimits(0, WINDOW_SIZE);

  sensors.begin();
}

void loop() {
  if(run){
    pid1.Compute();
    
    unsigned long now = millis();
    if(now - windowStartTime>WINDOW_SIZE){
      //time to shift the Relay Window
      windowStartTime += WINDOW_SIZE;
    }
    if(output1 > now - windowStartTime){
      digitalWrite(TEC_1_PIN, HIGH);
    } else {
      digitalWrite(TEC_1_PIN,LOW);
    }
  }
  
  sensors.requestTemperatures();
  temperatureProbe1 = sensors.getTempFByIndex(0);
}

// while these seems async and super cool, it actually happens between calls to loop(), so it's reliable that when we 
// set the TEC pin low, and run false, the value will stay low
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    // if the incoming character is a newline, determine the change in state
    if (inChar == '\n') {
      if (inputString == "RUN"){
        run = true;
        pid1.SetMode(AUTOMATIC);
      } else if (inputString == "STOP"){
        run = false;
        // stop the PID
        pid1.SetMode(MANUAL);
        // turn the tec off
        digitalWrite(TEC_1_PIN,LOW);
      } else if (inputString == "STATUS"){
        String outputString = "RUN:"+String(run)+"\tSP1:"+String(setPoint1,1)+"\tT1:"+String(temperatureProbe1,1)+"\tDS:"+String(output1/WINDOW_SIZE*100,1);
        Serial.println(outputString);
      } else if (inputString.substring(0,2) == "SP") {
        String value = inputString.substring(3);
        // toInt returns 0 if it couldn't parse, we need to validate whether we want the value to be 0 or not
        long parsed = value.toInt();
        if (parsed == 0) {
          if (value == "0") {
            setPoint1 = 0;
          }
        } else {
          setPoint1 = parsed;
        }
      } else {
        Serial.println("Unknown command " + inputString);
      }
    } else {
      // add the character to the buffer
      inputString += inChar;
    }
  }
}
