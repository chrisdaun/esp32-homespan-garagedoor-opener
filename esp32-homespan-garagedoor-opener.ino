#include "Arduino.h"
#include "HomeSpan.h"

#define PIN_RELAY 5             //Pin 5 on ESP32 board
#define PIN_CLOSED_SENSOR 6     //Pin 6 on ESP32 board
#define HK_Open 0
#define HK_Closed 1
#define HK_Opening 2
#define HK_Closing 3
#define HK_stopped 4
#define S_CLOSED 0
#define S_OPEN 1
#define MOVE_AWAY_MS 1000
#define PULSE_MS 500
#define TIME_TO_OPEN_MS 15000

struct DEV_GarageDoor : Service::GarageDoorOpener {     // A Garage Door Opener

  Characteristic::CurrentDoorState *current;            // reference to the Current Door State Characteristic (specific to Garage Door Openers)
  Characteristic::TargetDoorState *target;             // reference to the Target Door State Characteristic (specific to Garage Door Openers)  
  SpanCharacteristic *obstruction;        // reference to the Obstruction Detected Characteristic (specific to Garage Door Openers)

  DEV_GarageDoor() : Service::GarageDoorOpener(){       // constructor() method
    current=new Characteristic::CurrentDoorState(1);              // initial value of 1 means closed
    target=new Characteristic::TargetDoorState(1);                // initial value of 1 means closed
    obstruction=new Characteristic::ObstructionDetected(false);   // initial value of false means NO obstruction is detected
    
    Serial.println("Configuring Garage Door Opener");   // initialization message
  } // end constructor

  boolean update(){
    // see HAP Documentation for details on what each value represents
    if(target->getNewVal()==HK_Open){                     // if the target-state value is set to 0, HomeKit is requesting the door to be in open position
      LOG1("Opening Garage Door\n");
      current->setVal(HK_Opening);                  // set the current-state value to 2, which means "opening"
      obstruction->setVal(false);                   // clear any prior obstruction detection
      pulse_door_relay();
    } else {
      LOG1("Closing Garage Door\n");                // else the target-state value is set to 1, and HomeKit is requesting the door to be in the closed position
      current->setVal(HK_Closing);                  // set the current-state value to 3, which means "closing"         
      obstruction->setVal(false);                   // clear any prior obstruction detection
      pulse_door_relay();
    }
    return(true);
  }

  void pulse_door_relay(){
    Serial.println("Triggering door relay");
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(PIN_RELAY, HIGH); // Relay is triggered active low
    delay(PULSE_MS);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(PIN_RELAY, LOW);
    delay(MOVE_AWAY_MS); // Give the door time to move away from the closed sensor
  }

  void loop(){
    int sensor_state = digitalRead(PIN_CLOSED_SENSOR); //LOW=Closed, HIGH=Open

    if (sensor_state==S_CLOSED && current->getVal()!=HK_Closed){
      current->setVal(HK_Closed); // Set the current state of the door to closed
      target->setVal(HK_Closed);
      Serial.println("Set the current state of the door to closed");
      delay(200);
      return;
    }
    // Sensor reports open but door state isn't open
    if (sensor_state==S_OPEN && current->getVal()!=HK_Open && current->timeVal()>TIME_TO_OPEN_MS){
      current->setVal(HK_Open); // Set the current state of the door to open
      target->setVal(HK_Open);
      Serial.println("Set the current state of the door to open");
      delay(200);
      return;
    }
  } // loop
};

void setup(){
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(PIN_CLOSED_SENSOR, INPUT_PULLUP);
  
  // place the following in setup() before homeSpan.begin()
  homeSpan.setWifiCallback([](){homeSpan.setPairingCode("32313231");});
  homeSpan.begin(Category::GarageDoorOpeners,"Chris Daun's Garage Door");

  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Identify();
      new Characteristic::Name("Garage Door");
      new Characteristic::Manufacturer("Christopher Daun");
      new Characteristic::Model("ESP32-C3 Super Mini");
      new Characteristic::SerialNumber("Nologo-esp32-c3-super-mini");
    new DEV_GarageDoor();

} // end of setup()

void loop(){
  homeSpan.poll();
} // end of loop()
