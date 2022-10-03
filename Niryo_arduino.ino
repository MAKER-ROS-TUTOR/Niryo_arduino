#define MOTOR_ID   1  // <-- change this value for each stepper motor (1-4) on Niryo One

#include <Wire.h>
#include <SPI.h>

#include "config.h"
#include "AS5600.h"

#include "config.h"

#include "StepperController.h"
#include "CanBus.h"

uint8_t motor_id = MOTOR_ID;
// Stepper controller
StepperController stepper;

// Can driver
MCP_CAN can_driver(CAN_PIN_CS);

CanBus canBus(&can_driver,  &stepper, motor_id);

unsigned int driver_temperature = 0;

unsigned long time_last_write_position = micros();
unsigned long write_frequency_position = 80000; // 12.5Hz

unsigned long time_last_write_diagnostics = micros();
unsigned long write_frequency_diagnostics = 2000000; // 0.5Hz

unsigned long time_last_read_temperature = micros();
unsigned long read_temperature_frequency = 2000000; // 0.5 Hz

unsigned long time_last_write_firmware_version = micros();
unsigned long write_frequency_firmware_version = 5000000; // 0.2 Hz

void setup() {
   // This runs MCU at full speed. See comment below.
   
   Serial.begin(9600);
   Wire.begin();
   Wire.setClock(400000); //Not use 400KHz more noise 
   delay(2000);

   canBus.setup();

   // set register to read once
   init_position_sensor();
   delay(1000);

   stepper.start();
   stepper.setControlMode(STEPPER_CONTROL_MODE_RELAX);

 
 // myStepper.setCurrent(0);
   Serial.println("---------Success Setup --------");  
}
bool action_available = true;
bool analog_read_enable = true;
void loop() {
  
  action_available = true;
   
   // Change direction once the motor reaches target position
 

      // read position from sensor
      update_current_position(STEPPER_DEFAULT_MICRO_STEPS);

      // update stepper controller
      stepper.update();

    //  Serial.println(myStepper.currentPosition());

     // read CAN if available
  if (action_available) {
    if(canBus.available()) 
    {
      canBus.read();
      action_available = false;
     // Serial.println("CAN READ");
    }
  }
 
  // write position CAN
  if (action_available) {
    if (micros() - time_last_write_position > write_frequency_position) {
        time_last_write_position += write_frequency_position;
        canBus.writePosition();
        action_available = false;
        //Serial.println("[writePosition]");
    }
  }

  // write diagnostics CAN
  if (action_available) {
    if (micros() - time_last_write_diagnostics > write_frequency_diagnostics) {
      time_last_write_diagnostics += write_frequency_diagnostics;
    //  canBus.writeDiagnostics(stepper.getControlMode(), driver_temperature);
      action_available = false;
     // Serial.println("***writeDiagnostics...***");
    }
  }

  // write firmware version
  if (action_available) {
    if (micros() - time_last_write_firmware_version > write_frequency_firmware_version) {
      time_last_write_firmware_version += write_frequency_firmware_version;
      canBus.writeFirmwareVersion(NIRYO_STEPPER_VERSION_MAJOR, NIRYO_STEPPER_VERSION_MINOR, NIRYO_STEPPER_VERSION_PATCH);
      action_available = false;
      //  Serial.print("writeFirmware...");
      //  Serial.print(" ");
      //  Serial.print(NIRYO_STEPPER_VERSION_MAJOR);
      //  Serial.print(".");
      //  Serial.print(NIRYO_STEPPER_VERSION_MINOR);
      //  Serial.print(".");
      //  Serial.println(NIRYO_STEPPER_VERSION_PATCH);
    }
  }
  
 

}
