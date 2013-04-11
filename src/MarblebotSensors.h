//
//  MarblebotTodos.h
//  helloworld
//
//  Created by Simon Woodward on 14/12/2012.
//  Copyright (c) 2012 Simon Woodward. All rights reserved.
//

#ifndef __helloworld__MarblebotTodos__
#define __helloworld__MarblebotTodos__

// Set up all pins, etc. for the MarbleTodos
void sensorSetup();
void poll();

struct SerialSensorData : public Sensors::SensorData {
    int _byte;
};

class SerialSensor: public Sensors::Sensor {
protected:
    virtual Sensors::SensorData* doPoll();
public:
    virtual void setup(Sensors::Sensors* sensors);
};

/**********
  Manages the delivery of marbles and tracks where on the rails the bot
  is.
 **********/
// File numbers for the EEPROM
#define CHUTES_AND_HOPPERS 0

enum MarbleDelivererState {
  // We're going to the beginning of the track to recalibrate the zero position
  mdsZeroing,
  // States for marble delivery
  mdsGoingToHopper, mdsGettingMarble, mdsGoingToChute, mdsDroppingMarble, mdsWaitingForMarble,
  mdsReturnToStart, mdsDroppingRocker,
  // Just going to a destination, nothing more
  mdsGoto,
  mdsGoingToChuteOnly, 
  mdsGoingToHopperOnly,
  // Ready for some work
  mdsReady,
  
  // It's all gone horribly wrong and the bot needs resetting.
  mdsError};

// The pin the error LED is on
#define ERR_LED_PIN 2

// The pin the track beginning switch is on
#define TRACK_BEGIN_PIN 12

// Positions for the rocker 
#define SERVOPOS_GOTMARBLE 94
#define SERVOPOS_DROPMARBLE 55
#define SERVOPOS_START 135

// How wide the bits are
#define ROCKER_WIDTH 220
#define HOPPER_WIDTH 120

// How long to wait in millis before taking another step with the servo. 
#define SERVO_STEPDELAY 8

// How long to wait for the marble to roll off the rocker
#define MARBLE_ROLLOFF_DELAY 500

// How long to wait for a marble to go past the rocker LDR before going
// into error condition
#define GET_MARBLE_TIMEOUT 2000

// Numbers of hoppers/chutes we can support
#define HOPPER_COUNT 5
#define CHUTE_COUNT 15

// Number of slots to actually save in the EEPROM - leave ourselves some headroom!
#define HOPPER_SAVE_COUNT 10
#define CHUTE_SAVE_COUNT 30
#define CHUTES_AND_HOPPERS_SIZE HOPPER_SAVE_COUNT * sizeof(int) + CHUTE_SAVE_COUNT * sizeof(int)

// EEPROM file number for the deliverer
#define DELIVERER_FILE_NO 0

class MarbleDeliverer: public Sensors::Subscriber {
 protected:
  // If we're going somewhere, where?
  int _dest;

  // If we're in the middle of a marble delivery, the chute is 
  // stored here
  int _toChute;

  // Used to calculate the right delay for waiting for the marble to 
  // roll off the rocker
  Sensors::Waiter* _waiter;

  // What is the deliverer currently up to?
  MarbleDelivererState _state;

  // Step to hopper mapping
  int _hopperSteps[HOPPER_COUNT];
  // Step to chute mapping
  int _chuteSteps[CHUTE_COUNT];

  // Access to the switch at the start of the track.
  Sensors::SwitchSensor* _trackBeginSwitch;

  // Access to the motor
  Sensors::StepMotor* _stepMotor;

  // Access to the servo
  Sensors::SlowServo* _servo;

  // Access to the EEPROM
  Memory::Memory* _memory;

  // Access to the LDR on the rocker
  Sensors::VoltageDropSensor* _rockerLDR;

  // Error condition that's been set...
  char* _errorCondition;
  
  // Variables used when dropping multiple marbles.
  int _marblesToGo, _savedHopper, _savedChute;
    
  // Utility functions
  void saveArray(int* data, int datacount, int savecount);
  void loadArray(int* data, int datacount, int savecount);

  // put the deliverer into an err condition. This is an unrecoverable state,
  // and the bot needs resetting.
  void errorCondition(char* err); 

  // actual method to kick off the marble delivery; excludes all
  // business checks.
  void doMarble(int fromHopper, int toChute);    

  void gotoSafeDropPoint();

 public:
  // Reset all state (ie. delete all chutes and hoppers)
  void reset();

  // Hook all the objects up
  void setup(Sensors::SwitchSensor* trackBeginSwitch, Sensors::StepMotor* stepper, 
	     Sensors::SlowServo* servo, Memory::Memory* memory, Sensors::Waiter* waiter,
	     Sensors::VoltageDropSensor* rockerLDR);

  // Is the deliverer ready to do some work?
  bool busy() {return _state != mdsReady || _marblesToGo != 0;}
  // Print a decent error if the bot is busy
  bool checkBusy();
  // entry point to delivering a marble
  void marble(int fromHopper, int toChute);
  // entry point to delivering multiple marbles
  void marble(int fromHopper, int toChute, int count);
  // entry point to the goto command
  void goTo(int step);
  // move the rocker
  void rocker(int rockerpos);
    
  // Called by the sensors when we arrive
  void notify(Sensors::SensorData* data);

  // Emergency stop for the motor
  void stop();

  // Go to the beginning of the track and reset to 0.
  void zeroPosition();
  
  // Setup - map a step to a chute
  void setHopperStep(int hopper, int step);

  // Setup - map a step to a hopper
  void setChuteStep(int chute, int step);

  // Load settings from the eeprom
  void load();
  // Write chutes/hoppers to the eeprom
  void save();
  // Dump the config to the terminal
  void dumpConfig();
};

#define COMMAND_MAX_LEN 30
class CommandLineInterface : public Sensors::Subscriber {
protected:
    char _buffer[COMMAND_MAX_LEN];
    int _next;
    MarbleDeliverer* _marbleDeliverer;
    Memory::Memory* _memory;
public:
    // Set stuff up
    virtual void setup(MarbleDeliverer* marbleDeliverer, Memory::Memory* memory);
    // Notified when a byte arrives at the serial interface
    virtual void notify(Sensors::SensorData* data);
};

#endif /* defined(__helloworld__MarblebotTodos__) */
