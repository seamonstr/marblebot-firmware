//
//  MarblebotTodos.cpp
//  helloworld
//
//  Created by Simon Woodward on 14/12/2012.
//  Copyright (c) 2012 Simon Woodward. All rights reserved.
//
#include <stdlib.h>
#include <assert.h>

#include "Arduino.h"
#include "Servo.h"

#include "memory.h"
#include "Sensors.h"
#include "MarblebotSensors.h"

#define ORANGE 8
#define BLUE 9
#define RED 10
#define YELLOW 11

#define SERVO 3
#define ROCKER_LDR_PIN 0


Sensors::Sensors sensors;

CommandLineInterface cmdLine;
MarbleDeliverer marbleDeliverer;

// The switch at the start of the track; used to init the 0 value.
Sensors::SwitchSensor trackBeginSwitch;

Sensors::StepMotor stepper(YELLOW, BLUE, RED, ORANGE);
Sensors::SlowServo servo;
SerialSensor serialSensor;
Sensors::Waiter delivererWaiter; // a generic waiter used by the marble deliverer.
Sensors::VoltageDropSensor rockerLDR(ROCKER_LDR_PIN);
Memory::Memory memory;

/**********
Logging
 **********/
void logDebug(char* str) {
  Serial.print("DEBUG: ");
  Serial.println(str);
}

/***********
Hook all the objects up together
 ***********/
void sensorSetup() {
  trackBeginSwitch.setup(TRACK_BEGIN_PIN);

  sensors.addSensor(&stepper);
  sensors.addSensor(&serialSensor);
  sensors.addSensor(&servo);
  sensors.addSensor(&delivererWaiter);
  sensors.addSensor(&rockerLDR);
  sensors.addSensor(&trackBeginSwitch);

  stepper.setSubscriber(&marbleDeliverer);
  servo.setSubscriber(&marbleDeliverer);
  delivererWaiter.setSubscriber(&marbleDeliverer);
  // The rocker LDR doesn't get a subscriber because we don't want it to notify.
  // the marble deliverer uses the voltage drop sensor's trigger mechanism to 
  // figure out if the marble has been picked up, not the notification.
  //    rockerLDR.setSubscriber(&marbleDeliverer);
  serialSensor.setSubscriber(&cmdLine);
  trackBeginSwitch.setSubscriber(&marbleDeliverer);

  cmdLine.setup(&marbleDeliverer, &memory);
  marbleDeliverer.setup(&trackBeginSwitch, &stepper, &servo, &memory, 
			&delivererWaiter, &rockerLDR);
  servo.setup(SERVO, SERVOPOS_START, SERVO_STEPDELAY);

  // Now that's done, zero the position.
  marbleDeliverer.zeroPosition();
}

void poll() {
    sensors.poll();
}

/**************
 SerialSensor class
 **************/
SerialSensorData data;

// Return one byte off of the serial buffer
Sensors::SensorData* SerialSensor::doPoll() {
    if (Serial.available() > 0) {
        data._sensor = this;
        data._byte = Serial.read();
        return &data;
    }
    return NULL;
}

void SerialSensor::setup(Sensors::Sensors* sensors) {
    Sensors::Sensor::setup(sensors);
    Serial.begin(9600);
}


/**************
 * Sensor that monitors the serial interface, parses commands and invokes
 * the right stuff in response.
***************/

// "marble x y"
#define MARBLE_COMMAND "marble"
#define THIRTY_COMMAND "thirty"
#define CHUTE_SETUP_COMMAND "setchute"
#define HOPPER_SETUP_COMMAND "sethopper"
#define SET_ROCKER_LDR_THRESHOLD "setldr"
#define LDR_OUTPUT_ON "ldrvals"
#define GOTO_COMMAND "goto"
#define ROCKER_PICKUP_COMMAND "rp"
#define ROCKER_DROP_COMMAND "rd"
#define ROCKER_START_COMMAND "rs"
 
void CommandLineInterface::setup(MarbleDeliverer* marbleDeliverer, Memory::Memory* memory) {
  _memory = memory;
  _marbleDeliverer = marbleDeliverer;
}

/* 
   Parse two integers separated by a space out of buff.  The first integer 
   must start at offset 0; there must be only 1 space; both integers must
   be valid.  Return false if anything fails.
*/
bool twoIntParams( char* buff, int* int1, int* int2) {
  char *end;
  // First int after the "marble " bit
  *int1 = strtol(buff, &end, 10);
  *int2 = -1;
  bool ret = false;
  // Check:
  // 1) end != the start of the int (which would indicate error)
  // 2) we're still pointing within our buffer (ie. there's more data)
  if ((end != buff) &&
      (end - buff < strlen(buff))) {
	char* pEnd = NULL;
	*int2 = strtol(end, &pEnd, 10);
	if (pEnd != end && *pEnd == 0) // error?
	  ret = true;
      }
  return ret;
}

bool oneIntParam(char* buff, int* param) {
  char* end;
  *param = strtol(buff, &end, 10);
  return end != buff;
}

  /*
    if buffer start with command, try to parse two ints out of the remainder
    of buffer.  the syntax is:
    "command 1 2" - only one space for each delimeter.

    If any syntax fails, return false.  Else, return true.
  */
bool parseTwoIntCommand(char* buffer, char* command, int* int1, int* int2) {
  int cmdLen = strlen(command);
  int bufLen = strlen(buffer);
  // Needs to space in the buffer for at least the command and two single-
  // digit integers
  return (bufLen >= cmdLen + 4) && 
    strncmp(buffer, command, cmdLen) == 0 &&
    twoIntParams(&(buffer[cmdLen + 1]), int1, int2);
}

bool parseOneIntCommand(char* buffer, char* command, int* param) {
  int cmdLen = strlen(command);
  int bufLen = strlen(buffer);
  return (bufLen >= cmdLen + 2) &&
    strncmp(buffer, command, cmdLen) == 0 &&
    oneIntParam(&(buffer[cmdLen + 1]), param);
}

void doubleIntHelp(char* command) {
  Serial.print(command);
  Serial.println(" x y");
}

void singleIntHelp(char* command) {
  Serial.print(command);
  Serial.println(" x");  
}

void CommandLineInterface::notify(Sensors::SensorData* data) {
  SerialSensorData *serialData = (SerialSensorData*)data;
  if (_next == COMMAND_MAX_LEN) {
    Serial.println("Cmd too long");
    _next = 0;
  }
  // Echo - be polite to the serial user
  Serial.write(serialData->_byte);
  int int1, int2;
  if (serialData->_byte == '\r') {
    Serial.write('\n'); // more politeness - need '\r\n'
    _buffer[_next] = 0;
    if (_next == 0) {
      // Just pressed enter - emergency stop
      _marbleDeliverer->stop();
      Serial.println("Motor off");
    } else if (strcmp(_buffer, "?") == 0) {
      // Help
      doubleIntHelp(MARBLE_COMMAND);
      doubleIntHelp(CHUTE_SETUP_COMMAND);
      doubleIntHelp(HOPPER_SETUP_COMMAND);
      singleIntHelp(GOTO_COMMAND);
      singleIntHelp(SET_ROCKER_LDR_THRESHOLD);
      Serial.println(LDR_OUTPUT_ON);
      Serial.println(ROCKER_PICKUP_COMMAND);
      Serial.println(ROCKER_DROP_COMMAND);
      Serial.println(ROCKER_START_COMMAND);
      Serial.println("save");
      Serial.println("reload");
      Serial.println(THIRTY_COMMAND);
    } else if (strcmp(_buffer, "format") == 0) {
      if (!_marbleDeliverer->checkBusy()) {
        Serial.println("Wiping all data.  You asked for it.");
        _memory->addFile(CHUTES_AND_HOPPERS, CHUTES_AND_HOPPERS_SIZE);
        _marbleDeliverer->reset();
        _marbleDeliverer->save();
      }
    } else if (parseTwoIntCommand(_buffer, THIRTY_COMMAND, &int1, &int2)) {
      if (!_marbleDeliverer->checkBusy())
	_marbleDeliverer->marble(int1, int2, 30); 
    } else if (strcmp(_buffer, "save") == 0) {
	_marbleDeliverer->save();
    } else if (strcmp(_buffer, "reload") == 0) {
	_marbleDeliverer->load();
    } else if (strcmp(_buffer, "dump") == 0) {
	_marbleDeliverer->dumpConfig();
    } else if (strcmp(_buffer, ROCKER_DROP_COMMAND) == 0) {
      _marbleDeliverer->rocker(SERVOPOS_DROPMARBLE);
    } else if (strcmp(_buffer, ROCKER_PICKUP_COMMAND) == 0) {
      _marbleDeliverer->rocker(SERVOPOS_GOTMARBLE);
    } else if (strcmp(_buffer, ROCKER_START_COMMAND) == 0) {
      _marbleDeliverer->rocker(SERVOPOS_START);
    } else if (strcmp(_buffer, LDR_OUTPUT_ON) == 0) {
      rockerLDR.writeToSerial();
    } else if (parseOneIntCommand(_buffer, SET_ROCKER_LDR_THRESHOLD, &int1)) {
      rockerLDR.setThreshold(int1);
    } else if (parseTwoIntCommand(_buffer, CHUTE_SETUP_COMMAND, &int1, &int2)) {
      _marbleDeliverer->setChuteStep(int1, int2);
    } else if (parseTwoIntCommand(_buffer, HOPPER_SETUP_COMMAND, &int1, &int2)) {
      _marbleDeliverer->setHopperStep(int1, int2);
    } else if (parseTwoIntCommand(_buffer, MARBLE_COMMAND, &int1, &int2)) {
      if (!_marbleDeliverer->checkBusy())
	_marbleDeliverer->marble(int1, int2);
    } else if (parseOneIntCommand(_buffer, GOTO_COMMAND, &int1)) {
      if (!_marbleDeliverer->checkBusy())
	_marbleDeliverer->goTo(int1);
    } else {
      Serial.println("Error in the command syntax");
    }
    _next = 0;
  } else
    _buffer[_next++] = serialData->_byte;
}

/************
 MarbleDeliverer
 ************/
void MarbleDeliverer::setup(Sensors::SwitchSensor* trackBeginSwitch, Sensors::StepMotor* stepMotor, 
			    Sensors::SlowServo* servo, Memory::Memory* memory, Sensors::Waiter* waiter, 
			    Sensors::VoltageDropSensor* rockerLDR ) {
  _stepMotor = stepMotor;
  _servo = servo;
  _memory = memory;
  _waiter = waiter;
  _rockerLDR = rockerLDR;
  _trackBeginSwitch = trackBeginSwitch;
  
  reset();
  load();
}

void MarbleDeliverer::reset() {
  _state = mdsReady;
  for (int i = 0; i < HOPPER_COUNT; i++)
    setHopperStep(i, -1);
  for (int i = 0; i < CHUTE_COUNT; i++)
    setChuteStep(i, -1);
}

void MarbleDeliverer::setHopperStep(int hopper, int step) {
  if (hopper >= HOPPER_COUNT) {
    Serial.println("Hopper count is out of range!");
  }
  _hopperSteps[hopper] = step;
}

void MarbleDeliverer::setChuteStep(int chute, int step) {
  if (chute >= CHUTE_COUNT) {
    Serial.println("Chute count is out of range!");
  }
  _chuteSteps[chute] = step;
}

void MarbleDeliverer::loadArray(int* data, int datacount, int savecount) {
  for (int i = 0; i < savecount; i ++) {
    int val;
    _memory->read(&val, sizeof(int));
    if (i < datacount) 
      data[i] = val;
  }
}

void MarbleDeliverer::saveArray(int* data, int datacount, int savecount) {
  for (int i = 0; i < savecount; i ++) {
    int val = (i < datacount ? data[i] : -1);
    _memory->write(&val, sizeof(int));
  }
}

void MarbleDeliverer::save() {
  _memory->open(DELIVERER_FILE_NO);
  saveArray(_hopperSteps, HOPPER_COUNT, HOPPER_SAVE_COUNT);
  saveArray(_chuteSteps, CHUTE_COUNT, CHUTE_SAVE_COUNT);
  int rockerLDRThreshold = _rockerLDR->threshold();
  _memory->write(&rockerLDRThreshold, sizeof(int));
}

void MarbleDeliverer::load() {
  _memory->open(DELIVERER_FILE_NO);
  loadArray(_hopperSteps, HOPPER_COUNT, HOPPER_SAVE_COUNT);
  loadArray(_chuteSteps, CHUTE_COUNT, CHUTE_SAVE_COUNT);
  int rockerLDRThreshold;
  _memory->read(&rockerLDRThreshold, sizeof(int));
  _rockerLDR->setThreshold(rockerLDRThreshold);
}

void dumpArray(char* command, int* array, int len) {
  for (int i = 0; i < len; i ++) {
    if (array[i] == -1)
      break;
    char buff[20];
    sprintf(buff, "%s %d %d", command, i, array[i]);
    Serial.println(buff);
  }
}

void MarbleDeliverer::dumpConfig() {
  dumpArray(CHUTE_SETUP_COMMAND, _chuteSteps, CHUTE_COUNT);
  dumpArray(HOPPER_SETUP_COMMAND, _hopperSteps, HOPPER_COUNT);
  char buff[20];
  sprintf(buff, "setldr %d", _rockerLDR->threshold());
  Serial.println(buff);
}

void MarbleDeliverer::marble(int fromHopper, int toChute, int count) {
  if (checkBusy())
    return;

  _savedHopper = fromHopper;
  _savedChute = toChute;
  _marblesToGo = count - 1; // _marblesToGo doesn't include the current one
  doMarble(fromHopper, toChute);
}

void MarbleDeliverer::marble(int fromHopper, int toChute) {
  if (checkBusy())
    return;
  
  doMarble(fromHopper, toChute);
}

void MarbleDeliverer::doMarble(int fromHopper, int toChute) {
  if (_servo->pos() != SERVOPOS_START) {
    Serial.println("Servo not at start pos.");
    return;
  }

  if (fromHopper >= HOPPER_COUNT || toChute >= CHUTE_COUNT) {
    Serial.println("Requested hopper or chute is out of range.");
    return;
  }

  if (_chuteSteps[toChute] == -1) {
    Serial.println("Requested chute has not been set up.");
    return;
  }

  if (_hopperSteps[fromHopper] == -1) {
    Serial.println("Requested hopper has not been set up.");
    return;
  }

  _toChute = toChute;
  _state = mdsGoingToHopper;
  _stepMotor->gotoStep(_hopperSteps[fromHopper]);
}

bool  MarbleDeliverer::checkBusy() {
  if (busy()) {							    
    if (_state == mdsError) {      
      Serial.print("The bot is in an error state and needs resetting: ");
      Serial.println(_errorCondition);
    } else 
      Serial.println("MarbleDeliverer called, even though I'm already busy.  Rude.");
    return true;
  }
  return false;
}

void MarbleDeliverer::goTo(int step) {
  if (checkBusy())
    return;
  _state = mdsGoto;
  _stepMotor->gotoStep(step);
}

void MarbleDeliverer::rocker(int rockerpos) {
  if (checkBusy())
    return;

  _servo->goTo(rockerpos);
}

// Called by the stepper motor when we reach our destination
void MarbleDeliverer::notify(Sensors::SensorData* data) {
  
  // If the trackBeginSwitch changes state, that needs special handling regardless
  // of our state.
  if (data->_sensor == _trackBeginSwitch) {
    // Whatever happens, we need to stop the motor.
    _stepMotor->stop();

    Sensors::SwitchData *swData = ((Sensors::SwitchData*)data);
    
    if (_state != mdsZeroing && swData->_val == LOW) {
      // If we're not zeroing and the switch just closed, our positioning is screwed.  Zero.
      // Don't call zero() because that switches the motor on.
      _state = mdsZeroing;
    }
    
    if (_state == mdsZeroing) {
      if (swData->_val == LOW) {
	// If we're zeroing and the switch just closed then the car is in the bit too
	// close to the step motor.  We need to move forward until it opens again.
	_stepMotor->zero();
	// Go to an arbitrarily high step; ie. just go forward until the switch opens
	_stepMotor->gotoStep(10000);
      } else {
	// If we're zeroing and the switch just became open then stop here - this is zero.
	_stepMotor->zero();
	_state = mdsReady;
      }
    }
    return;
  }
  
  switch (_state) {
  case mdsGoto:
    _state = mdsReady;
    break;
  case mdsGoingToHopper:
    // Right, we're at the hopper.  Grab the marble; enable the rockerLDR
    // to spot if we've successfully got one.
    _servo->goTo(SERVOPOS_GOTMARBLE);
    _rockerLDR->watchForTrigger();
    _state = mdsGettingMarble;
    break;
  case mdsGettingMarble:
    if (data->_sensor != _waiter && data->_sensor != _servo) {
      errorCondition("Checking to see if we picked up a marble, but notifier is "
		     "neither servo not wait-a-bit thingy.");
    }
    // If it's the servo notifying us it's finished moving then this is the first time through
    // the notify() for this marble. Check if the LDR spotted one get picked up.
    // Only proceed if the rocker LDR spotted a marble.
    if (_rockerLDR->triggered()) {
      _stepMotor->gotoStep(_chuteSteps[_toChute]);
      _state = mdsGoingToChute;
    } else {
      if (data->_sensor == _servo) {
	// no, no marble yet - wait for a bit
	_waiter->wait(GET_MARBLE_TIMEOUT);
      } else if (data->_sensor == _waiter) {
	// Yikes! No marble got picked up.  Light up the red LED and stop.
	errorCondition("No marble spotted on the rocker.  Are all hoppers full and aligned?");
      }
    }
    break;

  case mdsGoingToChute:
    // Trigger the servo.
    _servo->goTo(SERVOPOS_DROPMARBLE);
    _state = mdsDroppingMarble;
    break;

  case mdsDroppingMarble:
    // Okay, servo's finished;  wait for the marble to roll off
    _waiter->wait(MARBLE_ROLLOFF_DELAY);
    _state = mdsWaitingForMarble;
    break;

  case mdsWaitingForMarble:
    gotoSafeDropPoint();
    _state = mdsDroppingRocker;
    break;

  case mdsDroppingRocker:
    _servo->goTo(SERVOPOS_START);
    _state = mdsReturnToStart;
    break;

  case mdsReturnToStart:
    Serial.println("Marble dropped.");
    _state = mdsReady;
    if (_marblesToGo > 0) {
      _marblesToGo --;
      // Call doMarble to bypass the busy() check
      doMarble(_savedHopper, _savedChute);
    }
    break;
  case mdsZeroing:
    // should never get here.
    errorCondition("Couldn't get to the beginning of the track... is the motor working/powered?");
    break;
  default:
    break;
  }
}

// The bot needs to drop the rocker.  Check if we're currently over any hoppers, and move
// to a clear patch if we are.
void MarbleDeliverer::gotoSafeDropPoint() {
    // Check if we're overlapping with any hoppers before putting the rocker back down
    int overlap = -1;
    int rockerleft, rockerright;
    rockerleft = _stepMotor->pos() - ROCKER_WIDTH / 2;
    rockerright = _stepMotor->pos() + ROCKER_WIDTH / 2;
    for (int i = 0; i < HOPPER_COUNT; i ++) {
      if (_hopperSteps[i] == -1)
	continue;
      int hopperleft = _hopperSteps[i] - HOPPER_WIDTH / 2;
      int hopperright = _hopperSteps[i] + HOPPER_WIDTH / 2;

      if (!(rockerright < hopperleft || rockerleft > hopperright)) {
	overlap = i;
	break;
      }
    }
    int destination = _stepMotor->pos();
    if (overlap != -1) {
      // Can be exact here because I've overcatered for the rocker width by about 10%.
      destination = (_hopperSteps[overlap] - HOPPER_WIDTH) - ROCKER_WIDTH / 2;
    }
    _stepMotor->gotoStep(destination);
}

void MarbleDeliverer::errorCondition(char* err) {
  _state = mdsError;
  Serial.print("ERROR: ");
  Serial.println(err);
  _errorCondition = (char*)malloc(strlen(err) + 1);
  strcpy(_errorCondition, err);
  digitalWrite(ERR_LED_PIN, HIGH);
}

  // Emergency stop the motor
void MarbleDeliverer::stop() {
  if (_state != mdsError) {
    _state = mdsReady;
    _marblesToGo = 0;
    _stepMotor->stop();
  }
}

void MarbleDeliverer::zeroPosition() {
  if (checkBusy()) {
    return;
  }

  _state = mdsZeroing;

  if (_trackBeginSwitch->state() == LOW) {
    // The switch is closed, so the car is already at the beginning of the track.
    // We need to move forward until it opens again.
    _stepMotor->zero();
    // Go to an arbitrarily high step; ie. just go forward until the switch opens
    _stepMotor->gotoStep(10000); 
  } else {
    // The switch is open, so the car is out on the track somewhere.
    // Goto to an arbitrarily huge negative number - we'll stop when 
    // the track begin switch triggers.
    _stepMotor->gotoStep(-10000);
  }
}
