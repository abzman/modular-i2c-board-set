
//uint16_t        temp;

/// Symbolic names for the direction the motor is turning
typedef enum
{
  DIRECTION_CCW = 1,  ///< Counter-Clockwise
  DIRECTION_CW  = 0   ///< Clockwise
} Direction;

/// Current direction motor is spinning in
/// Protected because some peoples subclasses need it to be so
boolean _direction[2]; // 1 == CW
boolean _directionActual[2]; // last asserted sirection
/// The current interval between steps in microseconds.
/// 0 means the motor is currently stopped with _speed == 0
unsigned long  _stepInterval[2];

/// Arduino pin number assignments for the 2 pins required to interface to the
/// stepper motor driver
uint8_t        _pin[2][2];
/// The current absolution position in steps.
long           _currentPos[2];    // Steps
/// The target position in steps. The AccelStepper library will move the
/// motor from the _currentPos to the _targetPos, taking into account the
/// max speed, acceleration and deceleration
long           _targetPos[2];     // Steps
/// The current motos speed in steps per second
/// Positive is clockwise
float          _speed[2];         // Steps per second
/// The maximum permitted speed in steps per second. Must be > 0.
float          _maxSpeed[2];

/// The acceleration to use to accelerate or decelerate the motor in steps
/// per second per second. Must be > 0
float          _acceleration[2];
float          _sqrt_twoa[2]; // Precomputed sqrt(2*_acceleration)
/// The last step time in microseconds
unsigned long  _lastStepTime[2];


/// The step counter for speed calculations
long _n[2];

/// Initial step size in microseconds
float _c0[2];

/// Last step size in microseconds
float _cn[2];
/// Min step size in microseconds based on maxSpeed
float _cmin[2]; // at max speed

void moveTo(StepMotor motor, long absolute)
{
  if (_targetPos[motor] != absolute)
  {
    _targetPos[motor] = absolute;
    computeNewSpeed(motor);
    // compute new n?
  }
}

void move(StepMotor motor, long relative)
{
  moveTo(motor, _currentPos[motor] + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean runSpeed(StepMotor motor)
{
  // Dont do anything unless we actually have a step interval
  if (!_stepInterval[motor])
    return false;

  unsigned long time = micros();
  if (time - _lastStepTime[motor] >= _stepInterval[motor])
  {
    if (_direction[motor] == DIRECTION_CW)
    {
      // Clockwise
      _currentPos[motor] += 1;
    }
    else
    {
      // Anticlockwise
      _currentPos[motor] -= 1;
    }
    step(motor);

    _lastStepTime[motor] = time; // Caution: does not account for costs in step()

    return true;
  }
  else
  {
    return false;
  }
}

long distanceToGo(StepMotor motor)
{
  return _targetPos[motor] - _currentPos[motor];
}

long targetPosition(StepMotor motor)
{
  return _targetPos[motor];
}

boolean motorDirection(StepMotor motor)
{
  return _direction[motor];
}


long currentPosition(StepMotor motor)
{
  return _currentPos[motor];
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void setCurrentPosition(StepMotor motor, long position)
{
  _targetPos[motor] = _currentPos[motor] = position;
  _n[motor] = 0;
  _stepInterval[motor] = 0;
  _speed[motor] = 0.0;
}

unsigned long computeNewSpeed(StepMotor motor)
{
  long distanceTo = distanceToGo(motor); // +ve is clockwise from curent location

  long stepsToStop = (long)((_speed[motor] * _speed[motor]) / (2.0 * _acceleration[motor])); // Equation 16

  if (distanceTo == 0 && stepsToStop <= 1)
  {
    // We are at the target and its time to stop
    _stepInterval[motor] = 0;
    _speed[motor] = 0.0;
    _n[motor] = 0;
    return _stepInterval[motor];
  }

  if (distanceTo > 0)
  {
    // We are anticlockwise from the target
    // Need to go clockwise from here, maybe decelerate now
    if (_n[motor] > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((stepsToStop >= distanceTo) || _direction[motor] == DIRECTION_CCW)
        _n[motor] = -stepsToStop; // Start deceleration
    }
    else if (_n[motor] < 0)
    {
      // Currently decelerating, need to accel again?
      if ((stepsToStop < distanceTo) && _direction[motor] == DIRECTION_CW)
        _n[motor] = -_n[motor]; // Start accceleration
    }
  }
  else if (distanceTo < 0)
  {
    // We are clockwise from the target
    // Need to go anticlockwise from here, maybe decelerate
    if (_n[motor] > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((stepsToStop >= -distanceTo) || _direction[motor] == DIRECTION_CW)
        _n[motor] = -stepsToStop; // Start deceleration
    }
    else if (_n[motor] < 0)
    {
      // Currently decelerating, need to accel again?
      if ((stepsToStop < -distanceTo) && _direction[motor] == DIRECTION_CCW)
        _n[motor] = -_n[motor]; // Start accceleration
    }
  }

  // Need to accelerate or decelerate
  if (_n[motor] == 0)
  {
    // First step from stopped
    _cn[motor] = _c0[motor];
    _direction[motor] = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
  }
  else
  {
    // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
    _cn[motor] = _cn[motor] - ((2.0 * _cn[motor]) / ((4.0 * _n[motor]) + 1)); // Equation 13
    _cn[motor] = max(_cn[motor], _cmin[motor]);
  }
  _n[motor]++;
  _stepInterval[motor] = _cn[motor];
  _speed[motor] = 1000000.0 / _cn[motor];
  if (_direction[motor] == DIRECTION_CCW)
    _speed[motor] = -_speed[motor];

#if 0
  Serial.println(_speed[motor]);
  Serial.println(_acceleration[motor]);
  Serial.println(_cn[motor]);
  Serial.println(_c0[motor]);
  Serial.println(_n[motor]);
  Serial.println(_stepInterval[motor]);
  Serial.println(distanceTo);
  Serial.println(stepsToStop);
  Serial.println("-----");
#endif
  return _stepInterval[motor];
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
boolean run(StepMotor motor)
{
  if (runSpeed(motor))
    computeNewSpeed(motor);
  return _speed[motor] != 0.0 || distanceToGo(motor) != 0;
}

void AccelStepper(StepMotor motor, uint8_t pin1, uint8_t pin2) //pin1 is step pin2 is direction
{
  _currentPos[motor] = 0;
  _targetPos[motor] = 0;
  _speed[motor] = 0.0;
  _maxSpeed[motor] = 0.0;
  _acceleration[motor] = 0.0;
  _sqrt_twoa[motor] = 1.0;
  _stepInterval[motor] = 0;
  _lastStepTime[motor] = 0;
  _pin[motor][0] = pin1;
  _pin[motor][1] = pin2;

  // NEW
  _n[motor] = 0;
  _c0[motor] = 0.0;
  _cn[motor] = 0.0;
  _cmin[motor] = 1.0;
  _direction[motor] = DIRECTION_CCW;
  _directionActual[motor] = _direction[motor];
  digitalWrite24v(_pin[motor][1], 0);

  // Some reasonable default
  setAcceleration(motor, 1);
  setMaxSpeed(motor, 1);
}

void setMaxSpeed(StepMotor motor, float speed)
{
  if (speed < 0.0)
    speed = -speed;
  if (_maxSpeed[motor] != speed)
  {
    _maxSpeed[motor] = speed;
    _cmin[motor] = 1000000.0 / speed;
    // Recompute _n from current speed and adjust speed if accelerating or cruising
    if (_n[motor] > 0)
    {
      _n[motor] = (long)((_speed[motor] * _speed[motor]) / (2.0 * _acceleration[motor])); // Equation 16
      computeNewSpeed(motor);
    }
  }
}

float   maxSpeed(StepMotor motor)
{
  return _maxSpeed[motor];
}

void setAcceleration(StepMotor motor, float acceleration)
{
  if (acceleration == 0.0)
    return;
  if (acceleration < 0.0)
    acceleration = -acceleration;
  if (_acceleration[motor] != acceleration)
  {
    // Recompute _n per Equation 17
    _n[motor] = _n[motor] * (_acceleration[motor] / acceleration);
    // New c0 per Equation 7, with correction per Equation 15
    _c0[motor] = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
    _acceleration[motor] = acceleration;
    computeNewSpeed(motor);
  }
}

float   acceleration(StepMotor motor)
{
  return _acceleration[motor];
}

void setSpeed(StepMotor motor, float speed)
{
  if (speed == _speed[motor])
    return;
  speed = constrain(speed, -_maxSpeed[motor], _maxSpeed[motor]);
  if (speed == 0.0)
    _stepInterval[motor] = 0;
  else
  {
    _stepInterval[motor] = fabs(1000000.0 / speed);
    _direction[motor] = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
  }
  _speed[motor] = speed;
}

float speed(StepMotor motor)
{
  return _speed[motor];
}

// 1 pin step function (ie for stepper drivers)
void step(StepMotor motor)
{
  //_pin[0] is step, _pin[1] is direction
  if (_direction[motor] != _directionActual[motor])
  {
    _directionActual[motor] = _direction[motor];
    //temp = 0;

    //Serial.print("direction");
    //Serial.println(_direction[motor]);
    if (_direction[motor] == DIRECTION_CW) // Set direction first else get rogue pulses
    {
      digitalWrite24v(_pin[motor][1], 1);
    }
    else
    {
      digitalWrite24v(_pin[motor][1], 0);
    }

    delayMicroseconds(5);
  }
  //temp = temp + 1;
  //Serial.println(temp);
  digitalWrite(_pin[motor][0], 1); // step HIGH
  // Caution 200ns setup time
  // Delay the minimum allowed pulse width
  delayMicroseconds(3);
  digitalWrite(_pin[motor][0], 0); // step LOW
}

// Blocks until the target position is reached and stopped
void runToPosition(StepMotor motor)
{
  while (run(motor))
    YIELD; // Let system housekeeping occur
}

boolean runSpeedToPosition(StepMotor motor)
{
  if (_targetPos[motor] == _currentPos[motor])
    return false;
  if (_targetPos[motor] > _currentPos[motor])
    _direction[motor] = DIRECTION_CW;
  else
    _direction[motor] = DIRECTION_CCW;
  return runSpeed(motor);
}

// Blocks until the new target position is reached
void runToNewPosition(StepMotor motor, long position)
{
  moveTo(motor, position);
  runToPosition(motor);
}

void stop(StepMotor motor)
{
  if (_speed[motor] != 0.0)
  {
    long stepsToStop = (long)((_speed[motor] * _speed[motor]) / (2.0 * _acceleration[motor])) + 1; // Equation 16 (+integer rounding)
    if (_speed[motor] > 0)
      move(motor, stepsToStop);
    else
      move(motor, -stepsToStop);
  }
}

//hack to stop without decelerationg
void eStop(StepMotor motor)
{
  //disableAxis(motor);
  _speed[motor] = 0.0;
  _targetPos[motor] = _currentPos[motor];
}

bool isRunning(StepMotor motor)
{
  return !(_speed[motor] == 0.0 && _targetPos[motor] == _currentPos[motor]);
}
