/* =====================================================================================
 *  RedBot Buggy Controller
 *  
 *  Arduino program to control a SparkFun RedBot using simple serial commands. Supported
 *  commands are: 
 *  !Move LeftVelocity RightVelocity\r\n
 *     Moves the robot using the velocity to determine speed and direction the motors 
 *     turn. Positive values move the robot forward, negative values move it backwards.
 *     The maximum speed is 255. The motor is stopped when velocity is zero. 
 *  !Stop\r\n
 *     Stops both motors. 
 *     
 *  http://www.MegunoLink.com/articles/redbot-ui/ 
 * ===================================================================================== */
 
 
#include "CommandHandler.h"

// The serial command handler. Receives serial data and dispatches 
// recognised commands to functions registered during setup. 
// See: http://www.megunolink.com/documentation/arduino-libraries/serial-command-handler/
CommandHandler<> SerialCommandHandler;


// Setup the pin assignments for the Ardumotor board. 
const int Pin_PWM_Right = 3;
const int Pin_PWM_Left = 11;
const int Pin_Direction_Right = 12;
const int Pin_Direction_Left = 13;

void setup()
{
  Serial.begin(9600);
  Serial.println(F("Buggy Bot Controller"));
  Serial.println(F(__TIMESTAMP__));
  Serial.println(F("--------------------"));

  // Initialize IO and make sure the robot is stopped.
  pinMode(Pin_PWM_Left, OUTPUT);  
  pinMode(Pin_PWM_Right, OUTPUT);
  pinMode(Pin_Direction_Left, OUTPUT);
  pinMode(Pin_Direction_Right, OUTPUT);
  SetMotorSpeed(0,0);
  
  // Setup the command handler. 
  SerialCommandHandler.AddCommand(F("Move"), Cmd_Move);
  SerialCommandHandler.AddCommand(F("Stop"), Cmd_AllStop);
}

void loop()
{ 
  // Call the serial command handler's process function. It will receive
  // the serial data and call the registered function when a 
  // recognized command is received. 
  SerialCommandHandler.Process();
}

/* ---------------------------------------------------------------------------------
*  Serial command handlers
*  --------------------------------------------------------------------------------- */

void Cmd_Move(CommandParameter &p)
{
  int LeftSpeed = p.NextParameterAsInteger(0);
  int RightSpeed = p.NextParameterAsInteger(0);
  
  SetMotorSpeed(LeftSpeed, RightSpeed);
}

void Cmd_AllStop(CommandParameter &p)
{
  SetMotorSpeed(0,0);
}
   
/* ---------------------------------------------------------------------------------
*  Motor control
*  --------------------------------------------------------------------------------- */
void SetMotorSpeed(int LeftSpeed, int RightSpeed)
{
  // Print debug info.
  Serial.print(F("Speed: "));
  Serial.print(LeftSpeed);
  Serial.print(' ');
  Serial.println(RightSpeed);

  // Update motor speed
  SetMotorSpeed(Pin_Direction_Left, Pin_PWM_Left, LeftSpeed);
  SetMotorSpeed(Pin_Direction_Right, Pin_PWM_Right, RightSpeed);
}

// Change the speed of one motor. DirectionPin and SpeedPins are the Arduino
// pins that set direction and speed, respectively. If Speed is positive
// the motor turns forwards; if negative it will spin backwards. 
void SetMotorSpeed(int DirectionPin, int SpeedPin, int Speed)
{
  if (Speed > 0)
  {
    digitalWrite(DirectionPin, HIGH);
  }
  else
  {
    digitalWrite(DirectionPin, LOW);
    Speed = -Speed;
  }
  analogWrite(SpeedPin, Speed);
}

