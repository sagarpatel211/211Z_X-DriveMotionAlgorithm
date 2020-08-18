/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       sagarpatel                                                */
/*    Description:  X-Drive Motion Algorithm Using 3 PIDs                     */
/*    Credits:      Cody Nelson from 98881A for answering our questions       */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightFront           motor         18              
// RightBack            motor         17              
// LeftFront            motor         20              
// LeftBack             motor         14              
// Controller1          controller                    
// MotorExample         motor         13              
// ---- END VEXCODE CONFIGURED DEVICES ----

/*-------------------------------Includes------------------------------------*/
#include "vex.h"
#include <math.h>
#include <iostream> 
using namespace vex;
vex::competition    Competition;
/*-------------------------------Variables-----------------------------------*/
//Variables are defined here for use in the Odometry calculations
#define Pi 3.14159265358979323846
#define SL 5 //distance from tracking center to middle of left wheel
#define SR 5 //distance from tracking center to middle of right wheel
#define SS 7.75 //distance from tracking center to middle of the tracking wheel
#define WheelDiam 4.125 //diameter of all the wheels being used for tracking
#define tpr 360  //Degrees per single encoder rotation
double DeltaL,DeltaR,DeltaB,currentL,currentR,currentB,PreviousL,PreviousR,PreviousB,DeltaTheta;
double X,Y,Theta,DeltaXSide,DeltaYSide,DeltaXBack,DeltaYBack,SideChord,BackChord,OdomHeading;
//Motion Variables below
double AngleError,AngleDerivative, AnglePreviousError,AngleIntegral,AngleCount,AnglemVrequest,XmVrequest,XError,
XDerivative,XPreviousError,XIntegral,XCount,YmVrequest,YError,YDerivative,YPreviousError,YIntegral,YCount;
/*---------------------------------------------------------------------------*/
/*                              Motion Functions                             */
/*---------------------------------------------------------------------------*/
//Odometry Function/////////////////////////////////////////////////////////////////////////////////////////
void TrackPOS() {
// 2 cases could be occuring in odometry
// 1: Going in a straight line
// 2: Going in an arc motion
// If the bot is on an angle and going straight the displacement would be linear at angle Theta, meaning a right triangle is formed (Trig ratios to calc movement)
// Since it is a linear motion, the Left and right will move the same amount so we can just pick a side and do our movement calculation
// Since this calculation is working based of very infinitely small arcs, the displacement of the robot will be a chord
// Below it Averages the Left and Right integrated motor encoders since we don't have encoders yet
  currentR = (RightFront.position(degrees) + RightBack.position(degrees)) / 2;
  currentL = (LeftFront.position(degrees) + LeftBack.position(degrees)) / 2;

  //Creates variables for change in each side info in inches (12.9590697 is circumference of wheel)
  DeltaL = ((currentL - PreviousL) * 12.9590697) / tpr;
  DeltaR = ((currentR - PreviousR) * 12.9590697) / tpr;
  //DeltaB = ((currentB - PreviousB) * 12.9590697) / tpr;

  //Determines the change in angle of the robot using the rotational change in each side
  DeltaTheta = (DeltaR - DeltaL) / (SL + SR);

  //Creates an if/else statement to prevent NaN values from appearing and causing issues with calculation
  if(DeltaTheta == 0) {  //If there is no change in angle
    X += DeltaL * sin (Theta);
    Y += DeltaL * cos (Theta);
    //X += DeltaB * cos (Theta + 1.57079633);
    //Y += DeltaB * sin (Theta + 1.57079633);

  //If there is a change in angle, it will calculate the changes in X,Y from chords of an arc/circle.
  } else {  //If the angle changes
      SideChord = 2 * ((DeltaL / DeltaTheta) + SL) * sin (DeltaTheta / 2);
      //BackChord = 2 * ((DeltaB / DeltaTheta) + SS) * sin (DeltaTheta / 2);
      DeltaYSide = SideChord * cos (Theta + (DeltaTheta / 2));
      DeltaXSide = SideChord * sin (Theta + (DeltaTheta / 2));
      //DeltaXBack = BackChord * sin (Theta + (DeltaTheta / 2));
      //DeltaYBack = -BackChord * cos (Theta + (DeltaTheta / 2));
      Theta += DeltaTheta;
      X += DeltaXSide;
      Y += DeltaYSide;
    }

    //Odom heading is converting the radian value of Theta into degrees
    OdomHeading = Theta * 57.295779513;

    //Converts values into newer values to allow for code to effectively work in next cycle
    PreviousL = currentL;
    PreviousR = currentR;
    DeltaTheta = 0;

    //This is for printing to the brain for debugging
    Brain.Screen.printAt(100,20, "X: %f",X);
    Brain.Screen.printAt(100,40, "Y: %f",Y);
    Brain.Screen.printAt(100,60, "Theta: %f",Theta);
    Brain.Screen.printAt(100,80, "Angle: %f",OdomHeading);
    Brain.Screen.printAt(100,100, "Displacement1: %f",SideChord);
    Brain.Screen.printAt(100,120, "DeltaLeftInches: %f",DeltaL);
    Brain.Screen.printAt(100,140, "DeltaRightInches: %f",DeltaR);
    Brain.Screen.printAt(100,160, "DeltaX: %f",DeltaXSide);
    Brain.Screen.printAt(100,180, "DeltaY: %f",DeltaYSide);
}

//PID #1 - X Displacement PID//////////////////////////////////////////////////////////////////////////////////
void XdisplacementPID(double desiredX, double XkP, double XkI, double XkD){
  XError = desiredX - X; //X error is calculated
  XDerivative = XError - XPreviousError; //Derivative is calculated from error - previous error
  if ((XError < 0.07) && (XError > -0.07)){ //if the error is really small (0.07 inches)...
    XIntegral = 0; //Keep integral at 0
     XCount += 1; //add 1 to count
  } else { 
    XIntegral += XError; //error is added to integral
    XCount = 0; //the count remains 0 if error is significant
  }
  if (XCount >= 10) { //if count reaches 10...
    XmVrequest = 0; //set voltage of x displacement control to 0
    return; //break from function
  }
  XmVrequest = XError * XkP + XDerivative * XkD + XIntegral * XkI; //X voltage is calculated
  XPreviousError = XError; //previous error is calculated from error
}

//PID #2 - Y Displacement PID//////////////////////////////////////////////////////////////////////////////////
void YdisplacementPID(double desiredY, double YkP, double YkI, double YkD){
  YError = desiredY - Y; //X error is calculated
  YDerivative = YError - YPreviousError; //Derivative is calculated from error - previous error
  if ((YError < 0.07) && (YError > -0.07)){ //if the error is really small (0.07 inches)...
    YIntegral = 0; //Keep integral at 0
     YCount += 1; //add 1 to count
  } else { 
    YIntegral += YError; //error is added to integral
    YCount = 0; //the count remains 0 if error is significant
  }
  if (YCount >= 10) { //if count reaches 10...
    YmVrequest = 0; //set voltage of y displacement control to 0
    return; //break from function
  }
  YmVrequest = YError * YkP + YDerivative * YkD + YIntegral * YkI;  //Y voltage is calculated
  YPreviousError = YError; //previous error is calculated from error
}

//PID #3 - Angle PID///////////////////////////////////////////////////////////////////////////////////////////
void AngledisplacementPID(double desiredangle, double anglekP, double anglekI, double anglekD){
  AngleError = desiredangle - OdomHeading; //angle error is calculated
  AngleDerivative = AngleError - AnglePreviousError;  //Derivative is calculated from error - previous error
  if ((AngleError < 0.1) && (AngleError > -0.1)){  //if the error is really small (0.1 degrees)...
      AngleIntegral = 0;  //Keep integral at 0
      AngleCount += 1; //add 1 to count
    }
    else { 
    AngleIntegral += AngleError; //error is added to integral
    AngleCount = 0;  //the count remains 0 if error is significant
  }
  if (AngleCount >= 10) { //if count reaches 10...
    AnglemVrequest = 0; //set voltage of angle displacement control to 0
    return; //break from function
  }
  AnglemVrequest = AngleError * anglekP + AngleDerivative * anglekD + AngleIntegral * anglekI;//angle voltage is calculated
  AnglePreviousError = AngleError; //previous error is calculated from error
}

//Combined PID/////////////////////////////////////////////////////////////////////////////////////////////////
void MoveToPos(double Xvalue, double Yvalue, double Angle, double kPx, double kIx, double kDx,
  double kPy, double kIy,double kDy,double kPa, double kIa,double kDa ){
  while (Xvalue != X && Yvalue != Y && Angle != OdomHeading) //if the robot reaches position, break loop. else...
    TrackPOS(); //run odometry function
    XdisplacementPID (Xvalue, kPx, kIx, kDx); //call the x displacement function
    YdisplacementPID (Yvalue, kPy, kIy, kDy); //call the y displacement function
    AngledisplacementPID (Angle, kPa, kIa, kDa); //call the angle displacement function
    if (AngleCount >= 10 && YCount >= 10 && XCount >= 10){ //if all count variables are 10 more more...
      return; //break from function
    }
    LeftFrontValue = XmVrequest + YmVrequest - AnglemVrequest;
    LeftBackValue = XmVrequest + YmVrequest + AnglemVrequest;
    RightFrontValue = XmVrequest - YmVrequest + AnglemVrequest;
    RightBackValue = XmVrequest - YmVrequest - AnglemVrequest;
    MaxVoltage = 12000; //this is the highest valid value for the spin function in millivolts
    if (fabs(LeftFrontValue) > MaxVoltage || fabs(LeftBackValue) > MaxVoltage || fabs(RightFrontValue) > MaxVoltage || fabs(RightBackValue) > MaxVoltage){
      if (LeftFrontValue>=LeftBackValue && LeftFrontValue>=RightFrontValue && LeftFrontValue>=RightBackValue) {
        MaxValue = LeftFrontValue / MaxVoltage;
      }
      if (LeftBackValue>=LeftFrontValue && LeftBackValue>=RightFrontValue && LeftBackValue>=RightBackValue) {
        MaxValue = LeftBackValue / MaxVoltage;
      }
      if (RightFrontValue>=LeftFrontValue && RightFrontValue>=LeftBackValue && RightFrontValue>=RightBackValue) {
        MaxValue = RightFrontValue / MaxVoltage;
      }
      if (RightBackValue>=LeftFrontValue && RightBackValue>=LeftBackValue && RightBackValue>=RightFrontValue) {
        MaxValue = RightBackValue / MaxVoltage;
      }
      LeftFrontValue /= MaxValue;
      LeftBackValue /= MaxValue;
      RightFrontValue /= MaxValue;
      RightBackValue /= MaxValue;
    }
    LeftFront.spin(forward, LeftFrontValue, voltageUnits::mV); //Motion
    LeftBack.spin(forward, LeftBackValue, voltageUnits::mV);  //Motion
    RightFront.spin(forward, RightFrontValue, voltageUnits::mV);//Motion
    RightBack.spin(forward, RightBackValue, voltageUnits::mV); //Motion
    vex::task::sleep(5); //Slight delay so the Brain doesn't overprocess
}
void VariableReset ( void ){  //This resets all the variables for the PID functions
  AngleCount = 0;
  YCount = 0;
  XCount = 0;
  XmVrequest = 0;
  YmVrequest = 0;
  AnglemVrequest = 0;
  AnglePreviousError = 0;
  YPreviousError = 0;
  XPreviousError = 0;
  AngleIntegral = 0;
  AngleError = 0;
  AngleDerivative = 0;
  XIntegral = 0;
  XError = 0;
  XDerivative = 0;
  YIntegral = 0;
  YError = 0;
  YDerivative = 0;
}
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
void pre_auton( void ) {
  vexcodeInit(); //Initializing Robot Configuration - Required!!!
  Brain.resetTimer(); //Resets The Brain Timer
  RightFront.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  RightBack.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  LeftFront.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  LeftBack.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void autonomous( void ) {
  //MoveToPos (X, Y, angle, PID values for x, PID values for y, PID values for angle)
  MotorExample.spin(forward,100,velocityUnits::pct);//sets an example motor to 100 to see if it will run while moving
  MoveToPos(10, 10, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0); //Example: X, Y = 10 and angle = 90
  VariableReset(); //Resets all values so the next function can run (just in case!!)
  MotorExample.stop(brakeType::brake);  //sets to motor to 0 to stop it
  MoveToPos(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); //Example: X, Y, and angle = 0
}
/*----------------------------------------------------------------------------*/
/*                              User Control Task                             */
/*----------------------------------------------------------------------------*/
void usercontrol( void ) {
  while (1){
    Brain.Screen.clearScreen(); //clears the screen to continuously display the odometry info
    //provides power to the motors to allow for movement of robot for testing using controller
    LeftBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value()) + (Controller1.Axis4.value()) - (Controller1.Axis1.value())), vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value()) + (Controller1.Axis4.value()) + (Controller1.Axis1.value())), vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value()) - (Controller1.Axis4.value()) + (Controller1.Axis1.value())), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value()) - (Controller1.Axis4.value()) - (Controller1.Axis1.value())), vex::velocityUnits::pct);
    TrackPOS(); //Calls the TrackPosition function
    Brain.Screen.render(); //push data to the LCD all at once to prevent image flickering
    vex::task::sleep(10); //Slight delay so the Brain doesn't overprocess
  }
}
int main() {
    pre_auton(); //Calls the pre-autonomous function
    Competition.autonomous( autonomous ); //Calls the autonomous function
    Competition.drivercontrol( usercontrol ); //Calls the driver control function
    while(1) {
      vex::task::sleep(15); //Slight delay so the Brain doesn't overprocess
    }
}
