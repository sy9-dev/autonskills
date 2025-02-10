/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 10, D        
// ClawMotor            motor         3               
// ArmMotor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition Competition;

motor bottomleft = motor(PORT13, ratio18_1, true);
motor bottomright = motor(PORT14, ratio18_1, false);
motor topleft = motor(PORT11, ratio18_1, true);
motor topright = motor(PORT12, ratio18_1, false);

motor intake1 = motor(PORT15, ratio18_1, false);
motor intake2 = motor(PORT16, ratio18_1, true);
motor intake3 = motor(PORT17, ratio18_1, true);

motor wall = motor(PORT1, ratio18_1, true);

controller Controller = controller(primary);

motor_group leftmotors = motor_group(topleft, bottomleft);
motor_group rightmotors = motor_group(topright, bottomright);
motor_group intake = motor_group(intake1,intake2,intake3);

drivetrain dt = drivetrain(leftmotors, rightmotors, 300, 320, 320, mm, 1);

digital_out* clinch;

bool tank = false;
double leftvalue;
double rightvalue;

void switchtype() {
  tank = !tank;
}

void useClinch() {
  clinch->set(!clinch->value());
}

void spinIntake(){
  intake.spin(forward);
}

void reverseIntake(){
  intake.spin(reverse);
}

void stopIntake(){
  intake.stop();
}

void wallIn(){
  wall.spinToPosition(0,degrees);
}

void wallOut(){
  wall.spin(forward);
}

void wallStop(){
  wall.stop();
}

void pre_auton(void) {
  vexcodeInit();

  leftmotors.setStopping(brake);
  rightmotors.setStopping(brake);
  wall.setStopping(brake);
  wall.setBrake(hold);
  wall.setPosition(0,degrees);

  intake.setVelocity(100,percent);

  clinch->set(true);

  Controller.ButtonLeft.pressed(switchtype);
  Controller.ButtonL1.pressed(useClinch);

  Controller.ButtonR1.pressed(spinIntake);
  Controller.ButtonR1.released(stopIntake);
  Controller.ButtonR2.pressed(reverseIntake);
  Controller.ButtonR2.released(stopIntake);

  Controller.ButtonX.pressed(wallOut);
  Controller.ButtonX.released(wallStop);
  Controller.ButtonB.released(wallIn);
}

void autonomous(void) {
  dt.setDriveVelocity(65,percent);
  dt.setTurnVelocity(40,percent);
  intake.setVelocity(100,percent);

  
  //put the ring on the wall stake
  wall.spinFor(forward,1.5,seconds);

  //get the clinch
  dt.driveFor(reverse,7,inches);
  wait(.5,sec);
  dt.turnFor(-33,degrees);
  dt.driveFor(reverse,30,inches,false);
  wait(0.65,sec);
  useClinch();
  wait(0.2,sec);

  //get the first ring
  dt.turnFor(-62,degrees);
  intake.spin(forward);
  dt.driveFor(forward,8,inches);
  wait(2,sec);

  //get the second ring
  dt.driveFor(forward,12,inches);
  wait(2.5,sec);
  
  //put into positive corner
  dt.turnFor(45,degrees);
  dt.driveFor(reverse,3,inches);
  wait(.5,sec);
  useClinch();


  //Get MoGo 2
  dt.turnFor(33,degrees);
  dt.driveFor(reverse,42,inches,false);
  wait(5,sec);
  useClinch();

  //Get Ring 2.1
  dt.turnFor(95,degrees);
  intake.spin(forward);
  dt.driveFor(forward,9,inches);
  wait(1,seconds);

  //Get Corner 2
  dt.turnFor(45,deg);
  dt.driveFor(reverse,10,inches);
  useClinch();

  //Get MoGo 3
  dt.turnFor(-45,degrees);
  dt.driveFor(forward,96,inches);
  dt.turnFor(-33,degrees);
  dt.driveFor(reverse,7,inches);
  useClinch();

  //Get Corner 3
  dt.turnFor(43,degrees);
  dt.driveFor(forward,11,inches);
  useClinch();

  //Get Mogo 4
  dt.driveFor(reverse,11,inches);
  dt.turnFor(-43,degrees);
  dt.driveFor(reverse,36,inches);
  useClinch();

  //Get Corner 4
  dt.turnFor(33,degrees);
  dt.driveFor(reverse,11,inches);
  useClinch();


/*
  //Get Ring 2.2
  dt.driveFor(5,inches);
  wait(1,sec);

  //Get Ring 2.3
  dt.turnFor(25,degrees);
  dt.driveFor(7,inches);


/*
  //get the third ring
  dt.driveFor(reverse,8,inches);
  wait(2,sec);


  /*
  //Get Ring 3
  dt.driveFor(reverse,5,inches);
  dt.turnFor(36,degrees);
  dt.driveFor(forward,10,inches);
  wait(1,sec);

  //Get Ring 4
  dt.driveFor(forward,18,inches);
  dt.turnFor(-35,degrees);
  dt.driveFor(forward,6,inches);
  wait(1,sec);

 //Get Ring 5
 dt.driveFor(reverse,6,inches);
 dt.turnFor(-48,degrees);
dt.driveFor(forward,18,inches);
wait(1,sec);

//Get Ring 6
dt.turnFor(30,degrees);
dt.driveFor(forward,18,inches);
wait(1,sec);

//put into positive corner
dt.turnFor(60,degrees);
dt.driveFor(reverse,5,inches);
useClinch();

  wait(1,seconds);
  dt.turnFor(32,degrees);
  intake.spin(forward);
  dt.driveFor(forward,15,inches);
  wait(2,seconds);

  /*
  wall.spinFor(1,seconds);

  //get the stake
  dt.setDriveVelocity(20,percent);
  dt.setTurnVelocity(20,percent);
  dt.driveFor(reverse,5,inches);
  dt.turnFor(-20,degrees);
  dt.driveFor(reverse,10,inches);
  useClinch();

  //get the first ring
  wait(1,sec);
  intake.spin(forward);
  dt.turnFor(50,degrees);
  dt.driveFor(forward,12,inches);
  wait(1,sec);
  dt.driveFor(forward,7.5,inches);
  wait(3,sec);
  */
}

void usercontrol(void) {
  while (1) {
    if (tank) {
      bottomleft.setVelocity(Controller.Axis3.position(), percent);
      topleft.setVelocity(Controller.Axis3.position(), percent);
      topright.setVelocity(Controller.Axis2.position(), percent);
      bottomright.setVelocity(Controller.Axis2.position(), percent);
    } else {
      leftvalue = (Controller.Axis4.position() + Controller.Axis3.position());
      rightvalue = (Controller.Axis4.position() - Controller.Axis3.position()) * -1;
      bottomleft.setVelocity(leftvalue, percent);
      topleft.setVelocity(leftvalue, percent);
      topright.setVelocity(rightvalue, percent);
      bottomright.setVelocity(rightvalue, percent);
    }

    bottomleft.spin(forward);
    bottomright.spin(forward);
    topleft.spin(forward);
    topright.spin(forward);

    wait(20, msec); 
  }
}

int main() {
  digital_out c(Brain.ThreeWirePort.A);
  clinch = &c;

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}

