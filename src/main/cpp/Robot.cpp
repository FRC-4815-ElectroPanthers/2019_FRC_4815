/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#define PI 3.14159265358979323

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  mDrive.SetRightSideInverted(false);

  elevator.RestoreFactoryDefaults();
  elevator.SetInverted(false);

  elevatorPID.SetP(0.0);
  elevatorPID.SetI(0.0);
  elevatorPID.SetD(0.0);
  elevatorPID.SetFF(0.0);
  //elevatorPID.SetIMaxAccum();

  elevatorEncoder.SetPositionConversionFactor(2.26/460);

  gripperR.ConfigFactoryDefault();
  gripperL.ConfigFactoryDefault();

  int shoulderAbsPos = gripperR.GetSelectedSensorPosition() & 0xFFF;

  gripperR.SetSelectedSensorPosition(0);//shoulderAbsPos);

  gripperR.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);

  gripperR.ConfigFeedbackNotContinuous(true);

  gripperR.SetSensorPhase(false);

  gripperR.ConfigNominalOutputForward(0);
	gripperR.ConfigNominalOutputReverse(0);
  gripperR.ConfigPeakOutputForward(1);
	gripperR.ConfigPeakOutputReverse(-1);

  gripperR.Config_kF(0, 0.0);
	gripperR.Config_kP(0, 32);
	gripperR.Config_kI(0, 0.00032);
	gripperR.Config_kD(0, 1023);

  gripperL.Follow(gripperR);

  gripperR.SetInverted(false);
  gripperL.SetInverted(InvertType::OpposeMaster);

  camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  camera2 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
  
  frc::SmartDashboard::PutNumber("P Gain", kP);
  frc::SmartDashboard::PutNumber("I Gain", kI);
  frc::SmartDashboard::PutNumber("D Gain", kD);
  frc::SmartDashboard::PutNumber("I Zone", kIz);
  frc::SmartDashboard::PutNumber("Feed Forward", kFF);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  prevElevatorPos = elevatorEncoder.GetPosition();
}

void Robot::AutonomousPeriodic() {
  double targetElevatorPos;

  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);

  if((p != kP)) { elevatorPID.SetP(p); kP = p; }
  if((i != kI)) { elevatorPID.SetI(i); kI = i; }
  if((d != kD)) { elevatorPID.SetD(d); kD = d; }
  if((iz != kIz)) { elevatorPID.SetIZone(iz); kIz = iz; }
  if((ff != kFF)) { elevatorPID.SetFF(ff); kFF = ff; }

  double contAXR = Deadband(ControllerA.GetY(frc::GenericHID::kRightHand), 0.25);
  double contAXL = Deadband(ControllerA.GetY(frc::GenericHID::kLeftHand), 0.25);

  targetElevatorPos = 25*contAXR + prevElevatorPos;

  //put limit switch code here
  if(targetElevatorPos > 0){
    targetElevatorPos = 0;
  }
  
  mDrive.DriveCartesian(
    Controller.GetX(frc::GenericHID::kRightHand),
    -1*Controller.GetY(frc::GenericHID::kLeftHand),
    Controller.GetX(frc::GenericHID::kLeftHand)
  );
  
  if(ControllerA.GetBumper(frc::GenericHID::kRightHand)){
    elevatorPID.SetReference(2, rev::kPosition);
  }else{
    elevator.Set(contAXR);
  }
  
  // if(ControllerA.GetAButton()){
  //   elevatorPID.SetReference(elevatorLvl1);
  // }else if(ControllerA.GetBButton()){
  //   elevatorPID.SetReference(elevatorLvl2);
  // }else if(ControllerA.GetXButton()){
  //   elevatorPID.SetReference(elevatorLvl3);
  // }else if(ControllerA.GetYButton()){
  //   elevatorPID.SetReference(elevator0);
  // }else{
  //   elevatorPID.SetReference(targetElevatorPos, rev::kPosition);
  // }

  gripperR.Set(ControlMode::Percent, contAXL);
  // if(ControllerA.GetBumper(frc::GenericHID::kLeftHand)){
  //   gripperR.Set(ControlMode::Position, gripperHatch);
  // }else if(ControllerA.GetBumper(frc::GenericHID::kRightHand)){
  //   gripperR.Set(ControlMode::Position, gripperBall);
  // }else{
  //   gripperR.Set(ControlMode::Position, gripperClose);
  // }

  //std::cout << "Elevator position: " << elevatorEncoder.GetPosition() << std::endl;
  frc::SmartDashboard::PutNumber("Elevator position: ", elevatorEncoder.GetPosition());

  prevElevatorPos = targetPosShoulder;
}

void Robot::TeleopInit() {
  prevElevatorPos = elevatorEncoder.GetPosition();
}

void Robot::TeleopPeriodic() {
  double targetElevatorPos;

  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);

  if((p != kP)) { elevatorPID.SetP(p); kP = p; }
  if((i != kI)) { elevatorPID.SetI(i); kI = i; }
  if((d != kD)) { elevatorPID.SetD(d); kD = d; }
  if((iz != kIz)) { elevatorPID.SetIZone(iz); kIz = iz; }
  if((ff != kFF)) { elevatorPID.SetFF(ff); kFF = ff; }

  double contAXR = Deadband(ControllerA.GetY(frc::GenericHID::kRightHand), 0.25);
  double contAXL = Deadband(ControllerA.GetY(frc::GenericHID::kLeftHand), 0.25);

  targetElevatorPos = 25*contAXR + prevElevatorPos;

  //put limit switch code here
  if(targetElevatorPos > 0){
    targetElevatorPos = 0;
  }
  
  mDrive.DriveCartesian(
    Controller.GetX(frc::GenericHID::kRightHand),
    -1*Controller.GetY(frc::GenericHID::kLeftHand),
    Controller.GetX(frc::GenericHID::kLeftHand)
  );
  
  if(ControllerA.GetBumper(frc::GenericHID::kRightHand)){
    elevatorPID.SetReference(2, rev::kPosition);
  }else{
    elevator.Set(contAXR);
  }
  
  // if(ControllerA.GetAButton()){
  //   elevatorPID.SetReference(elevatorLvl1);
  // }else if(ControllerA.GetBButton()){
  //   elevatorPID.SetReference(elevatorLvl2);
  // }else if(ControllerA.GetXButton()){
  //   elevatorPID.SetReference(elevatorLvl3);
  // }else if(ControllerA.GetYButton()){
  //   elevatorPID.SetReference(elevator0);
  // }else{
  //   elevatorPID.SetReference(targetElevatorPos, rev::kPosition);
  // }

  gripperR.Set(ControlMode::Percent, contAXL);
  // if(ControllerA.GetBumper(frc::GenericHID::kLeftHand)){
  //   gripperR.Set(ControlMode::Position, gripperHatch);
  // }else if(ControllerA.GetBumper(frc::GenericHID::kRightHand)){
  //   gripperR.Set(ControlMode::Position, gripperBall);
  // }else{
  //   gripperR.Set(ControlMode::Position, gripperClose);
  // }

  //std::cout << "Elevator position: " << elevatorEncoder.GetPosition() << std::endl;
  frc::SmartDashboard::PutNumber("Elevator position: ", elevatorEncoder.GetPosition());

  prevElevatorPos = targetPosShoulder;
}

void Robot::TestPeriodic() {}

double Robot::Deadband(double in, double dis){
  double out = in;

  if(out < dis && out > -1*dis){
    out = 0;
  }

  return out;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
