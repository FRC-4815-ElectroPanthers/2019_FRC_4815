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

  elevatorEncoder.SetPositionConversionFactor(1.0);

  armSR.ConfigFactoryDefault();
  armSL.ConfigFactoryDefault();
  armER.ConfigFactoryDefault();
  armEL.ConfigFactoryDefault();

  int shoulderAbsPos = armSR.GetSelectedSensorPosition() & 0xFFF;
  int elbowAbsPos = armER.GetSelectedSensorPosition() & 0xFFF;

  armSR.SetSelectedSensorPosition(0);//shoulderAbsPos);
  armER.SetSelectedSensorPosition(elbowAbsPos);

  armSR.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);
  armER.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);

  armSR.ConfigFeedbackNotContinuous(true);
  armER.ConfigFeedbackNotContinuous(true);

  armSR.SetSensorPhase(false);
  armER.SetSensorPhase(false);

  armSR.ConfigNominalOutputForward(0);
	armSR.ConfigNominalOutputReverse(0);
  armSR.ConfigPeakOutputForward(1);
	armSR.ConfigPeakOutputReverse(-1);

  armER.ConfigNominalOutputForward(0);
	armER.ConfigNominalOutputReverse(0);
  armER.ConfigPeakOutputForward(1);
	armER.ConfigPeakOutputReverse(-1);

  armSR.Config_kF(0, 0.0);
	armSR.Config_kP(0, 32);
	armSR.Config_kI(0, 0.00032);
	armSR.Config_kD(0, 1023);

  armER.Config_kF(0, 0.0);
	armER.Config_kP(0, 8);
	armER.Config_kI(0, 0.0024);
	armER.Config_kD(0, 100);

  armSL.Follow(armSR);
  armEL.Follow(armER);

  armSR.SetInverted(false);
  armSL.SetInverted(InvertType::OpposeMaster);
  armER.SetInverted(false);
  armEL.SetInverted(InvertType::OpposeMaster);

  camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  camera2 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
  // camera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  // camera2.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
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
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  prevPosShoulder = armSR.GetSelectedSensorPosition();
  prevPosElbow = armER.GetSelectedSensorPosition();
}

void Robot::TeleopPeriodic() {
  double targetPosShoulder; 
  double targetPosElbow; 

  double contAXR = Deadband(ControllerA.GetY(frc::GenericHID::kRightHand), 0.25);
  double contAXL = Deadband(ControllerA.GetY(frc::GenericHID::kLeftHand), 0.25);

  targetPosShoulder = 25*contAXR + prevPosShoulder;
  targetPosElbow = 12*contAXL + prevPosElbow;

  double bounds = cos(targetPosShoulder/4096*2*PI - targetPosElbow/4096*2*PI + 0.33)-cos(targetPosShoulder/4096*2*PI);
  if(!(-0.61 < bounds && bounds < 1.61)){
    targetPosShoulder = prevPosShoulder;
    targetPosElbow = prevPosElbow;
  }

  if(targetPosElbow > 0){
    targetPosElbow = 0;
  }  

  if(targetPosShoulder > 0){
    targetPosShoulder = 0;
  }
  
  mDrive.DriveCartesian(
    Controller.GetX(frc::GenericHID::kRightHand),
    -1*Controller.GetY(frc::GenericHID::kLeftHand),
    Controller.GetX(frc::GenericHID::kLeftHand)
  );

  armSR.Set(ControlMode::Position, targetPosShoulder);
  armER.Set(ControlMode::Position, targetPosElbow);
  
  VacuuMotorPivot.Set(0.5*(ControllerA.GetTriggerAxis(frc::GenericHID::kLeftHand) + -1*ControllerA.GetTriggerAxis(frc::GenericHID::kRightHand)));
  
  if (ControllerA.GetAButton()){
    VacuuMotor.Set(-1);
  }else {
    VacuuMotor.Set(0);
  }

  if(ControllerA.GetBumper(frc::GenericHID::kRightHand)){
    elevatorPID.SetReference(512, rev::kPosition);
  }

  std::cout << "Elevator position: " << elevatorEncoder.GetPosition() << std::endl;

  prevPosShoulder = targetPosShoulder;//armSR.GetSelectedSensorPosition();
  prevPosElbow = targetPosElbow;//armER.GetSelectedSensorPosition();
  
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
