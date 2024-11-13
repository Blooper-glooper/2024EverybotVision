// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.distanceConstants;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  CANSparkMax frontLeftMotor;
  CANSparkMax backLeftMotor;
  
  CANSparkMax frontRightMotor;
  CANSparkMax backRightMotor;

  MotorControllerGroup leftSide ;
  MotorControllerGroup rightSide;

  DifferentialDrive differentialDrive;

  PIDController drivePIDController;
  PIDController turnPIDController;

 
  public Drivetrain() {
    frontLeftMotor = new CANSparkMax(Constants.DrivetrainConstants.kLeftFrontId, CANSparkLowLevel.MotorType.kBrushed);
    backLeftMotor = new CANSparkMax(Constants.DrivetrainConstants.kLeftBackId, MotorType.kBrushed);

    frontRightMotor = new CANSparkMax(Constants.DrivetrainConstants.kRightFrontId, MotorType.kBrushed);
    backRightMotor = new CANSparkMax(Constants.DrivetrainConstants.kRightBackId, MotorType.kBrushed);

    drivePIDController = new PIDController(0.5, 0, 0.1);
    turnPIDController = new PIDController(0.03, 0, 0.003);

    frontRightMotor.setInverted(Constants.DrivetrainConstants.isRightFrontInverted);
    backRightMotor.setInverted(Constants.DrivetrainConstants.isRightBackInverted);
    frontLeftMotor.setInverted(Constants.DrivetrainConstants.isLeftFrontInverted);
    backLeftMotor.setInverted(Constants.DrivetrainConstants.isLeftBackInverted);

    leftSide = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
    rightSide = new MotorControllerGroup(frontRightMotor, backRightMotor);

    differentialDrive = new DifferentialDrive(leftSide, rightSide);
  }

  

  @Override
  public void periodic() {
      
    // This method will be called once per scheduler run
  }

  public void drive(double translation, double rotation){
    if(Math.abs(translation)<Constants.ControllerConstants.kControllerDeadband){
      translation = 0;
    }

    if(Math.abs(rotation)<Constants.ControllerConstants.kControllerDeadband){
      rotation = 0;
    }

    differentialDrive.arcadeDrive(translation, rotation);
  }

  public void visionDrive(double distance, double yaw, int id) {
    var color = DriverStation.getAlliance().get();

    if ((color == DriverStation.Alliance.Blue && id == 7) || (color == DriverStation.Alliance.Red && id == 4)) {
      double driveSpeed = drivePIDController.calculate(distance, distanceConstants.goalMeterDistance);
      double rotationSpeed = -turnPIDController.calculate(yaw, 0);
      differentialDrive.arcadeDrive(driveSpeed, rotationSpeed);
    }
  }
}
