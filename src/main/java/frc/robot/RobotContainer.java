// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive;
import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.commands.VisionMove;
import frc.robot.subsystems.AprilTagStats;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

public static Drivetrain m_drivetrain = new Drivetrain();
public static Shooter m_shooter = new Shooter();
public static AprilTagStats m_AprilTagStats = new AprilTagStats();

public static CommandXboxController driverController = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
public static CommandXboxController operatorController = new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {

    m_drivetrain.setDefaultCommand(new Drive(m_drivetrain));

  
    if(Constants.ControllerConstants.kIsOneController){
      driverController.a().whileTrue(new Intake(m_shooter));
      driverController.b().whileTrue(new Shoot(m_shooter));
      driverController.x().whileTrue(new VisionMove(m_AprilTagStats, m_drivetrain));
    }
    else{
      operatorController.a().whileTrue(new Intake(m_shooter));
      operatorController.b().whileTrue(new Shoot(m_shooter));
    }
    
  }

  public static void setRumble(double rumbleIntensity) {
    driverController.getHID().setRumble(RumbleType.kBothRumble, rumbleIntensity);
  }

  
}
