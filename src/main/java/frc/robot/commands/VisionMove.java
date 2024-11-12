package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTagStats;
import frc.robot.subsystems.Drivetrain;

public class VisionMove extends Command {
    AprilTagStats aprilTagStats;
    Drivetrain drivetrain;
    boolean targetIsFound;

    public VisionMove(AprilTagStats aprilTagStats, Drivetrain drivetrain) {
        this.aprilTagStats = aprilTagStats;
        this.drivetrain = drivetrain;
        addRequirements(aprilTagStats, drivetrain);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        targetIsFound = aprilTagStats.hasTarget();
        aprilTagStats.setTagView(targetIsFound);
        if (targetIsFound) {
            aprilTagStats.updateData();
            drivetrain.visionDrive(aprilTagStats.getDistance(), aprilTagStats.getYaw(), aprilTagStats.getID());
        } else {
            //RobotContainer.setRumble(0.5);
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //RobotContainer.setRumble(0);
        drivetrain.visionDrive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
