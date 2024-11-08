// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final boolean kIsOneController = true;

    public static final double kControllerDeadband = 0.04;
  }

  public static class DrivetrainConstants{
    public static final int kRightBackId = 1;
    public static final int kRightFrontId = 2;

    public static final int kLeftBackId = 3;
    public static final int kLeftFrontId = 4;

    public static final boolean isRightBackInverted = true;
    public static final boolean isRightFrontInverted = true;
    public static final boolean isLeftBackInverted = false;
    public static final boolean isLeftFrontInverted = false;

    public static final double kTranslationCoefficient = 1.0;
    public static final double kRotationCoefficient = 1.0;
  }

  public static class ShooterContstants{
    public static final int kUpperShooterId = 6;
    public static final int kLowerShooterId = 7;

    public static final double kIntakeSpeed = 0.3;

    public static final double kShootSpeed = 1.0;
  }
  public static class VisionConstants {
    public static class cameraTranslationConstants {
      //translation of camera in meters (change when camera has been mounted on robot)
      public static final double tX = -32 * 0.01;
      public static final double tY = 0.0 * 0.01;
      public static final double tZ = 32 * 0.01;
    }
    public static class cameraRotationConstants {
      //rotation of camera (change when camera has been mounted on robot)
      public static final double rRoll = 0.0;
      public static final double rPitch = 0.0;
      public static final double rYaw = 0.0;
    }

    public static class distanceConstants {
      public static final double goalMeterDistance = 3.0;
    }
 }
}

