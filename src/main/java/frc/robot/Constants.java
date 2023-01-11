// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DriveConstants {

    public static final int

    nathan = 0,
        leftDriveAxis = 1, rightDriveAxis = 5,

        rightBack = 0, rightFront = 1,
        leftBack = 2, leftFront = 3,

        gyro = 4,

        kfalconScale = 1,

        kCountsPerRev = 1, kGearRatio = 1, kWheelRadiusInches = 1;

    public static final double

    driveSpeed = .7,

        ksVolts = 0.56093,

        kvVoltSecondsPerMeter = 2.3578,

        kaVoltSecondsSquaredPerMeter = 0.25974,

        kPDriveVelcotiy = 3.0225,

        kTrackWidthMeters = Units.inchesToMeters(25),

        kMaxSpeedMetersPerSecond = 3,

        kMaxAccelerationMPSSquared = 3,

        kRamsesteB = 2,

        kRamseteZeta = .7,

        kEncoderDistancePerPulse = ((Math.PI * 6) / (24576 * 9.375)),



        //////////////////////// Vision Tracking /////////////////////////////////

        hubHeight = 8.667, limelightHeight = 2.25, bottomAngle = 25, targetDistance1 = 10,

        targetHeight = 69, limeBaseHeight = 27, limeAngle = 27,

        targetDistance = 137,

        ///////////////////////               Limelight Angle Tracking PIDS      ////////////////////////////////

        shortAimKP = 0.01, shortAimKI = .025,

        longAimKP = .007, longAimKI = 0.01,  

        aimKD = 0.6, targetAngle = 0.00,



////////////////////////                  PIDS for drive                ////////////////////////////////

//aimKP = 0.05, aimKI = 0, aimKD1 = 0, 

distanceKP = 0.38, distanceKI = 0.09, distanceKD = 0.07;

    // aimKP = .09, aimKI = 0.0001, aimKD = 0, distanceKP = .1 distanceKD 0.01, dKP
    // = .13

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);

  }

  public static class ButtonConstants
  {
    public static int

    limelightButton = 1;



  }
}
