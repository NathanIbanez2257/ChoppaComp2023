// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive;

public class limelightCommand extends CommandBase {
  drive drive;

  double proportionAim, integralAim, derivativeAim, // PID Aiming Vars

      errorAim, oldErrorAim, speedAim, // error values, speed aim = PID finalization

      proportionDistance, integralDistance, derivativeDistance, // PID Distance Vars

      errorDistance, oldErrorDistance, speedDistance, // error values, distance forward || backwards, speed Distance =
                                                      // PID finalization

      pastTime, td; // td = target distance, past time

  /*
   * 
   * tv Whether the limelight has any valid targets (0 or 1)
   * 
   * tx Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees
   * | LL2: -29.8 to 29.8 degrees)
   * 
   * ty Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
   * degrees | LL2: -24.85 to 24.85 degrees)
   * 
   * ta Target Area (0% of image to 100% of image)
   * 
   * ts Skew or rotation (-90 degrees to 0 degrees)
   * 
   * tl The pipeline’s latency contribution (ms) Add at least 11ms for image
   * capture latency.
   * 
   * tshort Sidelength of shortest side of the fitted bounding box (pixels)
   * 
   * tlong Sidelength of longest side of the fitted bounding box (pixels)
   * 
   * thor Horizontal sidelength of the rough bounding box (0 - 320 pixels)
   * 
   * tvert Vertical sidelength of the rough bounding box (0 - 320 pixels)
   * 
   * getpipe True active pipeline index of the camera (0 .. 9)
   * 
   * camtran Results of a 3D position solution, NumberArray: Translation (x,y,z)
   * Rotation(pitch,yaw,roll)
   * 
   */

  public limelightCommand(drive driveSub, double targetDistance) {
    drive = driveSub;
    td = targetDistance;
    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
    oldErrorAim = 0;
    oldErrorDistance = 0;
    integralAim = 0;
    integralDistance = 0;
    pastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {

    /*
     * while(Math.abs(drive.angleOff()) > 3) {
     * 
     * // run PID with no D, between error of 20 degrees +-
     * if(drive.angleOff() < 20)
     * {
     * 
     * errorAim = DriveConstants.targetAngle + (-1 * drive.angleOff()); // target
     * angle offset, angle Off = error angle
     * proportionAim = errorAim * DriveConstants.shortAimKP; // proportion
     * 
     * errorDistance = td - drive.currentDistance();
     * 
     * proportionDistance = errorDistance * DriveConstants.distanceKP;
     * 
     * 
     * 
     * double dt = Timer.getFPGATimestamp() - pastTime;
     * pastTime = Timer.getFPGATimestamp();
     * 
     * 
     * 
     * 
     * integralAim += (errorAim * dt) * DriveConstants.shortAimKI;
     * 
     * integralDistance += (errorDistance * dt) * DriveConstants.distanceKI;
     * 
     * 
     * 
     * double dxAim = errorAim - oldErrorAim;
     * double dxDistance = errorDistance - oldErrorDistance;
     * 
     * derivativeAim = ( dxAim / dt) * DriveConstants.aimKD;
     * derivativeDistance = (dxDistance / dt) * DriveConstants.distanceKD;
     * 
     * oldErrorAim = errorAim;
     * oldErrorDistance = errorDistance;
     * 
     * 
     * speedAim = proportionAim + integralAim;
     * 
     * drive.move(speedAim, speedAim *-1); //+ speedDistance, -1* speedAim +
     * speedDistance (.34, .89)
     * 
     * }
     * 
     * else if(drive.angleOff() > -20)
     * {
     * 
     * errorAim = DriveConstants.targetAngle + (-1 * drive.angleOff()); // target
     * angle offset, angle Off = error angle
     * proportionAim = errorAim * DriveConstants.shortAimKP; // proportion
     * 
     * errorDistance = td - drive.currentDistance();
     * 
     * proportionDistance = errorDistance * DriveConstants.distanceKP;
     * 
     * 
     * 
     * double dt = Timer.getFPGATimestamp() - pastTime;
     * pastTime = Timer.getFPGATimestamp();
     * 
     * 
     * 
     * 
     * integralAim += (errorAim * dt) * DriveConstants.shortAimKI;
     * integralDistance += (errorDistance * dt) * DriveConstants.distanceKI;
     * 
     * 
     * 
     * 
     * double dxAim = errorAim - oldErrorAim;
     * double dxDistance = errorDistance - oldErrorDistance;
     * derivativeAim = ( dxAim / dt) * DriveConstants.aimKD;
     * derivativeDistance = (dxDistance / dt) * DriveConstants.distanceKD;
     * oldErrorAim = errorAim;
     * oldErrorDistance = errorDistance;
     * 
     * 
     * 
     * 
     * speedAim = proportionAim + integralAim;
     * 
     * //speedDistance = proportionDistance + integralDistance + derivativeDistance;
     * 
     * drive.move(speedAim, speedAim *-1); //+ speedDistance, -1* speedAim +
     * speedDistance
     * 
     * }
     * 
     * 
     * 
     * else
     * {
     * drive.move(0, 0);
     * errorAim = DriveConstants.targetAngle + (-1 * drive.angleOff());
     * proportionAim = errorAim * DriveConstants.longAimKP;
     * 
     * errorDistance = td - drive.currentDistance();
     * proportionDistance = errorDistance * DriveConstants.distanceKP;
     * 
     * 
     * 
     * double dt = Timer.getFPGATimestamp() - pastTime;
     * pastTime = Timer.getFPGATimestamp();
     * 
     * 
     * 
     * 
     * integralAim += (errorAim * dt) * DriveConstants.longAimKI;
     * integralDistance += (errorDistance * dt) * DriveConstants.distanceKI;
     * 
     * 
     * 
     * 
     * double dxAim = errorAim - oldErrorAim;
     * double dxDistance = errorDistance - oldErrorDistance;
     * derivativeAim = Math.abs(( dxAim / dt) * DriveConstants.aimKD);
     * derivativeDistance = (dxDistance / dt) * DriveConstants.distanceKD;
     * oldErrorAim = errorAim;
     * oldErrorDistance = errorDistance;
     * 
     * 
     * 
     * 
     * speedAim = proportionAim + integralAim - derivativeAim;
     * 
     * 
     * 
     * // speedDistance = proportionDistance + integralDistance +
     * derivativeDistance;
     * 
     * drive.move(speedAim, speedAim *-1); //+ speedDistance, -1* speedAim +
     * speedDistance
     * 
     * 
     * }
     * }
     */

    if (drive.angleOff() < 20) {

      errorAim = DriveConstants.targetAngle + (-1 * drive.angleOff()); // target angle offset, angle Off = error angle
      proportionAim = errorAim * DriveConstants.shortAimKP; // proportion

      errorDistance = td - drive.currentDistance();

      proportionDistance = errorDistance * DriveConstants.distanceKP;

      double dt = Timer.getFPGATimestamp() - pastTime;
      pastTime = Timer.getFPGATimestamp();

      integralAim += (errorAim * dt) * DriveConstants.shortAimKI;

      integralDistance += (errorDistance * dt) * DriveConstants.distanceKI;

      double dxAim = errorAim - oldErrorAim;
      double dxDistance = errorDistance - oldErrorDistance;

      derivativeAim = (dxAim / dt) * DriveConstants.aimKD;
      derivativeDistance = (dxDistance / dt) * DriveConstants.distanceKD;

      oldErrorAim = errorAim;
      oldErrorDistance = errorDistance;

      speedAim = proportionAim + integralAim;

      drive.move(speedAim, speedAim * -1); // + speedDistance, -1* speedAim + speedDistance (.34, .89)

    }

    else if (drive.angleOff() > -20) {

      errorAim = DriveConstants.targetAngle + (-1 * drive.angleOff()); // target angle offset, angle Off = error angle
      proportionAim = errorAim * DriveConstants.shortAimKP; // proportion

      errorDistance = td - drive.currentDistance();

      proportionDistance = errorDistance * DriveConstants.distanceKP;

      double dt = Timer.getFPGATimestamp() - pastTime;
      pastTime = Timer.getFPGATimestamp();

      integralAim += (errorAim * dt) * DriveConstants.shortAimKI;
      integralDistance += (errorDistance * dt) * DriveConstants.distanceKI;

      double dxAim = errorAim - oldErrorAim;
      double dxDistance = errorDistance - oldErrorDistance;
      derivativeAim = (dxAim / dt) * DriveConstants.aimKD;
      derivativeDistance = (dxDistance / dt) * DriveConstants.distanceKD;
      oldErrorAim = errorAim;
      oldErrorDistance = errorDistance;

      speedAim = proportionAim + integralAim;

      // speedDistance = proportionDistance + integralDistance + derivativeDistance;

      drive.move(speedAim, speedAim * -1); // + speedDistance, -1* speedAim + speedDistance

    }

    else {
      drive.move(0, 0);
      errorAim = DriveConstants.targetAngle + (-1 * drive.angleOff());
      proportionAim = errorAim * DriveConstants.longAimKP;

      errorDistance = td - drive.currentDistance();
      proportionDistance = errorDistance * DriveConstants.distanceKP;

      double dt = Timer.getFPGATimestamp() - pastTime;
      pastTime = Timer.getFPGATimestamp();

      integralAim += (errorAim * dt) * DriveConstants.longAimKI;
      integralDistance += (errorDistance * dt) * DriveConstants.distanceKI;

      double dxAim = errorAim - oldErrorAim;
      double dxDistance = errorDistance - oldErrorDistance;
      derivativeAim = Math.abs((dxAim / dt) * DriveConstants.aimKD);
      derivativeDistance = (dxDistance / dt) * DriveConstants.distanceKD;
      oldErrorAim = errorAim;
      oldErrorDistance = errorDistance;

      speedAim = proportionAim + integralAim - derivativeAim;

      // speedDistance = proportionDistance + integralDistance + derivativeDistance;

      drive.move(speedAim, speedAim * -1); // + speedDistance, -1* speedAim + speedDistance

    }

    // smart dashboard
    SmartDashboard.putNumber("Aim Error", errorAim);
    SmartDashboard.putNumber("Aim speed", speedAim);
    SmartDashboard.putNumber("kd", derivativeAim);

  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.move(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
