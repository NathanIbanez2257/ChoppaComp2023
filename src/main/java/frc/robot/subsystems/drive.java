// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class drive extends SubsystemBase {
  /** Creates a new drive. */

  public static WPI_TalonFX rightBack = new WPI_TalonFX(DriveConstants.rightBack);
  public static WPI_TalonFX rightFront = new WPI_TalonFX(DriveConstants.rightFront);

  public static WPI_TalonFX leftBack = new WPI_TalonFX(DriveConstants.leftBack);
  public static WPI_TalonFX leftFront = new WPI_TalonFX(DriveConstants.leftFront);

  public static WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.gyro);
  Rotation2d gyro2D = new Rotation2d(Units.degreesToRadians(getHeading()));

  MotorControllerGroup leftSide = new MotorControllerGroup(rightBack, rightFront);
  MotorControllerGroup rightSide = new MotorControllerGroup(leftBack, leftFront);

  DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);

  Pose2d pose = new Pose2d();

  double x, y, a, distanceToTarget,
      leftPosition, rightPosition, leftEncoderInFeet, rightEncoderInFeet;

  private final DifferentialDriveOdometry mOdometry;

  public drive() {

    resetEncoders();
    brakeMode();
    setFalconScale();
    followSides();
    resetGyro();

    mOdometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);
    mOdometry.resetPosition(gyro2D, leftSideNativeDistanceInMeters(0), rightSideNativeDistanceInMeters(0), null);

  }

  /////////////////////                 Drivetrain Methods       ////////////////////////
  public void move(double leftSpeed, double rightSpeed) {
    leftSide.setInverted(true);
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  private void followSides() {
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
  }

  private void brakeMode() {
    rightBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);

    leftBack.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);
  }

  private void coastMode() {
    rightBack.setNeutralMode(NeutralMode.Coast);
    rightFront.setNeutralMode(NeutralMode.Coast);

    leftBack.setNeutralMode(NeutralMode.Coast);
    leftFront.setNeutralMode(NeutralMode.Coast);

  }

  /////////////////////                 Encoder Methods       ////////////////////////

  public double getRightSideEncoderPosition() {
    return -rightFront.getSelectedSensorPosition();
  }

  public double getLeftSideEncoderPosition() {
    return -leftFront.getSelectedSensorPosition();
  }

  public double getRightSideSensorVelocity() {
    return rightFront.getSelectedSensorVelocity();
  }

  public double getLeftSideSensorVelocity() {
    return leftFront.getSelectedSensorVelocity();
  }

  private void resetEncoders() {
    rightBack.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);

    leftBack.setSelectedSensorPosition(0);
    leftFront.setSelectedSensorPosition(0);
  }


  /////////////////////                 Odometry Methods       ////////////////////////

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    mOdometry.resetPosition(gyro2D, leftSideNativeDistanceInMeters(0), rightSideNativeDistanceInMeters(0), null);
  }

  private void resetGyro() {
    gyro.reset();
    gyro.configFactoryDefault();
  }

  private void zeroHeading() {
    gyro.setYaw(0);
    gyro.configFactoryDefault();
  }

  private WPI_Pigeon2 getPigeon2() {
    return getPigeon2();
  }

  public static double getHeading() {
    return gyro.getYaw();
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public double getRate() {
    return gyro.getRate(); // negative
  }


  /////////////////////                 Differential Drive Method        ////////////////////////

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSideSensorVelocity(), getRightSideSensorVelocity());
  }

 
  /////////////////////                 Volatage Methods       ////////////////////////

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftFront.setVoltage(-leftVolts);
    rightFront.setVoltage(rightVolts);

    drive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public double getAvgEncoderDistance() {
    return ((getRightSideEncoderPosition() + getLeftSideEncoderPosition()) / 2.0);

  }

  private void setFalconScale() {
    rightBack.configSelectedFeedbackCoefficient(DriveConstants.kfalconScale);
    rightFront.configSelectedFeedbackCoefficient(DriveConstants.kfalconScale);

    leftBack.configSelectedFeedbackCoefficient(DriveConstants.kfalconScale);
    leftFront.configSelectedFeedbackCoefficient(DriveConstants.kfalconScale);
  }

  private double leftSideNativeDistanceInMeters(double sensorCounts) {
    double motorRotations = leftFront.getSelectedSensorPosition() / DriveConstants.kCountsPerRev;
    double wheelRotations = motorRotations / DriveConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));

    return positionMeters;

  }

  private double rightSideNativeDistanceInMeters(double sensorCounts) {
    double motorRotations = rightFront.getSelectedSensorPosition() / DriveConstants.kCountsPerRev;
    double wheelRotations = motorRotations / DriveConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));

    return positionMeters;

  }


  /////////////////////                 Limelight Methods       ////////////////////////

  public double angleOff()
  {
    return x;
  }

  public double verticalOffset()
  {
    return y;
  }

  public double targetArea()
  {
    return a;
  }

  public double currentDistance()
  {
    return distanceToTarget;
  }

  public double encoderCurrentDistance()
  {
    return leftEncoderInFeet;
  }



  @Override
  public void periodic() {

    leftEncoderInFeet = leftPosition * DriveConstants.kEncoderDistancePerPulse;
    rightEncoderInFeet = rightPosition * DriveConstants.kEncoderDistancePerPulse;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    a = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", a);


    double limeHeightToTarget = DriveConstants.targetHeight - DriveConstants.limeBaseHeight;
    double theta = Math.toRadians(DriveConstants.limeAngle + y);
    distanceToTarget = limeHeightToTarget / (Math.tan(theta));

    SmartDashboard.putNumber("distanceToTarget", distanceToTarget);
    SmartDashboard.putNumber("LeftEncoder", leftPosition);
    SmartDashboard.putNumber("RightEncoder", rightPosition);




    Rotation2d gyroAngle = gyro.getRotation2d();

    mOdometry.update(gyroAngle, leftSideNativeDistanceInMeters(leftFront.getSelectedSensorPosition()),
        rightSideNativeDistanceInMeters(rightFront.getSelectedSensorPosition()));

    SmartDashboard.putNumber("Gyro 2D", gyroAngle.getDegrees());

    SmartDashboard.putNumber("Pose Meters X", mOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Meters Y", mOdometry.getPoseMeters().getY());

    SmartDashboard.putNumber("Left Encoder Native Meters",
        leftSideNativeDistanceInMeters(leftFront.getSelectedSensorPosition()));

    SmartDashboard.putNumber("Right Encoder Native Meters",
        rightSideNativeDistanceInMeters(rightFront.getSelectedSensorPosition()));

  }
}
