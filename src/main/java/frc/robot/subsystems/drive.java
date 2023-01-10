// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class drive extends SubsystemBase {
  /** Creates a new drive. */

  WPI_TalonFX rightBack = new WPI_TalonFX(DriveConstants.rightBack);
  WPI_TalonFX rightFront = new WPI_TalonFX(DriveConstants.rightFront);

  WPI_TalonFX leftBack = new WPI_TalonFX(DriveConstants.leftBack);
  WPI_TalonFX leftFront = new WPI_TalonFX(DriveConstants.leftFront);

  MotorControllerGroup leftSide = new MotorControllerGroup(rightBack, rightFront);
  MotorControllerGroup rightSide = new MotorControllerGroup(leftBack, leftFront);

  DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);

  public drive() {
    brakeMode();






  }



  public void move(double leftSpeed, double rightSpeed) {
    leftSide.setInverted(true);
    drive.tankDrive(leftSpeed, rightSpeed);
  }

    private void brakeMode()
    {
      rightBack.setNeutralMode(NeutralMode.Brake);
      rightFront.setNeutralMode(NeutralMode.Brake);

      leftBack.setNeutralMode(NeutralMode.Brake);
      leftFront.setNeutralMode(NeutralMode.Brake);
    }

    private void coastMode()
    {
      rightBack.setNeutralMode(NeutralMode.Coast);
      rightFront.setNeutralMode(NeutralMode.Coast);

      leftBack.setNeutralMode(NeutralMode.Coast);
      leftFront.setNeutralMode(NeutralMode.Coast);
      
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
