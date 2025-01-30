// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMaxSendable leftLeader;
  private final SparkMaxSendable leftFollower;
  private final SparkMaxSendable rightLeader;
  private final SparkMaxSendable rightFollower;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMaxSendable(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMaxSendable(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMaxSendable(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMaxSendable(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Configuration setup for motors
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    // Create the globalConfiguration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping breakers.
    globalConfig
        .voltageCompensation(DriveConstants.DRIVE_MOTOR_NOMINAL_VOLTAGE)
        .smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);

    // Apply global configurations & follower mode
    // Resetting in case a new controller is swapped in and persisting in case of a controller 
    // reset due to breaker trip
    globalConfig.disableFollowerMode();
    
    leftLeaderConfig.apply(globalConfig);
    rightLeaderConfig.apply(globalConfig);
    leftFollowerConfig.apply(globalConfig)
        .follow(leftLeader);
    rightFollowerConfig.apply(globalConfig)
        .follow(rightLeader);

    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    leftLeader.configure(globalConfig,ResetMode.kResetSafeParameters,   PersistMode.kPersistParameters);
    rightLeader.configure(globalConfig,ResetMode.kResetSafeParameters,   PersistMode.kPersistParameters);
    leftFollower.configure(globalConfig,ResetMode.kResetSafeParameters,   PersistMode.kPersistParameters);
    rightFollower.configure(globalConfig,ResetMode.kResetSafeParameters,   PersistMode.kPersistParameters);

    // Add Live Window -- used in Test mode
    addChild("Drive/leftLeader", leftLeader);
    addChild("Drive/rightLeader", rightLeader);

    // Add subsystem components for dashboard
    configDashboardEntries();
  }

  private void configDashboardEntries() {
    SmartDashboard.putData("Drive/leftLeader", leftLeader);
    SmartDashboard.putData("Drive/rightLeader", rightLeader);
  }

  @Override
  public void periodic() {
  }

  // Command to drive the robot with joystick inputs
  public Command driveArcade(
      CANDriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem);
  }
}
