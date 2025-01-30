// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfigs.CANDriveSubsystemConfigs;
import frc.robot.Constants.DriveConstants;

public class CANDriveSubsystem extends SubsystemBase {
  // add motors
  private final SparkMaxSendable leftLeader;
  private final SparkMaxSendable leftFollower;
  private final SparkMaxSendable rightLeader;
  private final SparkMaxSendable rightFollower;

  // setup closed loop controller
  private final SparkClosedLoopController leftClosedLoopController;
  private final SparkClosedLoopController rightClosedLoopController;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMaxSendable(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMaxSendable(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMaxSendable(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMaxSendable(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // setup closed loop controller
    leftClosedLoopController = leftLeader.getClosedLoopController();
    rightClosedLoopController = rightLeader.getClosedLoopController();
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(CANDriveSubsystemConfigs.kDriveCANTimeout);
    rightLeader.setCANTimeout(CANDriveSubsystemConfigs.kDriveCANTimeout);
    leftFollower.setCANTimeout(CANDriveSubsystemConfigs.kDriveCANTimeout);
    rightFollower.setCANTimeout(CANDriveSubsystemConfigs.kDriveCANTimeout);

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
        .voltageCompensation(CANDriveSubsystemConfigs.kDriveMotorNominalVoltage)
        .smartCurrentLimit(CANDriveSubsystemConfigs.kDriveMotorCurrentLimit)
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

    // add enconders
    leftLeaderConfig.encoder
        .positionConversionFactor(CANDriveSubsystemConfigs.kDrivePositionConversionFactor)
        .velocityConversionFactor(CANDriveSubsystemConfigs.kDriveVelocityConversionFactor);
    rightLeaderConfig.encoder
        .positionConversionFactor(CANDriveSubsystemConfigs.kDrivePositionConversionFactor)
        .velocityConversionFactor(CANDriveSubsystemConfigs.kDriveVelocityConversionFactor);

    // configure closed loop controller
    leftLeaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // set PID values for position control. Closed loop slot defaults to slot 0
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // set PID values for velocity control to slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        // FIXME: magic numbers! 
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

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
    SmartDashboard.putData("Drive/Leader Left", leftLeader);
    SmartDashboard.putData("Drive/Leader Right", rightLeader);
    SmartDashboard.setDefaultNumber("Drive/Target Position(inch)", 0);
    SmartDashboard.setDefaultNumber("Drive/Target Velocity", 0);
    //SmartDashboard.setDefaultBoolean("Drive/Control Mode (false: Velocity)", false);  // Default Velocity
    SmartDashboard.setDefaultBoolean("Drive/Reset Encoder", false);
  }

  private void updateDashboardEntries() {
    SmartDashboard.putNumber("Drive/Actual Position Left", leftEncoder.getPosition());
    SmartDashboard.putNumber("Drive/Actual Position Right", rightEncoder.getPosition());
    SmartDashboard.putNumber("Drive/Actual Velocity Left", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Drive/Actual Velocity Right", leftEncoder.getVelocity());
    
    if (SmartDashboard.getBoolean("Drive/Reset Encoder", false)) {
      SmartDashboard.putBoolean("Drive/Reset Encoder", false);
      // reset encoders
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
    }
  }

  @Override
  public void periodic() {
    updateDashboardEntries();
  }

  // update control loop 
  public void setClosedLoopControllerPosition(double setPosition) {
    leftClosedLoopController.setReference(setPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightClosedLoopController.setReference(setPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);    
  }

  public void setClosedLoopControllerVelocity(double setVelocity) {
    leftClosedLoopController.setReference(setVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    rightClosedLoopController.setReference(setVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);    
  }

  // Command to drive the robot with joystick inputs
  public Command driveArcade(
      CANDriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem);
  }

}
