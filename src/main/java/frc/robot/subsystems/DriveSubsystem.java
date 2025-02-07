// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfigs.DriveSubsystemConfigs;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // add motors
  private final SparkMaxSendable leftLeader;
  private final SparkMaxSendable leftFollower;
  private final SparkMaxSendable rightLeader;
  private final SparkMaxSendable rightFollower;

  // setup closed loop controller
  private final SparkClosedLoopController leftClosedLoopController;
  private final SparkClosedLoopController rightClosedLoopController;
  private final SparkRelativeEncoder leftEncoder;
  private final SparkRelativeEncoder rightEncoder;

  private final DifferentialDrive drive;
  private final Field2d field = new Field2d();

  // simulation support
  // 1. create DC motor objects for motor type
  //    note: simulate followers as one motor
  // 2. create simulation spark max object
  // 3. create encoder objects -- 
  // 4. simulate drivetrain
  private DCMotor leftDcMotor;
  private DCMotor rightDcMotor;
  private SparkMaxSim leftDcMotorSim;
  private SparkMaxSim rightDcMotorSim;
  private SparkRelativeEncoderSim leftEncoderSim;
  private SparkRelativeEncoderSim rightEncoderSim;
  public final DifferentialDrivetrainSim drivetrainSim;

  public DriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMaxSendable(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMaxSendable(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMaxSendable(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMaxSendable(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    // TODO: how does this fit w/ config setup?
    leftLeader.setCANTimeout(DriveSubsystemConfigs.kDriveCANTimeout);
    rightLeader.setCANTimeout(DriveSubsystemConfigs.kDriveCANTimeout);
    leftFollower.setCANTimeout(DriveSubsystemConfigs.kDriveCANTimeout);
    rightFollower.setCANTimeout(DriveSubsystemConfigs.kDriveCANTimeout);

    // Apply global configurations & follower mode
    // Resetting in case a new controller is swapped in and persisting in case of a controller 
    // reset due to breaker trip
    DriveSubsystemConfigs.globalConfig.disableFollowerMode();

    // Apply configurations to the motors
    // kResetSafeParameters puts SPARKMAX to a known state (if sparkmax is replaced)
    // kPersistParameters ensure configuration is not lost when sparkmax looses power (ex. mid-operation power cycles)
    leftLeader.configure(DriveSubsystemConfigs.leftLeaderConfig,ResetMode.kResetSafeParameters,   PersistMode.kPersistParameters);
    rightLeader.configure(DriveSubsystemConfigs.rightLeaderConfig,ResetMode.kResetSafeParameters,   PersistMode.kPersistParameters);
    leftFollower.configure(DriveSubsystemConfigs.leftFollowerConfig,ResetMode.kResetSafeParameters,   PersistMode.kPersistParameters);
    rightFollower.configure(DriveSubsystemConfigs.rightFollowerConfig,ResetMode.kResetSafeParameters,   PersistMode.kPersistParameters);

    // setup followers
    DriveSubsystemConfigs.leftFollowerConfig.follow(leftLeader);
    DriveSubsystemConfigs.rightFollowerConfig.follow(rightLeader);

    // setup closed loop controller & encoder
    leftClosedLoopController = leftLeader.getClosedLoopController();
    rightClosedLoopController = rightLeader.getClosedLoopController();
    leftEncoder = (SparkRelativeEncoder) leftLeader.getEncoder();
    rightEncoder = (SparkRelativeEncoder) rightLeader.getEncoder();

    // reset encoder positions
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
  
     // set up differential drive class
     drive = new DifferentialDrive(leftLeader, rightLeader);

    // construct simulation
    leftDcMotor = DCMotor.getCIM(1);
    rightDcMotor = DCMotor.getCIM(1);
    leftDcMotorSim = new SparkMaxSim(leftLeader, leftDcMotor);
    rightDcMotorSim = new SparkMaxSim(rightLeader, rightDcMotor);
    leftEncoderSim = new SparkRelativeEncoderSim(leftLeader);     // is this setting up encoder correctly?
    rightEncoderSim = new SparkRelativeEncoderSim(rightLeader); 
    leftEncoderSim.setPosition(0.0);
    rightEncoderSim.setPosition(0.0);
  
    drivetrainSim = new DifferentialDrivetrainSim(
      DCMotor.getCIM(2),      // two CIM motors on each side of drivetrain
      1,                                // no gear reduction
      7.5,                              // FIXME: MOI of 7.5kg m^2 from CAD model 
      60.0,                             // FIXME: mass of the robot
      Units.inchesToMeters(3),   // Robot uses 3" radius wheels
      0.7112,                           // Track width is 0.7112 meters
      // The standard deviations for measurement noise:
      // x and y: 0.001 m
      // heading: 0.001 rad
      // l and r velocity: 0.1 m/s
      // l and r positions: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 01, 01, 0.005, 0.005) 
    );

    // Dashboard configurations
    // Add Live Window -- used in Test mode
    addChild("Drive/leftLeader", leftLeader);
    addChild("Drive/rightLeader", rightLeader);

    // Add subsystem components for dashboard
    configDashboardEntries();
  }

  private void configDashboardEntries() {
    SmartDashboard.putData("Drive/Leader Left", leftLeader);
    SmartDashboard.putData("Drive/Leader Right", rightLeader);
    SmartDashboard.setDefaultBoolean("Drive/Reset Encoder", false);
    SmartDashboard.putData("Drive/Field", field); 
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
      leftEncoderSim.setPosition(0.0);
      rightEncoderSim.setPosition(0.0);
    }

    SmartDashboard.putNumber("Drive/Encoder Left Sim Position", leftEncoderSim.getPosition());
    SmartDashboard.putNumber("Drive/Encoder Left Sim Velocity", leftEncoderSim.getVelocity());
    
  }

  @Override
  public void periodic() {
    updateDashboardEntries();
  }

  public void simulationPeriodic() {
    // Update simulation for drive subsystem
    // 1. set 'inputs' (voltages)
    // 2. write simulated positions and velocities for simulated components
    //    -- note: negates right side so positive voltages make right side move forward
    drivetrainSim.setInputs(
      leftDcMotorSim.getAppliedOutput() * RobotController.getInputVoltage(),    // was: leftLeader.get() * RobotController.getInputVoltage(),
      rightDcMotorSim.getAppliedOutput() * RobotController.getInputVoltage());  // was: rightLeader.get() * RobotController.getInputVoltage());
          
    drivetrainSim.update(0.020);

    leftEncoderSim.iterate(drivetrainSim.getLeftVelocityMetersPerSecond(), 0.020);    // fixme: dt
    // this is the wpi way? 
    // leftEncoderSim.setPosition(drivetrainSim.getLeftPositionMeters());
    // leftEncoderSim.setVelocity(drivetrainSim.getLeftVelocityMetersPerSecond());
    // rightEncoderSim.setPosition(drivetrainSim.getRightPositionMeters());
    // rightEncoderSim.setVelocity(drivetrainSim.getRightVelocityMetersPerSecond());
  }

  // update control loop 
  public void setClosedLoopControllerPosition(double setPosition) {
    leftClosedLoopController.setReference(setPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightClosedLoopController.setReference(setPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);    
  }

  public Command setClosedLoopControllerPositionWithSpeed(DriveSubsystem driveSubsystem, DoubleSupplier setPosition, DoubleSupplier targetSpeed) {
    return Commands.run (
        () -> leftClosedLoopController.setReference(setPosition.getAsDouble(), ControlType.kPosition, ClosedLoopSlot.kSlot0),
        // () -> rightClosedLoopController.setReference(setPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0),    
        // () -> leftLeader.set(targetSpeed.getAsDouble()),
        // rightLeader.set(targetSpeed.getAsDouble()),
        driveSubsystem
      );
  }

  public void setClosedLoopControllerVelocity(double setVelocity) {
    leftClosedLoopController.setReference(setVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    rightClosedLoopController.setReference(setVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);    
  }

  // Command to drive the robot with joystick inputs
  public Command driveArcade(
      DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem);
  }

}
