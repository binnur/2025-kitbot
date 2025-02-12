// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.epilogue.Logged;

import frc.robot.MotorConfigs.DriveSubsystemConfigs;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

@Logged
public class DriveSubsystem extends SubsystemBase {
   // create brushed motors for drive train
  @Logged(name="Leader Left")
  private final SparkMaxSendable leftLeader = new SparkMaxSendable(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushed);
  private final SparkMaxSendable leftFollower = new SparkMaxSendable(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushed);
  @Logged(name="Leader Right")
  private final SparkMaxSendable  rightLeader = new SparkMaxSendable(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushed);
  private final SparkMaxSendable  rightFollower = new SparkMaxSendable(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushed);

  // setup closed loop controller
  private final SparkClosedLoopController leftController = leftLeader.getClosedLoopController();
  private final SparkClosedLoopController rightController = rightLeader.getClosedLoopController();

  @Logged(name="Differential Drive")
  private final DifferentialDrive drive;

  //private final Field2d field = new Field2d(); 

  // simulation support
  // 1. create DC motor objects for motor type
  //    note: simulate followers as one motor
  // 2. create simulation spark max object
  // 3. create encoder objects -- 
  // 4. simulate drivetrain
  private DCMotor dcMotorLeft;
  private DCMotor dcMotorRight;
  private SparkMaxSim simDcMotorLeft;
  private SparkMaxSim simDcMotorRight;
  private SparkRelativeEncoderSim simEncoderLeft;
  private SparkRelativeEncoderSim simEncoderRight;
  public final DifferentialDrivetrainSim drivetrainSim;

  public double actualDistanceInMetersLeft = 0.0;
  public double actualDistanceInMetersRight = 0.0;

  @Logged(name="DriveIOInfo")
  private final DriveIOInfo ioInfo = new DriveIOInfo();
  @Logged
  public static class DriveIOInfo {
    public double leftPositionInMeters = 0.0;
    public double leftVelocityInMetersPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;

    public double rightPositionInMeters = 0.0;
    public double rightVelocityInMetersPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;

    // TODO: add updateInputs() to this class?
  }

  public DriveSubsystem() {
    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(DriveSubsystemConfigs.canBusTimeout);
    rightLeader.setCANTimeout(DriveSubsystemConfigs.canBusTimeout);
    leftFollower.setCANTimeout(DriveSubsystemConfigs.canBusTimeout);
    rightFollower.setCANTimeout(DriveSubsystemConfigs.canBusTimeout);

    // Apply global configurations & follower mode
    // Resetting in case a new controller is swapped in and persisting in case of a controller 
    // reset due to breaker trip
    DriveSubsystemConfigs.globalConfig.disableFollowerMode();

    // Apply configurations to the motors
    // kResetSafeParameters puts SPARKMAX to a known state (if sparkmax is replaced)
    // kPersistParameters ensure configuration is not lost when sparkmax looses power (ex. mid-operation power cycles)
    leftLeader.configure(DriveSubsystemConfigs.leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(DriveSubsystemConfigs.rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(DriveSubsystemConfigs.leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(DriveSubsystemConfigs.rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // setup followers
    DriveSubsystemConfigs.leftFollowerConfig.follow(leftLeader);
    DriveSubsystemConfigs.rightFollowerConfig.follow(rightLeader);
  
     // construct the differential drive 
     drive = new DifferentialDrive(leftLeader, rightLeader);

    // construct simulation -- important, adding noise distorts the readings and simulation becomes jittery
    // drivetrainSim = new DifferentialDrivetrainSim(
    //   DriveConstants.gearbox,
    //   DriveConstants.gearing,
    //   DriveConstants.robotMOI,
    //   DriveConstants.robotMassKg,
    //   DriveConstants.wheelRadiusMeters,
    //   DriveConstants.trackWidthInMeters,
    //   // The standard deviations for measurement noise:
    //   // x and y: 0.001 m
    //   // heading: 0.001 rad
    //   // l and r velocity: 0.1 m/s
    //   // l and r positions: 0.005 m
    //   VecBuilder.fill(0.0, 0.0, 0.0, 0, 0, 0.0, 0.0)     // was: fill(0.001, 0.001, 0.001, 01, 01, 0.005, 0.005)
    // );

    // Using kitbot simulation -- resulted in less jitter in encoder & velocity readings
    drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide,
      KitbotGearing.k8p45,    // KitbotGearing.k10p71, i.e. 10.71:1
      KitbotWheelSize.kSixInch,
      null
    );

    dcMotorLeft = DCMotor.getCIM(1);
    dcMotorRight = DCMotor.getCIM(1);
    simDcMotorLeft = new SparkMaxSim(leftLeader, dcMotorLeft);
    simDcMotorRight = new SparkMaxSim(rightLeader, dcMotorRight);
    simEncoderLeft = new SparkRelativeEncoderSim(leftLeader);    
    simEncoderRight = new SparkRelativeEncoderSim(rightLeader); 

    // reset encoder positions
    resetEncoders();
  
    // Add Live Window -- used in Test mode
    addChild("Drive/leftLeader", leftLeader);
    addChild("Drive/rightLeader", rightLeader);

    // Add subsystem components for dashboard
    // configDashboardEntries();
  }

  // private void configDashboardEntries() {
  //   SmartDashboard.putData("Drive/Leader Left", leftLeader);
  //   SmartDashboard.putData("Drive/Leader Right", rightLeader);
  //   SmartDashboard.putData(resetEncodersCmd());
  //   // SmartDashboard.putData("Drive/Field", field); 
  // }

  private void updateDriveIOInfo() {
    ioInfo.leftPositionInMeters = leftLeader.getEncoder().getPosition();
    ioInfo.leftVelocityInMetersPerSec = leftLeader.get();
    ioInfo.leftAppliedVolts = leftLeader.getAppliedOutput();
    ioInfo.leftCurrentAmps = leftLeader.getOutputCurrent();

    ioInfo.rightPositionInMeters = rightLeader.getEncoder().getPosition();
    ioInfo.rightVelocityInMetersPerSec = rightLeader.get();
    ioInfo.rightAppliedVolts = rightLeader.getAppliedOutput();
    ioInfo.rightCurrentAmps = rightLeader.getOutputCurrent();  
  }

  private void updateActualDistancesInMeters() {
    actualDistanceInMetersLeft += leftLeader.getEncoder().getPosition();
    actualDistanceInMetersRight += rightLeader.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    updateActualDistancesInMeters();
    updateDriveIOInfo();
  }

   // Update simulation for drive subsystem
  public void simulationPeriodic() {
    // 1. set 'inputs' (voltages) to pysical simulation
    drivetrainSim.setInputs(
      simDcMotorLeft.getAppliedOutput() * RobotController.getInputVoltage(),    // was: leftLeader.get() * RobotController.getInputVoltage(),
      simDcMotorRight.getAppliedOutput() * RobotController.getInputVoltage());  // was: rightLeader.get() * RobotController.getInputVoltage());
    
    // System.out.printf("in simulationPeriodic sim velocity%.2f %.2f\n", drivetrainSim.getLeftVelocityMetersPerSecond(), drivetrainSim.getRightVelocityMetersPerSecond());
    // System.out.printf("in simulationPeriodic real leaders velocity%.2f %.2f\n", leftLeader.get(), rightLeader.get());

    // 2.  update drivetrain state 
    drivetrainSim.update(0.020);
    
    // 3. iterate on simulated motors -- these will update corresponding spark motor & encoder values
    simDcMotorLeft.iterate(
      drivetrainSim.getLeftVelocityMetersPerSecond(),
      RobotController.getBatteryVoltage(), 
      0.020);

    simDcMotorRight.iterate(
      drivetrainSim.getRightVelocityMetersPerSecond(),
      RobotController.getBatteryVoltage(),
      0.020
    );

    // testing w/ arcadeDrive() has getAppliedOutput for real & sim motors -- identical values to each other
    // System.out.printf("in periodic real leaders applied output %.2f %.2f\n", leftLeader.getAppliedOutput(), rightLeader.getAppliedOutput());
    // System.out.printf("in periodic simDCMotor getOutput %.2f %.2f\n", simDcMotorLeft.getAppliedOutput(), simDcMotorRight.getAppliedOutput());

    //updateActualDistancesInMeters(); 
    //updateDriveIOInfo();
  }

  /* 
   * Quick adjustment to the simulated values to update system state
   */
  private void updateSimState(double leftVolts, double rightVolts) {
      // System.out.printf("1.in updateSimState %.2f %.2f\n", leftVolts, rightVolts);
      // stop the simulated drive & iterate over the simulated motors
      drivetrainSim.setInputs(
        leftVolts * RobotController.getInputVoltage(),
        rightVolts * RobotController.getInputVoltage());
      drivetrainSim.update(0.010);

      simDcMotorLeft.iterate(drivetrainSim.getLeftVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), 0.0);
      simDcMotorLeft.iterate(drivetrainSim.getRightVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), 0.0);

      updateDriveIOInfo();
  }

  /* 
   * Set velocity for left & right motors
   * FIXME: in simulation runOpenLoop behaves differently from drive.arcadeDrive(). The velocities differs greatly
   * Note: use driveArcadeCmd() for running open loop
   */
  // set velocity for left & right motors
  public void runOpenLoop(double leftVolts, double rightVolts) {
    // setVoltage behaves differently in simulation from drive.arcade() 
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);

    if (Robot.isSimulation()) {
      //System.out.printf("in runOpenLoop %.2f %.2f\n", leftVolts, rightVolts);
      updateSimState(leftVolts, rightVolts);
    }
  }

  /*
   * Closed-loop velocity control 
   */
  public void setVelocity(double leftVelocity, double rightVelocity) {
    leftController.setReference(leftVelocity, ControlType.kVelocity);
    rightController.setReference(rightVelocity, ControlType.kVelocity);  
    
    if (Robot.isSimulation()) {
      updateSimState(simDcMotorLeft.getAppliedOutput(), simDcMotorRight.getAppliedOutput());
    }
  }

  public void stop()
  {
    runOpenLoop(0.0, 0.0);
  }

  /* Zero drive encoders
   * Note: when running in simulation, need to stop the drivetrain & iterate on simulated motors
   */
  public void resetEncoders() {
    if (Robot.isSimulation()) {
      // update simulated drive 
      updateSimState(drivetrainSim.getLeftVelocityMetersPerSecond(), drivetrainSim.getRightVelocityMetersPerSecond());
      // reset the simulated encoder 
      simDcMotorLeft.setPosition(0.0);
      simDcMotorRight.setPosition(0.0);
    } 
    // note in simulation there is a small drift 
    leftLeader.getEncoder().setPosition(0.0);
    rightLeader.getEncoder().setPosition(0.0);

    actualDistanceInMetersLeft = 0.0;
    actualDistanceInMetersRight = 0.0; 
  }  

  // FIXME: For some reason, left motor encoder is counting backwards
  public BooleanSupplier isAtDistance(double desiredDistanceInMeters) {
    return () -> ((Math.abs(actualDistanceInMetersLeft) >= desiredDistanceInMeters) || 
                  (Math.abs(actualDistanceInMetersRight) >= desiredDistanceInMeters)); 
  }

  // Command to drive the robot with joystick inputs
  public Command driveArcadeCmd(
      DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem)
        .withName("Drive/CMD/driveArcade");
  }

  // FIXME: In simulation this command in provided voltages behave differently than driveArcadeCmd
  public Command driveOpenLoopCmd(
      DriveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        double leftVolts = xSpeed.getAsDouble() + zRotation.getAsDouble();
        double rightVolts = xSpeed.getAsDouble() +zRotation.getAsDouble();
        return Commands.run(
          (() -> this.runOpenLoop(leftVolts, rightVolts)))
          .withName("Drive/CMD/runOpenLoop FIXME!");
  }

  public Command stopCmd() {
    return Commands.runOnce(
      () -> this.stop())
      .withName("Drive/CMD/stopCmd");
  }

  public Command resetEncodersCmd() {
    return this.runOnce(this::resetEncoders)
      .withName("Drive/CMD/Reset Encoders");
  }

  public Command driveFwdInMetersCmd(DriveSubsystem driveSubsystem, DoubleSupplier distanceInMeters) {
    return Commands.startRun(
      this::resetEncodersCmd, 
      () -> this.setVelocity(-DriveConstants.walkingSpeedMetersPerSec, DriveConstants.walkingSpeedMetersPerSec), 
      driveSubsystem)
      .until(this.isAtDistance(distanceInMeters.getAsDouble()))
      .andThen(this.stopCmd())
      .withName("Drive/CMD/driveFwd 3 meters");
  }
  
}
