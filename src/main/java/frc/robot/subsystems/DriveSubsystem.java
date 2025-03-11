// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkMaxSim;
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
  public final DifferentialDrivetrainSim drivetrainSim;


  @Logged(name="DriveIOInfo")
  private final DriveIOInfo ioInfo = new DriveIOInfo();
  @Logged
  public static class DriveIOInfo {
    public double leftPositionInMeters = 0.0;
    public double leftVelocityInMetersPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftDesiredPositionInMeters = 0.0;

    public double rightPositionInMeters = 0.0;
    public double rightVelocityInMetersPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightDesiredPositionInMeters = 0.0;

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

    // IMPORTANT: important, adding noise distorts the readings and simulation becomes jittery
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

    dcMotorLeft = DCMotor.getCIM(2);
    dcMotorRight = DCMotor.getCIM(2);
    simDcMotorLeft = new SparkMaxSim(leftLeader, dcMotorLeft);
    simDcMotorRight = new SparkMaxSim(rightLeader, dcMotorRight);

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

  @Override
  public void periodic() {
    updateDriveIOInfo();
  }

   // Update simulation for drive subsystem
  public void simulationPeriodic() {
    // 1. set 'inputs' (voltages) to pysical simulation
    drivetrainSim.setInputs(
      simDcMotorLeft.getAppliedOutput() * RobotController.getInputVoltage(),    // was: leftLeader.get() * RobotController.getInputVoltage(),
      simDcMotorRight.getAppliedOutput() * RobotController.getInputVoltage());  // was: rightLeader.get() * RobotController.getInputVoltage());
        
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
  }

  /** 
   * Set velocity for left & right motors
   * Note: use driveArcadeCmd() for running open loop
   */
  // set velocity for left & right motors
  public void runOpenLoop(double leftVolts, double rightVolts) {
    // setVoltage behaves differently in simulation from drive.arcade() 
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  /**
   * Closed-loop velocity control 
   */
  public void setVelocity(double leftVelocity, double rightVelocity) {
    leftController.setReference(leftVelocity, ControlType.kVelocity);
    rightController.setReference(rightVelocity, ControlType.kVelocity);  
  }

  public void stop()
  {
    runOpenLoop(0.0, 0.0);
  }

  /* Zero drive encoders
   * Note: when running in simulation, need to stop the drivetrain & iterate on simulated motors
   */
  public void resetEncoders() {
    // note in simulation there is a small drift 
    leftLeader.getEncoder().setPosition(0.0);
    rightLeader.getEncoder().setPosition(0.0);

    // reset the simulated encoder -- seems to require it for simulation vs. resetting from leader.getEncoder().setPosition()
    if (Robot.isSimulation()) {
      simDcMotorLeft.setPosition(0.0);
      simDcMotorRight.setPosition(0.0);
    } 
  }  

  /**
   * Determines if given encoders moved at desired distance
   */
  public BooleanSupplier isAtDistance(double desiredDistanceInMeters) {
    return () -> {
        double currentLeftPosition = leftLeader.getEncoder().getPosition();
        double currentRightPosition = rightLeader.getEncoder().getPosition();
        
        // Use absolute difference to handle both positive and negative distances
        double leftDifference = Math.abs(currentLeftPosition - desiredDistanceInMeters);
        double rightDifference = Math.abs(currentRightPosition - desiredDistanceInMeters);
        
        // Define a small tolerance for position accuracy
        double positionTolerance = 0.05; // 5 cm tolerance -- adjust as needed
        
        // Return true if either encoder is within tolerance of the desired position
        return (leftDifference <= positionTolerance) || (rightDifference <= positionTolerance);
    };
}

  /** Command to drive the robot with joystick inputs */
  public Command driveArcadeCmd(
      DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
        // forward values are positive (was -xSpeed and -zRotation)
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem)
        .withName("Drive/CMD/driveArcade");
  }

  /** Emulates driveArcadeCmd using setVoltage for motors */
  public Command driveOpenLoopCmd(
      DriveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        return Commands.run( () -> {
            double leftVolts = xSpeed.getAsDouble() + zRotation.getAsDouble();
            double rightVolts = xSpeed.getAsDouble() +zRotation.getAsDouble();
            this.runOpenLoop(leftVolts, rightVolts);
          })
          .withName("Drive/CMD/runOpenLoop");
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

  /** 
   * Drives forward for specified distance
   */
  public Command driveFwdInMetersCmd(DriveSubsystem driveSubsystem, DoubleSupplier distanceInMeters) {
    // note: this is commented section is WRONG! values are captured during command creation and NOT updated dynamically at run time
    // whoever the latest autoCommand is will override the desired positions
    // ioInfo.leftDesiredPositionInMeters = 0.5; // distanceInMeters.getAsDouble();
    // ioInfo.rightDesiredPositionInMeters = 0.5; // distanceInMeters.getAsDouble();

    return Commands.sequence(
      Commands.runOnce( () -> {
          // Initial setup when command starts
          this.resetEncodersCmd();
          ioInfo.leftDesiredPositionInMeters = distanceInMeters.getAsDouble();
          ioInfo.rightDesiredPositionInMeters = distanceInMeters.getAsDouble();
      }),
      Commands.run( () -> {
          this.setVelocity(DriveConstants.walkingSpeedMetersPerSec, DriveConstants.walkingSpeedMetersPerSec);
      }, driveSubsystem)
          .until(this.isAtDistance(distanceInMeters.getAsDouble()))
          .andThen(this.stopCmd())
          .withName("Drive/CMD/driveBack in meters")
    );
  }

  /**
   * Drive backwards for specified distance
   */
  public Command driveBackInMetersCmd(DriveSubsystem driveSubsystem, DoubleSupplier distanceInMeters) {
    return Commands.sequence(
      Commands.runOnce(() -> {
          // Initial setup when command starts
          this.resetEncodersCmd();
          // Set the initial desired position -- 
          ioInfo.leftDesiredPositionInMeters = distanceInMeters.getAsDouble();
          ioInfo.rightDesiredPositionInMeters = distanceInMeters.getAsDouble();
      }),
      Commands.run( () -> {
          // Set velocity -- passing - to left & + to right, matching inverted motor configuration
          this.setVelocity(-DriveConstants.maxSpeedMetersPerSec, -DriveConstants.maxSpeedMetersPerSec);
      }, driveSubsystem)
          .until(this.isAtDistance(distanceInMeters.getAsDouble()))
          .andThen(this.stopCmd())
          .withName("Drive/CMD/driveBack in meters")
    );
  } 
}
