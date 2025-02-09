// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  @Logged(name="Differential Drive")
  private final DifferentialDrive drive;



  //private final Field2d field = new Field2d(); 

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

  public double startLeftPositionInMeters = 0.0;
  public double startRightPositionInMeters = 0.0;

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

    leftDcMotor = DCMotor.getCIM(1);
    rightDcMotor = DCMotor.getCIM(1);
    leftDcMotorSim = new SparkMaxSim(leftLeader, leftDcMotor);
    rightDcMotorSim = new SparkMaxSim(rightLeader, rightDcMotor);
    leftEncoderSim = new SparkRelativeEncoderSim(leftLeader);    
    rightEncoderSim = new SparkRelativeEncoderSim(rightLeader); 

    // reset encoder positions
    resetEncoders();
  
    // Add Live Window -- used in Test mode
    addChild("Drive/leftLeader", leftLeader);
    addChild("Drive/rightLeader", rightLeader);

    // Add subsystem components for dashboard
    configDashboardEntries();
  }

  private void configDashboardEntries() {
    SmartDashboard.putData("Drive/Leader Left", leftLeader);
    SmartDashboard.putData("Drive/Leader Right", rightLeader);
    SmartDashboard.putData(resetEncodersCmd());
    // SmartDashboard.putData("Drive/Field", field); 
  }

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
      leftDcMotorSim.getAppliedOutput() * RobotController.getInputVoltage(),    // was: leftLeader.get() * RobotController.getInputVoltage(),
      rightDcMotorSim.getAppliedOutput() * RobotController.getInputVoltage());  // was: rightLeader.get() * RobotController.getInputVoltage());
    
    System.out.printf("in simulationPeriodic %.2f %.2f\n", drivetrainSim.getLeftVelocityMetersPerSecond(), drivetrainSim.getRightVelocityMetersPerSecond());

    // 2.  update drivetrain state 
    drivetrainSim.update(0.020);
    
    // 3. iterate on simulated motors -- these will update corresponding spark motor & encoder values
    leftDcMotorSim.iterate(
      drivetrainSim.getLeftVelocityMetersPerSecond(),
      RobotController.getBatteryVoltage(), 
      0.020);

    rightDcMotorSim.iterate(
      drivetrainSim.getRightVelocityMetersPerSecond(),
      RobotController.getBatteryVoltage(),
      0.020
    );

    updateDriveIOInfo();
  }

  /* 
   * Quick adjustment to the simulated values to update system state
   */
  private void updateSimState(double leftVolts, double rightVolts) {
      System.out.printf("1.in updateSim %.2f %.2f\n", leftVolts, rightVolts);
      // stop the simulated drive & iterate over the simulated motors
      drivetrainSim.setInputs(
        leftVolts,
        rightVolts);
      drivetrainSim.update(0.010);
      //System.out.printf("2.in updateSimState %.2f %.2f\n", drivetrainSim.getLeftVelocityMetersPerSecond(), drivetrainSim.getRightVelocityMetersPerSecond());

      leftDcMotorSim.iterate(drivetrainSim.getLeftVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), 0.0);
      leftDcMotorSim.iterate(drivetrainSim.getRightVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), 0.0);

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

    // using drive.arcade() in runOpenLoop doesn't improve simulation
    // drive.arcadeDrive(leftVolts, rightVolts);
    if (Robot.isSimulation()) {
      //System.out.printf("in runOpenLoop %.2f %.2f\n", leftVolts, rightVolts);
      updateSimState(leftVolts, rightVolts);
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
      leftDcMotorSim.setPosition(0.0);
      rightDcMotorSim.setPosition(0.0);
    } 
    // note in simulation there is a small drift 
    leftLeader.getEncoder().setPosition(0.0);
    rightLeader.getEncoder().setPosition(0.0);
} 

  public Command setClosedLoopControllerPositionWithSpeed(DriveSubsystem driveSubsystem, DoubleSupplier setPosition, DoubleSupplier targetSpeed) {
    return Commands.run (
        () -> leftController.setReference(setPosition.getAsDouble(), ControlType.kPosition, ClosedLoopSlot.kSlot0),
        // () -> rightClosedLoopController.setReference(setPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0),    
        // () -> leftLeader.set(targetSpeed.getAsDouble()),
        // rightLeader.set(targetSpeed.getAsDouble()),
        driveSubsystem
      );
  }

  public void setClosedLoopControllerVelocity(double setVelocity) {
    leftController.setReference(setVelocity, ControlType.kVelocity);
    rightController.setReference(setVelocity, ControlType.kVelocity);    
  }

  // Command to drive the robot with joystick inputs
  public Command driveArcadeCmd(
      DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem)
        .withName("Drive/CMD/driveArcade");
  }

  public Command driveOpenLoopCmd(
      DriveSubsystem subdsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        double leftVolts = xSpeed.getAsDouble() + zRotation.getAsDouble();
        double rightVolts = xSpeed.getAsDouble() +zRotation.getAsDouble();
        return Commands.run(
          (() -> this.runOpenLoop(leftVolts, rightVolts))
        );
  }

  public Command resetEncodersCmd() {
    return this.runOnce(this::resetEncoders)
      .withName("Drive/CMD/Reset Encoders");
  }
  
}
