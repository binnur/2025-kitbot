// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.MotorConfigs;
import frc.robot.Constants.RollerConstants;


/** Class to run the rollers over CAN */
@Logged
public class RollerSubsystem extends SubsystemBase {
  // state of the roller motor
  public static enum RollerState {
    STOPPED,
    FORWARD,
    REVERSE
  };

  @Logged(name="RollerMotorInfo")
  private final MotorIOInfo ioInfo = new MotorIOInfo();
  @Logged
  public static class MotorIOInfo {
    public double motorPositionInMeters = 0.0;
    public double motorVelocityInMetersPerSec = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
  }

  // Initialize roller SPARK
  // FIXME
  private final SparkMaxSendable rollerMotor = 
    new SparkMaxSendable(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushed);

  // subsystem internals
  private double speed = 0.5;
  private RollerState rollerState = RollerState.STOPPED;

  // Debouncer for current stall detection
  LinearFilter currenFilter = LinearFilter.movingAverage(10);
  private double filteredCurrent;

  public RollerSubsystem() {
    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    rollerMotor.setCANTimeout(250);

    rollerMotor.configure(
      MotorConfigs.RollerSubsystem.rollerConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    // Add Live Window -- used in Test mode
    addChild("Roller/rollerMotor", rollerMotor);

    // Add subsystem components for dashboard
    //configDashboardEntries();
  }

  // private void configDashboardEntries() {
  //   SmartDashboard.putData("Roller/rollerMotor", rollerMotor);
  //   SmartDashboard.putData(runRollerForward());
  //   SmartDashboard.putData(runRollerReverse());
  //   SmartDashboard.putData(runRollerStop());
  //   SmartDashboard.putString("Roller/rollerState", rollerState.name());
  //   SmartDashboard.putNumber("Roller/speed", speed);
  // }

  // private void updateDashboardEntries() {
  //   //SmartDashboard.putData("Roller/rollerMotor", rollerMotor);
  //   SmartDashboard.putString("Roller/rollerState", rollerState.name());
  //   SmartDashboard.putNumber("Roller/speed", rollerMotor.get());
  // }

  private void updateMotorIOInfo() {
    //ioInfo.leftPositionInMeters = motor.getEncoder().getPosition();
    ioInfo.motorVelocityInMetersPerSec = rollerMotor.get();
    ioInfo.motorAppliedVolts = rollerMotor.getAppliedOutput();
    ioInfo.motorCurrentAmps = rollerMotor.getOutputCurrent();
  }

  private void runRollerMotorForward() {
    rollerState = RollerState.FORWARD;
    rollerMotor.set(Math.abs(speed));
  }

  private void runRollerMotorReverse() {
    rollerState = RollerState.REVERSE;
    rollerMotor.set(-Math.abs(speed));
  }

  private void stopRollerMotor() {
    rollerState = RollerState.STOPPED;
    rollerMotor.set(0.0);
  }

  @Override
  public void periodic() {
    //updateDashboardEntries();
    updateMotorIOInfo();
    filteredCurrent = currenFilter.calculate(rollerMotor.getOutputCurrent());
  }

  @Override
  public void simulationPeriodic() {
    updateMotorIOInfo();
  }

  // Command to run the roller with joystick inputs
  public Command runRoller(
      RollerSubsystem rollerSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.run(
        () -> rollerMotor.set(forward.getAsDouble() - reverse.getAsDouble()), rollerSubsystem);
  }

  public Command runRollerForward() {
    return this.startEnd(this::runRollerMotorForward, this::stopRollerMotor)
                .withName("Roller/CMD/runRollerForward");
  }

  public Command runRollerReverse() {
    return this.startEnd(this::runRollerMotorReverse, this::stopRollerMotor)
                .withName("Roller/CMD/runRollerReverse");
  }

  public Command runRollerStop() {
    return this.runOnce(this::stopRollerMotor)
                .withName("Roller/CMD/runRollerStop");
  }

  // Sample command to test out ideas
  public Command runCoralIntakeCmd() {
    Debouncer debounce = new Debouncer(1, Debouncer.DebounceType.kRising);

    // Run roller
    return runOnce(
     () -> {
      // initialize
      debounce.calculate(false);
     })
     .andThen(
      // set intake to coral intaking speed
      run( () -> {
        runRollerMotorForward();
      })
        // wait until current spike is detected for more than 1s
        .until( () -> debounce.calculate(filteredCurrent > 1 )))    // INTAKE_STALL_DETECTION is set to 1
      .finallyDo(
        // reduce power to holding state
       (interrupted) -> {
        rollerMotor.set(0.05); // hold coral speed
       }); 
  }

}
