// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.RollerConstants;
import frc.robot.ShuffleboardConfig.RollerTabManager;
//import frc.robot.ShuffleboardConfig.RollerTabManager.RollerSubsystemEntries;


/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {

  public static enum RollerState {
    STOPPED,
    FORWARD,
    REVERSE
  };

  private final SparkMax rollerMotor;
    
  private RollerState rollerState = RollerState.STOPPED;
  
  public static ShuffleboardTab debugTab;

  //private ShuffleboardLayout overviewWidget;
  GenericEntry rollerStateWidget, targetSpeedWidget, currentSpeedWidget, cmdWidget;
  double targetSpeed, speed;

  public CANRollerSubsystem() {
    runRollerStop();    // default behavior for subsysystem is STOPPED

    // Set up the roller motor as a brushed motor
    rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushed);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    rollerMotor.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Setup dashboard for roller subsystem
    // TODO: add support for Sendable to enable Live Window
    // addChild() organizes Live Window easy access to subsystem components
    // addChild("Roller motor", rollerMotor);
    addSubsystemDashboardWidgets(this);   
  }

  @Override
  public void periodic() {
    updateDashboard();
  }

  public RollerState getRollerState() {
    return rollerState;
  }

  // Command to run the roller with joystick inputs
  // FIXME: associate w/ RollerState
  public Command runRoller(
      CANRollerSubsystem rollerSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.run(
        () -> rollerMotor.set(forward.getAsDouble() - reverse.getAsDouble()), rollerSubsystem);
  }

  // Run roller with set speed
  private void runRollerMotors() {
    if (speed > 0) 
      rollerState = RollerState.FORWARD; 
    else 
      rollerState = RollerState.REVERSE;
    rollerMotor.set(speed);
  }

  private void stopRollerMotors() {
    rollerState = RollerState.STOPPED;
    rollerMotor.set(0.0);
  }

  public Command runRollerForward() {
    return this.startEnd(this::runRollerMotors, this::stopRollerMotors)
              .withName("runRollerForward");
  }

  public Command runRollerReverse() {
    return this.startEnd(this::runRollerMotors, this::stopRollerMotors)
              .withName("runRollerReverse");
  }

  public Command runRollerStop() {
    return this.runOnce(this::stopRollerMotors)
              .withName("runRollerStop");
  }

  /*
   * Setup the dashboard during the subsystem construction phase
   */
  private void addSubsystemDashboardWidgets(CANRollerSubsystem subsystem) {
    // Use the defaultTab set for the subsystem
    ShuffleboardLayout shuffleboardWidget = RollerTabManager.getLayout();
    
    // add commands
    shuffleboardWidget.add(subsystem);
    shuffleboardWidget.add(runRollerForward());
    shuffleboardWidget.add(runRollerReverse());
    shuffleboardWidget.add(runRollerStop());

    // add widgets
    rollerStateWidget = shuffleboardWidget.add("State", rollerState.name() )
         .withWidget(BuiltInWidgets.kTextView)
         .getEntry();
    
    currentSpeedWidget = shuffleboardWidget.add("Speed", speed)
         .withWidget(BuiltInWidgets.kTextView)
         .getEntry();

    targetSpeedWidget = shuffleboardWidget.add("TargetSpeed", 0.0)
         .withWidget(BuiltInWidgets.kNumberSlider)
         .withProperties(Map.of("min",-1, "max", 1))
         .getEntry();
  }

  /*
   * Update the dashboard values periodically
   * Note: Shuffleboard app had issue updating the widgets appropriately (read-back issue?)
   * Demonstrated the workings of the dashboard via Glass app
   */
  public void updateDashboard() {
    rollerStateWidget.setString(rollerState.name());
    speed = targetSpeedWidget.getDouble(targetSpeed);
    currentSpeedWidget.setDouble(speed);
  }

}
