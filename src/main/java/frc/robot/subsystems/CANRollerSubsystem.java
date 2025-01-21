// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.RollerConstants;


/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {
  // Initialize roller SPARK
  private final SparkMaxSendable rollerMotor = 
    new SparkMaxSendable(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushed);

  // subsystem internals
  double speed = 0.0;

  public CANRollerSubsystem() {
    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    rollerMotor.setCANTimeout(250);

    rollerMotor.configure(
      Configs.CANRollerSubsystem.rollerConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    // Add Live Window -- used in Test mode
    addChild("rollerMotor", rollerMotor);

    // Add subsystem components for dashboard
    configDashboardKeys();
  }

  private void configDashboardKeys() {
    SmartDashboard.putData("Roller/rollerMotor", rollerMotor);
  }

  @Override
  public void periodic() {
  }

  // Command to run the roller with joystick inputs
  public Command runRoller(
      CANRollerSubsystem rollerSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.run(
        () -> rollerMotor.set(forward.getAsDouble() - reverse.getAsDouble()), rollerSubsystem);
  }



}
