// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANRollerSubsystem;
import frc.robot.Constants.RollerConstants;

public final class Autos {
  // Do Nothing
  public static final Command doNothing() {
    return null;
  }

  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(CANDriveSubsystem driveSubsystem) {
    return driveSubsystem.driveArcade(driveSubsystem, () -> 0.5, () -> 0.0).withTimeout(1.0);
  }

  public static final Command exampleAutoDriveAndRoll(CANDriveSubsystem driveSubsystem, CANRollerSubsystem rollerSubsystem) {
    driveSubsystem.driveArcade(driveSubsystem, () -> 0.5, () -> 0.0).withTimeout(1.0);
    return rollerSubsystem.runRoller(rollerSubsystem, ()-> RollerConstants.ROLLER_EJECT_VALUE, () -> 0);
  }
}
