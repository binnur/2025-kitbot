// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 @Logged
public class RobotContainer {
  // The robot's subsystems
  @Logged(name ="DriveSystem")
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  @Logged(name ="RollerSubsystem")
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();      // add trigger-> command mappings 
    configureAutoChooser();   // add autonomous options
    SmartDashboard.putData("Auto/Cmd Chooser", autoChooser);
    //configDashboardEntries(); // enable dashboard, which also adds autoChooser
   }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Set the A button to run the "runRoller" command from the factory with a fixed
    // value ejecting the gamepiece while the button is held
    operatorController.a()
        .whileTrue(rollerSubsystem.runRoller(rollerSubsystem, () -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveArcadeCmd(
            driveSubsystem, () -> -driverController.getLeftY(), () -> -driverController.getRightX()));

    // Set the default command for the roller subsystem to the command from the
    // factory with the values provided by the triggers on the operator controller
    // rollerSubsystem.setDefaultCommand(
    //     rollerSubsystem.runRoller(
    //         rollerSubsystem,
    //         () -> operatorController.getRightTriggerAxis(),
    //         () -> operatorController.getLeftTriggerAxis()));
    rollerSubsystem.setDefaultCommand(rollerSubsystem.runRollerStop());


  }

  // private void configDashboardEntries() {
  //   SmartDashboard.putData(autoChooser);
  // }

  private void configureAutoChooser() {
    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Do Nothing", Autos.doNothing());
    autoChooser.addOption("Drive Forward", Autos.driveFwdOpenLoopCmd(driveSubsystem));
    autoChooser.addOption("Arcade Drive (no rotation @ 50%)", Autos.driveArcadeCmd(driveSubsystem));
    autoChooser.addOption("Drive FWD 3 meeters", Autos.driveFwd3meters(driveSubsystem));
    autoChooser.addOption("Reset Encoders",  Autos.resetEncoders(driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

}
