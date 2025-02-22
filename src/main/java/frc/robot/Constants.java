// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // device IDs
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 1;
    public static final int RIGHT_FOLLOWER_ID = 2;

    // chassis configuration for PathPlanner or simulations
    // FIXME: update assumed numbers for simulation from CAD
    public static final DCMotor gearbox = DCMotor.getCIM(2);
    public static final double gearing = 8.45;   // kitbot gearing: k8p45
    public static final double robotMOI = 7.5;       // moment of inertia of drivetrain in its center
    public static final double robotMassKg = 60.0;   
    public static final double trackWidthInMeters = 0.7112; // distance between right & left wheels

    public static final double wheelRadiusInches = 3.0;
    public static final double wheelDiameterInches = 2 * wheelRadiusInches;
    public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;

    // chassis configuration in meters
    public static final double wheelRadiusMeters = Units.inchesToMeters(wheelRadiusInches);
    public static final double wheelDiameterMeters = Units.inchesToMeters(wheelDiameterInches);
    public static final double wheelCircumferenceMeters = Units.inchesToMeters(wheelCircumferenceInches);

    // speed references meters/sec
    public static final double walkingSpeedMetersPerSec = 1.0;  
    public static final double maxSpeedMetersPerSec = 3.0;  // FIXME
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.44;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final int coralDeliverToReef = 2; 
    public static final int coralDeliverToElevator = 3;
  }
}
