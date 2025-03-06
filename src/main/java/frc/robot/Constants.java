// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public static final double maxSpeedMetersPerSec = 3.0;
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.44;
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_LEADER_MOTOR_ID = 10;
    public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 11;
    public static final int ELEVATOR_ARM_MOTOR_ID = 12;

    public static final int ELEVATOR_LIFT_BOTTOM_LIMIT_SWITCH = 1;       // hard stop for bottom position of lift
    
    // Elevator positions are specified in meters
    public static enum ElevatorPosition {
      BOTTOM(0.0),      // min height will trigger limit switch
      INTAKE(0.35),     // coral intake
      CORAL_L1(0.8),
      CORAL_L2(1.2),
      TOP(1.5);        // max height

      public final double value;

      private ElevatorPosition(double value) {
        this.value = value;
      }
    }

    // Elevator mechanism for simulation
    public static final DCMotor gearbox = DCMotor.getNEO(2);
    public static final double gearing = 12.0;    // 12:1 gearing
    public static final double massKg = Units.lbsToKilograms(20);
    public static final double drumRadiusInMeters = Units.inchesToMeters(1);     // sprocket diameter is 2"
    public static final double drumCircumferenceInMeters = 2.0 * Math.PI * drumRadiusInMeters;
    public static final double encoderRotationsInMeters = drumCircumferenceInMeters / gearing;

    public static final double MIN_HEIGHT_METERS = 0.005;
    public static final double MAX_HEIGHT_METERS = 1.57;

    public static final double FREE_SPEED_VOLTAGE = 1; 
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3; 
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; 

    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED); 
    
  }

  public static final class WpiElevatorLiftConfigs {
    // Linear mechanism values from:  ReCalc (https://www.reca.lc/linear)
    public static final double simkP = 10;
    public static final double simkI = 0.01;
    public static final double simkD = 0.01;
    public static final double simkS = 0.095388;
    public static final double simkG = 0.34;
    public static final double simkV = 9.21;
    public static final double simkA = 0.04;
    
    public static final double TOLERANCE = 0.05;  // tolerance @ 5cm
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final int coralDeliverToReef = 2; 
    public static final int coralDeliverToElevator = 3;

    public static final int elevatorToBottom = 4;
    public static final int elevatorToCoralIntake = 5;
    public static final int elevatorToL1 = 6;
    public static final int elevatorToL2 = 7;
    public static final int elevatorToTop = 8;
    public static final int resetLiftToBottomPosition = 9;
    
  }
}
