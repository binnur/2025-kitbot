package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RollerConstants;

public final class MotorConfigs {

    // Configure drive subsystem
    public static final class CANDriveSubsystemConfigs {
        // motor config settings
        public static final double kDriveMotorNominalVoltage = 12.0;
        public static final int kDriveMotorCurrentLimit = 60;
        public static final int kDriveCANTimeout = 250; // in milliseconds
 
        // close loop controller conversion factors -- FIXME: getEncoder() returns in inches ? Velocity??
        public static final int kEncoderCoundsPerRevolution = 2048 * 4;
        public static final double kDrivePositionConversionFactor = kEncoderCoundsPerRevolution / DriveConstants.kWheelCircumferenceInches;
        public static final double kDriveVelocityConversionFactor = kEncoderCoundsPerRevolution / DriveConstants.kWheelCircumferenceInches;

        static {

        }
    }

    // Configure roller subsystem
    public static final class CANRollerSubsystem {
        public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();

        static {
            // Apply configuration for roller motor. Voltage compensation helps
            // the roller behave the same as the battery voltage dips. The current limit helps 
            // prevent breaker trips or burning out the motor in the event the roller stalls.
            rollerConfig
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP)
                .smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
        }
    }
}
