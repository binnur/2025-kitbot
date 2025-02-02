package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RollerConstants;

public final class MotorConfigs {

    // Configure drive subsystem
    public static final class DriveSubsystemConfigs {
        public static final SparkMaxConfig globalConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
        public static final  SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

        // motor config settings
        public static final double kDriveMotorNominalVoltage = 12.0;
        public static final int kDriveMotorCurrentLimit = 60;
        public static final int kDriveCANTimeout = 250; // in milliseconds
 
        // close loop controller conversion factors -- FIXME: getEncoder() returns in inches ? Velocity??
        public static final int kEncoderCoundsPerRevolution = 2048 * 4;
        public static final double kDrivePositionConversionFactor = kEncoderCoundsPerRevolution / DriveConstants.kWheelCircumferenceInches;
        public static final double kDriveVelocityConversionFactor = kEncoderCoundsPerRevolution / DriveConstants.kWheelCircumferenceInches;

        static {
            // Create the globalConfiguration to apply to motors. Voltage compensation
            // helps the robot perform more similarly on different
            // battery voltages (at the cost of a little bit of top speed on a fully charged
            // battery). The current limit helps prevent tripping breakers.
            globalConfig
                .voltageCompensation(DriveSubsystemConfigs.kDriveMotorNominalVoltage)
                .smartCurrentLimit(DriveSubsystemConfigs.kDriveMotorCurrentLimit)
                .idleMode(IdleMode.kBrake);      
            
            // Apply global config to the motor configurations
            leftLeaderConfig.apply(globalConfig);
            rightLeaderConfig.apply(globalConfig);
            leftFollowerConfig.apply(globalConfig);
            rightFollowerConfig.apply(globalConfig);

            // configure encoders
            // velocityConversionFactor returns rotations per min -- divide by 60.0 for rotation per seconds
            leftLeaderConfig.encoder
                .positionConversionFactor(DriveSubsystemConfigs.kDrivePositionConversionFactor)
                .velocityConversionFactor(DriveSubsystemConfigs.kDriveVelocityConversionFactor / 60.0);
                rightLeaderConfig.encoder
                .positionConversionFactor(DriveSubsystemConfigs.kDrivePositionConversionFactor)
                .velocityConversionFactor(DriveSubsystemConfigs.kDriveVelocityConversionFactor / 60.0);

            // configure closed loop controllers
            leftLeaderConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // set PID values for position control. Closed loop slot defaults to slot 0
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                // set PID values for velocity control to slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                // FIXME: magic numbers! 
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
            rightLeaderConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // set PID values for position control. Closed loop slot defaults to slot 0
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                // set PID values for velocity control to slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                // FIXME: magic numbers! 
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        }
    }

    // Configure roller subsystem
    public static final class RollerSubsystem {
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
