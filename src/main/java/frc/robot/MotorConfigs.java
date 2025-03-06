package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RollerConstants;

public final class MotorConfigs {

    // Configure drive subsystem
    public static final class DriveSubsystemConfigs {
        public static final SparkMaxConfig globalConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

        // motor configurations
        public static final int currentLimit = 60;
        public static final double nominalVoltage = 12.0;
        public static final int canBusTimeout = 250; // in milliseconds

        public static final boolean leftInverted = false;
        public static final boolean rightInverted = true;

        // velocity PID configurations
        // note: these values are taken from KitBot (AdvantageKit template) and should be updated to match our robot
        public static final double simKp = 0.1; // was: 0.0001
        public static final double simKd = 0.0;
        public static final double simKi = 0.0;
    
        // close loop controller conversion factors
        public static final int kEncoderCountsPerRevolution = 2048 * 4;     // one rotation is 8192 ticks of the hardware encoder
        public static final double kDrivePositionConversionFactor = DriveConstants.wheelCircumferenceMeters / kEncoderCountsPerRevolution;
        public static final double kDriveVelocityConversionFactor = DriveConstants.wheelCircumferenceMeters / kEncoderCountsPerRevolution;     // RPM (per minute)

        static {
            // Create the globalConfiguration to apply to motors. Voltage compensation
            // helps the robot perform more similarly on different
            // battery voltages (at the cost of a little bit of top speed on a fully charged
            // battery). The current limit helps prevent tripping breakers.
            globalConfig
                .voltageCompensation(DriveSubsystemConfigs.nominalVoltage)
                .smartCurrentLimit(DriveSubsystemConfigs.currentLimit)
                .idleMode(IdleMode.kBrake);      
            
            // Apply global config to the motor configurations
            leftLeaderConfig.apply(globalConfig);
            rightLeaderConfig.apply(globalConfig);
            leftFollowerConfig.apply(globalConfig);
            rightFollowerConfig.apply(globalConfig);

            // Apply if inverted to leaders
            leftLeaderConfig.inverted(DriveSubsystemConfigs.leftInverted);
            rightLeaderConfig.inverted(DriveSubsystemConfigs.rightInverted);

            // configure encoders
            // velocityConversionFactor returns rotations per min -- divide by 60.0 for rotation per seconds
            // FIX?: in setReference() left encoder is counting backwards -- multiplying by -1
            leftLeaderConfig.encoder
                .positionConversionFactor(-1*DriveSubsystemConfigs.kDrivePositionConversionFactor)
                .velocityConversionFactor(DriveSubsystemConfigs.kDriveVelocityConversionFactor / 60.0);
            rightLeaderConfig.encoder
                .positionConversionFactor(-1*DriveSubsystemConfigs.kDrivePositionConversionFactor)
                .velocityConversionFactor(DriveSubsystemConfigs.kDriveVelocityConversionFactor / 60.0);

            // configure closed loop controllers for velocity -- by default written to slot 0
            leftLeaderConfig.closedLoop
                // set PID values for position control. Closed loop slot defaults to slot 0
                .p(DriveSubsystemConfigs.simKp)
                .i(DriveSubsystemConfigs.simKi)
                .d(DriveSubsystemConfigs.simKd)
                .velocityFF(1.0 / DriveSubsystemConfigs.kEncoderCountsPerRevolution) 
                .outputRange(-1, 1);
 
            rightLeaderConfig.closedLoop
                .p(DriveSubsystemConfigs.simKp)
                .i(0.0)
                .d(DriveSubsystemConfigs.simKd)
                .velocityFF(1.0 / DriveSubsystemConfigs.kEncoderCountsPerRevolution)
                .outputRange(-1, 1);
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

    public static final class ElevatorSubsystemConfigs {
        public static final SparkMaxConfig globalConfig = new SparkMaxConfig();
        public static final SparkMaxConfig liftConfig = new SparkMaxConfig();
        public static final SparkMaxConfig liftFollowerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig armConfig = new SparkMaxConfig();
     
        // motor configurations
        public static final int currentLimit = 50;                  // per REV 2025 code
        public static final double nominalVoltage = 12.0;
        public static final int canBusTimeout = 250; // in milliseconds

        public static final boolean motorLiftInverted = false;
        public static final boolean motorArmInverted = false;

        // close loop controller conversion factors
        public static final int encoderCountsPerRevolution = 42 * (int)ElevatorConstants.gearing;     // one rotation is 8192 ticks of the hardware encoder
        public static final double liftPositionConversionFactor = ElevatorConstants.drumCircumferenceInMeters / encoderCountsPerRevolution;
        public static final double liftVelocityConversionFactor = ElevatorConstants.drumCircumferenceInMeters / encoderCountsPerRevolution;     // RPM (per minute)
        

        static {
            globalConfig
                .voltageCompensation(ElevatorSubsystemConfigs.nominalVoltage)
                .smartCurrentLimit(ElevatorSubsystemConfigs.currentLimit)
                .idleMode(IdleMode.kBrake); 
            
            liftConfig.apply(globalConfig);
            liftFollowerConfig.apply(globalConfig);
            armConfig.apply(globalConfig);

            // Apply if inverted to motors
            liftConfig.inverted(ElevatorSubsystemConfigs.motorLiftInverted);
            armConfig.inverted(ElevatorSubsystemConfigs.motorArmInverted);

            liftConfig.encoder
                 .positionConversionFactor(liftPositionConversionFactor)
                 .velocityConversionFactor(liftVelocityConversionFactor / 60);      // RPS

            // Configure closedLoop controls -- used with REVlib for control loop
        }
    }
}
