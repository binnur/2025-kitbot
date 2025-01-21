package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.RollerConstants;

public final class Configs {

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
