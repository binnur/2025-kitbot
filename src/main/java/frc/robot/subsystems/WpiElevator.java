package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.MotorConfigs.ElevatorSubsystemConfigs;
import frc.robot.Constants.WpiElevatorLiftConfigs; 

/** 
 * Elevator & arm functions -- interfaces & functionality levaraged from 2025-Ri3D RustHounds 
 * and REV Kitbot 2025

 High-level approach
 * 1. define & configure the motors+controllers
 * 2. setup 'zero' baseline for lift & arm for move to controls
 * 3. specify desired targets (lift levels, arm angles)
 * 4. write code to track to the target goals
 * 5. add simulation and debug the behavior until looks right :) 
 * 
 * General logic flow
 *  1. operator specifies a target, ex. move to 'CORAL_L1'
 *  2. setTargetPositionCommand() decodes the target value and calls executes moveToPositionCommand()
 *  3. moveToPosition() translates the target value to setPoint goal and executes moveToSetPointCommand() until at target
 *  4. moveToSetPointCommand computes feedback & feedforward voltage to apply to motors
 * notes:
 *  -- instead of sisID, used reCalc for feedforward values (https://www.reca.lc/linear)
 */
@Logged
public class WpiElevator extends SubsystemBase {
    // limit switch for bottom position of the lift
    @Logged(name="Lift LimitSwitch")
    DigitalInput bottomLiftLimitSwitch = new DigitalInput(ElevatorConstants.ELEVATOR_LIFT_BOTTOM_LIMIT_SWITCH);

    /* Elevator consists of elevatorMotor for position & armMotor that pivots for coral placement */
    @Logged(name="Lift Motor")
    private final SparkMaxSendable liftMotor = new SparkMaxSendable(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxSendable liftFollowerMotor = new SparkMaxSendable(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
    //@Logged(name="Arm Motor")
    private final SparkMaxSendable armMotor = new SparkMaxSendable(ElevatorConstants.ELEVATOR_ARM_MOTOR_ID, MotorType.kBrushless);

    // setup closed loop controller
    private ProfiledPIDController liftPidController = new ProfiledPIDController(WpiElevatorLiftConfigs.simkP, 
                                                    WpiElevatorLiftConfigs.simkI, 
                                                    WpiElevatorLiftConfigs.simkD,
                                                    ElevatorConstants.MOVEMENT_CONSTRAINTS);

    private final ElevatorFeedforward liftFFController = new ElevatorFeedforward(WpiElevatorLiftConfigs.simkS, 
                                                    WpiElevatorLiftConfigs.simkG,
                                                    WpiElevatorLiftConfigs.simkV,
                                                    WpiElevatorLiftConfigs.simkA);

    private double feedbackVoltage = 0;
    private double feedforwardVoltage = 0;

    @Logged(name="Elevator: Lift IOInfo")
    private final ElevatorIOInfo ioInfo = new ElevatorIOInfo();
    @Logged
    public static class ElevatorIOInfo {
        public double liftAtPositionInMeters = 0.0;
        public double liftDesiredPositionInMeters = ElevatorPosition.BOTTOM.value;
        public double liftSimVelocityInMetersPerSec = 0.0;
        public double liftVelocityInMetersPerSec = 0.0;
        public double liftAppliedVolts = 0.0;
        public double liftCurrentAmps = 0.0;
    }

    /**
     * The representation of the "elevator" for simulation. (even though this is a
     * rotational mechanism w.r.t. its setpoints, we still control it as a linear
     * mechanism since that is the cloest physical mechanism to this)
     */
    private final ElevatorSim elevatorSim = new ElevatorSim(
            ElevatorConstants.gearbox,                  // number of motors in the gearbox
            ElevatorConstants.gearing,                  // elevator gearing                  
            ElevatorConstants.massKg,                   // carriage weight
            ElevatorConstants.drumRadiusInMeters,       // radius of drum where elevator spool is wrapped around
            ElevatorConstants.MIN_HEIGHT_METERS,
            ElevatorConstants.MAX_HEIGHT_METERS,
            true,
            ElevatorPosition.BOTTOM.value);


    // setup simulation support
    private DCMotor dcMotorLift = DCMotor.getNEO(2);
    private SparkMaxSim simDcMotorLift;

    public WpiElevator() {
        // Apply configurations to the motors
        // kResetSafeParameters puts SPARKMAX to a known state (if sparkmax is replaced)
        // kPersistParameters ensure configuration is not lost when sparkmax looses power (ex. mid-operation power cycles)
        liftMotor.configure(ElevatorSubsystemConfigs.liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        liftFollowerMotor.configure(ElevatorSubsystemConfigs.liftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotor.configure(ElevatorSubsystemConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // construct simulation 
        simDcMotorLift = new SparkMaxSim(liftMotor, dcMotorLift);

        // setup the elevator baseline
        moveToBottom();        // Reset elevator and arm position to its baseline
    }

    private void updateElevatorIOInfo() {
        ioInfo.liftAtPositionInMeters = liftMotor.getEncoder().getPosition();
        ioInfo.liftVelocityInMetersPerSec = liftMotor.get();     // note: does not get updated during simulation use corresponding liftSimVelocity
        ioInfo.liftAppliedVolts = liftMotor.getAppliedOutput();
        ioInfo.liftCurrentAmps = liftMotor.getOutputCurrent();
        // note: ioInfo.liftDesiredPositionMeters updated with the operator control commands

        if (Robot.isSimulation()) {
            ioInfo.liftSimVelocityInMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        }
    }

    /**
     * Moves the lift to the bottom position of the elevator
     */
    private void moveToLimitSwitch() {
        while (bottomLiftLimitSwitch.get() != true) {
            liftMotor.setVoltage(-ElevatorConstants.FREE_SPEED_VOLTAGE);   // move elevator down
        }
    }

    /**
     * Resets the encoders make sure hard stop is reached first
     */
    private void resetEncoders() {
        liftMotor.getEncoder().setPosition(0.0);

        if (Robot.isSimulation()) {
            simDcMotorLift.setPosition(0.0);
        }
    }

    @Override
    public void periodic() {
        // note: default command moveToSetPointCommand() automatically runs
        updateElevatorIOInfo();
        SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {
        // note: using liftMotor (instead of simDCMotorLift) provided better results
        //elevatorSim.setInput(simDcMotorLift.getAppliedOutput() * RobotController.getInputVoltage());
        elevatorSim.setInput(liftMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        elevatorSim.update(0.020);

        // note: liftVelocityInMetersPerSec values are not updated via iterate -- liftMoter.get() returns 0
        simDcMotorLift.iterate(
            elevatorSim.getVelocityMetersPerSecond(),       // see liftConfig conversion factors
            RobotController.getBatteryVoltage(),
            0.02);

        // note: .iterate should reset encoder position, but this seem to simulate better
        liftMotor.getEncoder().setPosition(elevatorSim.getPositionMeters());

        updateElevatorIOInfo();
    }

    /**
     * Resets the position of the mechanism to a specific value (this should be the
     * position of a hard stop).
     * TODO: move the elevator till triggered limitswitch
     */
    public void moveToBottom() {
        ioInfo.liftDesiredPositionInMeters = ElevatorPosition.BOTTOM.value;

        // drive elevator to bottom until limit switch is tripped
        moveToLimitSwitch();
        resetEncoders();

        // initialize the PID controller to goal position of BOTTOM
        liftPidController.reset(getPosition());
        liftPidController.setGoal(ElevatorPosition.BOTTOM.value);
    }

    /**
     * Gets the position of the mechanism. 0 should be at the lowest movement point,
     * and the position should increase as the mechanism moves up.
     * Returns the position of the mechanism in meters
     */
    public double getPosition() {
        return liftMotor.getEncoder().getPosition();        // maps to ioInfo.liftAtPositionInMeters
    }
 
    /**
     * Explicit function to set the voltage of the motors attached to the linear
     * mechanism,
     * Handle safeties and clamping here.
     */
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);

        if ((voltage > 0.0 && getPosition() >= ElevatorConstants.MAX_HEIGHT_METERS) ||
            (voltage < 0.0 && getPosition() <= ElevatorConstants.MIN_HEIGHT_METERS)) {
               voltage = 0.0;
        }

        liftMotor.setVoltage(voltage);
    }

    /**
     * Creates a command that continuously applies voltage to the motor controllers
     * to move them to the currently set goal.
     */
    public Command moveToSetPointCommand() {
        return run( () -> {
            feedbackVoltage = liftPidController.calculate(getPosition());
            feedforwardVoltage = liftFFController.calculate(liftPidController.getSetpoint().velocity);

            // TODO: replace this code with Math.signum function
            // feedforwardVoltage = 0.0;
            // if (feedbackVoltage < 0) {
            //     if (feedbackVoltage > -1) {
            //         feedforwardVoltage = -1;
            //     }
            // }
            // else if (feedbackVoltage > 0) {
            //     if (feedbackVoltage < 1) {
            //         feedforwardVoltage = 1;
            //     }
            // }

            setVoltage(feedbackVoltage+feedforwardVoltage);
        }).withName("elevator.moveToCurrentGoal");
    }

    /**
     * Returns if lift limit switch is triggered
     * Resets encoders if true
     */
    public boolean didTriggerLiftLimitSwitch() {
        if (bottomLiftLimitSwitch.get()) {
            resetEncoders();
        }
        return bottomLiftLimitSwitch.get();
    }

    /*
     * Returns if target goal is reached within tolerance specified
     * TODO: check if triggered limit switch, i.e. moving passed min elevator position
     */
    public boolean liftAtGoal() {
        return liftPidController.atGoal();
    }

    /**
     * Creates a command that sets target setpoints for the elevator & arm. 
     * Command is cancelled once the mechanism has reached that goal.
     * TODO: add arm controls
     */
    public Command moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        ioInfo.liftDesiredPositionInMeters = goalPositionSupplier.get().value;

        // stop current motion and clear integral values
        // specify the new goal position to the PID controller 
        liftPidController.reset(getPosition());
        liftPidController.setGoal((goalPositionSupplier.get().value));

        // run the motors until target goal is reached
        return Commands.sequence(
                            moveToSetPointCommand()
                                .until( () -> liftAtGoal() || didTriggerLiftLimitSwitch() ) 
                        )
                        .withTimeout(3)
                        .withName("elevator.moveToPosition");
    }

    /**
     * Creates an instantaneous command that resets the position of the linear
     * mechanism.
     */
    public Command resetPositionCommand() {
        return runOnce(this::moveToBottom).withName("elevator.resetPosition");
    }

    /**
     * Command to set the target setpoints for arm & elevator. This will set the arm and elevator to their predefined
     * positions for the given setpoint.
     */ 
    public Command setTargetPositionCommand(ElevatorPosition level) {
        ElevatorPosition liftLevelTarget;

         // TODO add arm position & command to setup arm
        switch (level) {
            case INTAKE:
                liftLevelTarget = ElevatorPosition.INTAKE;
                break;
            case TOP:
                liftLevelTarget = ElevatorPosition.TOP;
                break;
            case CORAL_L1, CORAL_L2, BOTTOM:
                liftLevelTarget = ElevatorPosition.BOTTOM;
                break;
            default: 
                liftLevelTarget = ElevatorPosition.BOTTOM;
                break;   
        }

        return Commands.runOnce( 
            () -> {
                moveToPositionCommand( () -> liftLevelTarget);
            }); 
    }

    /**
     * Creates a command stops the motor and sets it to coast mode, to allow for
     * moving the mechanism manually.
     */
    // FIXME should we use break or coast?
    //public Command coastMotorsCommand();

}