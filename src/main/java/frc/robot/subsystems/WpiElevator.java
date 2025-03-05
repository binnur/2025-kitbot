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

/* 
 * 1. define the motors+controllers
 * 2. configure the motors+controllers
 * 3. define the expected behavior
 * 
 * 
 * ... add simulation
 * ... add sysID routines
 */
@Logged
public class WpiElevator extends SubsystemBase {
    /* Elevator consists of elevatorMotor for position & armMotor that pivots for coral placement */
    //@Logged(name="Elevator Motor")
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

    //@Logged(name="Lift Feedback Control")
    private double feedbackVoltage = 0;
    //@Logged(name="Lift Feedforward Control")
    private double feedforwardVoltage = 0;
    private double simVelocity = 0.0;
    private double desiredVoltageForSetPoint = 0.0;         // FIXME: this is temporary for logging
    private double liftToDesiredPositionInMeters = 0.0;

    @Logged(name="ElevatorIOInfo")
    private final ElevatorIOInfo ioInfo = new ElevatorIOInfo();
    @Logged
    public static class ElevatorIOInfo {
        public double liftAtPositionInMeters = 0.0;
        public double liftDesiredPositionInMeters = ElevatorPosition.BOTTOM.value;
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
            ElevatorConstants.gearbox,
            ElevatorConstants.gearing,
            ElevatorConstants.massKg,
            ElevatorConstants.drumRadiusInMeters,
            ElevatorConstants.MIN_HEIGHT_METERS,
            ElevatorConstants.MAX_HEIGHT_METERS,
            true,
            ElevatorPosition.BOTTOM.value);


    // setup simulation support
    private DCMotor dcMotorLift = DCMotor.getNEO(2);
    private DCMotor dcMotorArm;
    private SparkMaxSim simDcMotorLift;
    private SparkMaxSim simDcMotorArm;

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
        moveToBottom();        // default elevator position is at BOTTOM & reset encoders
        setDefaultCommand(moveToCurrentGoalCommand());      // FIXME: with this, don't need to specify periodic?

      }

    @Override
    public void periodic() {
        // FIXME
        //moveToCurrentGoalCommand();
        updateElevatorIOInfo();
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(simDcMotorLift.getAppliedOutput() * RobotController.getInputVoltage());
        elevatorSim.update(0.020);

        simDcMotorLift.iterate(
            // FIXME velocity in rotations?
            ((elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.drumCircumferenceInMeters)
                    * ElevatorConstants.gearing) * 60.0,
            RobotController.getBatteryVoltage(),
            0.02);

        //liftMotor.getEncoder().setPosition(elevatorSim.getPositionMeters());
        simVelocity = elevatorSim.getVelocityMetersPerSecond();

        // sim values are not updated via iterate -- liftMoter.get() returns 0
        simDcMotorLift.setVelocity(elevatorSim.getVelocityMetersPerSecond());
        updateElevatorIOInfo();
    }

    private void updateElevatorIOInfo() {
        ioInfo.liftAtPositionInMeters = liftMotor.getEncoder().getPosition();
        ioInfo.liftVelocityInMetersPerSec = liftMotor.get();
        ioInfo.liftAppliedVolts = liftMotor.getAppliedOutput();
        ioInfo.liftCurrentAmps = liftMotor.getOutputCurrent();
        ioInfo.liftDesiredPositionInMeters = liftToDesiredPositionInMeters;
    }

    // FIXME: resetEncoder to 0 vs. BOTTOM value of elevator?
    private void resetEncoders() {
        liftMotor.getEncoder().setPosition(0.0);

        if (Robot.isSimulation()) {
            simDcMotorLift.setPosition(0.0);
        }
    }

    // The interfaces pulled from 2025-Ri3D RustHounds BaseLinearMechanism interface

    /**
     * Resets the position of the mechanism to a specific value (this should be the
     * position of a hard stop).
     * TODO: move the elevator till triggered limitswitch
     */
    public void moveToBottom() {
        ioInfo.liftDesiredPositionInMeters = ElevatorPosition.BOTTOM.value;
        // TODO: drive the lift motor until limit switch is triggered
        // initialize the PID controller to goal position of BOTTOM
        resetEncoders();
        liftPidController.reset(getPosition());
        liftPidController.setGoal(ElevatorPosition.BOTTOM.value);
    }

    /**
     * Gets the position of the mechanism. 0 should be at the lowest movement point,
     * and the position should increase as the mechanism moves up.
     * 
     * @return the position of the mechanism, in meters
     */
    public double getPosition() {
        return liftMotor.getEncoder().getPosition();        // ioInfo.liftAtPositionInMeters
    }
 
    /**
     * Explicit function to set the voltage of the motors attached to the linear
     * mechanism,
     * should handle safeties and clamping here.
     * 
     * @param voltage the voltage to apply to the motors, [-12, 12]
     */
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        // TODO any further voltage setup as position reaches towards top or bottom?
        // voltage = Utils.applySoftStops(voltage, getPosition(), MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        // if (voltage < 0
        //         && positionTracker.getElevatorPosition() < Constants.Elevator.MOTION_LIMIT
        //         && positionTracker.getArmAngle() < 0) {
        //     voltage = 0;
        // }
        desiredVoltageForSetPoint = voltage;
        liftMotor.setVoltage(voltage);
    }

    /**
     * Creates a command that continuously applies voltage to the motor controllers
     * to move them to the currently set goal.
     * 
     * @return the command
     */
    public Command moveToCurrentGoalCommand() {
        return run( () -> {
            feedbackVoltage = liftPidController.calculate(getPosition());
            //feedforwardVoltage = liftFFController.calculate(liftPidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage+feedforwardVoltage);
        }).withName("elevator.moveToCurrentGoal");
    }

    /**
     * Creates a command that sets the current goal position to the setpoint, and
     * cancels once the mechanism has reached that goal.
     * 
     * @apiNote use {@code moveToCurrentGoalCommand()} internally to avoid code
     *          duplication
     * 
     * @param goalPositionSupplier a supplier of an instance of the setpoint enum
     * @return the command
     */
    public Command moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        System.out.println("Updating lift position: " + goalPositionSupplier.get().value);
        ioInfo.liftDesiredPositionInMeters = goalPositionSupplier.get().value;
        liftToDesiredPositionInMeters = goalPositionSupplier.get().value;

        // stop motion: clear integral values & set PID controller to current position
        liftPidController.reset(getPosition());
        // specify the new goal position for PID controller
        liftPidController.setGoal((goalPositionSupplier.get().value));

        return Commands.sequence(
                            moveToCurrentGoalCommand()
                                .until( () -> liftAtGoal() ) 
                        )
                        .withTimeout(3)
                        .withName("elevator.moveToPosition");

    //     return Commands.sequence(
    //             // stop motion: clear integral values & set PID controller to current position
    //             runOnce(() -> liftPidController.reset(getPosition())), 
    //             // specify the new goal position for PID controller
    //             runOnce( () -> {
    //                 System.out.println("PID to goal position: " + goalPositionSupplier.get().value);
    //                 liftPidController.setGoal(goalPositionSupplier.get().value);
    //             }),
    //             moveToCurrentGoalCommand()
    //                     .until(() -> liftAtGoal()))
    //         .withTimeout(3)
    //         .withName("elevator.moveToPosition");
     }

    /**
     * Creates an instantaneous command that resets the position of the linear
     * mechanism.
     * 
     * @return the command
     */
    public Command resetPositionCommand() {
        return runOnce(this::moveToBottom).withName("elevator.resetPosition");
    }

    /**
     * Creates a command stops the motor and sets it to coast mode, to allow for
     * moving the mechanism manually.
     * 
     * @apiNote use
     *          {@code .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)}
     *          for safety
     * @return the command
     */
    // FIXME should we use break or coast?
    //public Command coastMotorsCommand();


    public boolean liftAtGoal() {
        // if (liftPidController.atGoal()) {
        //     System.out.println("Elevator At Goal!!!!");
        // } else {
        //     System.out.println("WORKING*****");
        // }
        return liftPidController.atGoal();
    }

    /**
     * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
     * positions for the given setpoint.
     */
    // FIXME 
    public Command setPositionCommand(ElevatorPosition level) {
        ElevatorPosition liftLevelTarget;

        // ArmPosition armPosition;
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

        // TODO add arm commands
        return Commands.runOnce( 
            () -> {
                moveToPositionCommand( () -> liftLevelTarget);
            }); 
    }
}