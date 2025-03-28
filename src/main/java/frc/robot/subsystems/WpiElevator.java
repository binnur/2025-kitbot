package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ArmPosition;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WpiElevatorArmConfigs;
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

    /* Elevator consists of elevatorMotor for position & armMotor that pivots for coral placement 
     *      - liftMotor moves the elevator to its set positions: intake, L1, L2, etc. -- linear movement
     *      - armMotor picks and deliveres the coral, ex. pick from intake, and deliver to L1 -- angular movement
     *  Note: Lift has two motors. Configuring them as a follower did not work (motor directions were fighting to each other). 
     *        Instead, we are feeding these motors separately, and using liftMotor for PID positioning
    */
    @Logged(name="Lift Motor")
    private final SparkMaxSendable liftMotor = new SparkMaxSendable(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxSendable liftFollowerMotor = new SparkMaxSendable(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
    @Logged(name="Arm Motor")
    private final SparkMaxSendable armMotor = new SparkMaxSendable(ElevatorConstants.ELEVATOR_ARM_MOTOR_ID, MotorType.kBrushless);

    // setup closed loop controllers
    private final ProfiledPIDController liftPidController = new ProfiledPIDController(WpiElevatorLiftConfigs.simkP, 
                                                    WpiElevatorLiftConfigs.simkI, 
                                                    WpiElevatorLiftConfigs.simkD,
                                                    ElevatorConstants.MOVEMENT_CONSTRAINTS);

    private final ElevatorFeedforward liftFFController = new ElevatorFeedforward(WpiElevatorLiftConfigs.simkS, 
                                                    WpiElevatorLiftConfigs.simkG,
                                                    WpiElevatorLiftConfigs.simkV,
                                                    WpiElevatorLiftConfigs.simkA);

    private final ProfiledPIDController armPidController = new ProfiledPIDController(WpiElevatorArmConfigs.simkP, 
                                                    WpiElevatorArmConfigs.simkI, 
                                                    WpiElevatorArmConfigs.simkD,
                                                    ElevatorConstants.ARM_MOVEMENT_CONSTRAINTS);

    private final ArmFeedforward armFFController = new ArmFeedforward(WpiElevatorArmConfigs.simkS, 
                                                    WpiElevatorArmConfigs.simkG,
                                                    WpiElevatorArmConfigs.simkV,
                                                    WpiElevatorArmConfigs.simkA);

    private double liftFeedbackVoltage = 0;
    private double liftFeedforwardVoltage = 0;
    private double armFeedbackVoltage = 0;
    private double armFeedforwardVoltage = 0;

    @Logged(name="Elevator: Lift IOInfo")
    private final ElevatorIOInfo ioInfo = new ElevatorIOInfo();
    @Logged
    public static class ElevatorIOInfo {
        // lift info
        public double liftAtPositionInMeters = 0.0;
        public double liftDesiredPositionInMeters = ElevatorPosition.BOTTOM.value;
        public double liftSimVelocityInMetersPerSec = 0.0;
        public double liftVelocityInMetersPerSec = 0.0;
        public double liftAppliedVolts = 0.0;
        public double liftCurrentAmps = 0.0;
        // arm info
        public double armAtPositionInRadians = 0.0;
        public double armDesiredPositionInRadians = ArmPosition.STOW.value;
        public double armVelocityInRotationsPerMin = 0.0;
        public double armAppliedVolts = 0.0;
        public double armCurrentAmps = 0.0;
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

    private DCMotor dcMotorArm = DCMotor.getNEO(1);
    private SparkMaxSim simDcMotorArm;

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            dcMotorArm,
            ElevatorConstants.armGearing,
            SingleJointedArmSim.estimateMOI(
                ElevatorConstants.armLengthInMeters, ElevatorConstants.armMassKg),
            ElevatorConstants.armLengthInMeters,
            ElevatorConstants.MIN_ANGLE_RADIANS,
            ElevatorConstants.MAX_ANGLE_RADIANS,
            true,                       // simulate gravity
            ArmPosition.STOW.value);     // initial position of arm in radius


    public WpiElevator() {
        // Apply configurations to the motors
        // kResetSafeParameters puts SPARKMAX to a known state (if sparkmax is replaced)
        // kPersistParameters ensure configuration is not lost when sparkmax looses power (ex. mid-operation power cycles)
        liftMotor.configure(ElevatorSubsystemConfigs.liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        liftFollowerMotor.configure(ElevatorSubsystemConfigs.liftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotor.configure(ElevatorSubsystemConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // construct simulation 
        simDcMotorLift = new SparkMaxSim(liftMotor, dcMotorLift);
        simDcMotorArm = new SparkMaxSim(armMotor, dcMotorArm);

        // setup the elevator baseline
        moveToBottom();        // Reset elevator and arm position to its baseline
    }

    private void updateElevatorIOInfo() {
        ioInfo.liftAtPositionInMeters = liftMotor.getEncoder().getPosition();
        ioInfo.liftVelocityInMetersPerSec = liftMotor.get();     // note: does not get updated during simulation use corresponding liftSimVelocity
        ioInfo.liftAppliedVolts = liftMotor.getAppliedOutput();
        ioInfo.liftCurrentAmps = liftMotor.getOutputCurrent();
        // note: ioInfo.liftDesiredPositionMeters updated with the operator control commands
        ioInfo.armAtPositionInRadians = armMotor.getEncoder().getPosition();
        // note: ioInfo.armDesiredPositionInRadians updated w/ operator commands
        ioInfo.armVelocityInRotationsPerMin = armMotor.get();
        ioInfo.armAppliedVolts = armMotor.getAppliedOutput();
        ioInfo.armCurrentAmps = armMotor.getOutputCurrent();
        
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
        armMotor.getEncoder().setPosition(0.0);

        if (Robot.isSimulation()) {
            simDcMotorLift.setPosition(0.0);
            simDcMotorArm.setPosition(0.0);
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

        armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        armSim.update(0.020);

        // simDcMotorArm.iterate(
        //         Units.radiansPerSecondToRotationsPerMinute(
        //             armSim.getVelocityRadPerSec() * ElevatorConstants.armGearing
        //         ), 
        //         RobotController.getBatteryVoltage(),
        //         0.020);    

        simDcMotorArm.iterate(
                armSim.getVelocityRadPerSec(),
                RobotController.getBatteryVoltage(),
                0.020);    

        // note: .iterate should reset encoder position, but this seem to simulate better
        armMotor.getEncoder().setPosition(armSim.getAngleRads());

        updateElevatorIOInfo();
    }

    /**
     * Resets the position of the mechanism to a specific value (this should be the
     * position of a hard stop).
     */
    public void moveToBottom() {
        ioInfo.liftDesiredPositionInMeters = ElevatorPosition.BOTTOM.value;
        ioInfo.armDesiredPositionInRadians = ArmPosition.STOW.value;

        // drive elevator to bottom until limit switch is tripped
        moveToLimitSwitch();
        resetEncoders();

        // initialize the PID controller to goal position of BOTTOM
        // liftPidController.reset(getLiftPosition());
        // liftPidController.setGoal(ElevatorPosition.BOTTOM.value);
        // armPidController.reset(getLiftPosition());
        // armPidController.setGoal(ArmPosition.STOW.value);
    }

    /**
     * Gets the position of the mechanism. 0 should be at the lowest movement point,
     * and the position should increase as the mechanism moves up.
     * Returns the position of the mechanism in meters
     */
    public double getLiftPosition() {
        return liftMotor.getEncoder().getPosition();        // maps to ioInfo.liftAtPositionInMeters
    }
    
    public double getArmPosition() {
        return armMotor.getEncoder().getPosition();        // maps to ioInfo.armAtPositionInRotations
    }
    /**
     * Explicit function to set the voltage of the motors attached to the linear
     * mechanism,
     * Handle safeties and clamping here.
     */
    public void setLiftVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);

        if ((voltage > 0.0 && getLiftPosition() >= ElevatorConstants.MAX_HEIGHT_METERS) ||
            (voltage < 0.0 && getLiftPosition() <= ElevatorConstants.MIN_HEIGHT_METERS)) {
               voltage = 0.0;
        }

        liftMotor.setVoltage(voltage);
    }

    public void setArmVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);

        // ensure arm can be safely pivoted, i.e. moved
        // if (voltage < 0
        //         && getArmPosition() < 0
        //         && getLiftPosition() < ElevatorConstants.ARM_PIVOT_LIMIT) {
        //     voltage = 0;
        // }

        armMotor.setVoltage(voltage);
    }

    /**
     * Creates a command that continuously applies voltage to the motor controllers
     * to move them to the currently set goal.
     */
    public Command moveToSetPointCommand() {
        return run( () -> {
            liftFeedbackVoltage = liftPidController.calculate(getLiftPosition());
            liftFeedforwardVoltage = liftFFController.calculate(liftPidController.getSetpoint().velocity);

            armFeedbackVoltage = armPidController.calculate(getArmPosition());
            armFeedforwardVoltage = armFFController.calculate(getArmPosition(), armPidController.getSetpoint().velocity);

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

            setLiftVoltage(liftFeedbackVoltage+liftFeedforwardVoltage);
            setArmVoltage(armFeedbackVoltage+armFeedforwardVoltage);
            //setArmVoltage(12.0);
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
     */
    public boolean liftAtGoal() {
        return liftPidController.atGoal();
    }

    public boolean armAtGoal() {
        return armPidController.atGoal();
    }

    /**
     * Creates a command that sets target setpoints for the elevator & arm. 
     * Command is cancelled once the mechanism has reached that goal.
     */
    public Command moveToPositionCommand(Supplier<ElevatorPosition> liftPositionSupplier,
                                         Supplier<ArmPosition> armPositionSupplier) {
        ioInfo.liftDesiredPositionInMeters = liftPositionSupplier.get().value;
        ioInfo.armDesiredPositionInRadians = armPositionSupplier.get().value;

        // stop current motion and clear integral values
        // specify the new goal position to the PID controllers 
        liftPidController.reset(getLiftPosition());
        liftPidController.setGoal(liftPositionSupplier.get().value);

        armPidController.reset(getArmPosition());
        armPidController.setGoal(armPositionSupplier.get().value);

        // run the motors until target goal is reached
        return Commands.sequence(
                            moveToSetPointCommand()
                                .until( () -> /*liftAtGoal() && */ armAtGoal() || didTriggerLiftLimitSwitch() ) 
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
        ArmPosition armPositionTarget;

        switch (level) {
            // Note: need to safely pivot arm to feeder station 
            case INTAKE:
                liftLevelTarget = ElevatorPosition.INTAKE;
                armPositionTarget = ArmPosition.FEED;
                break;
            // Arm positioned @ 180 degrees
            case TOP:
                liftLevelTarget = ElevatorPosition.TOP;
                armPositionTarget = ArmPosition.FEED;
                break;
            // Safely pivot arm to required degrees
            case CORAL_L1:       // Hijacked to test horizontal arm
                liftLevelTarget = ElevatorPosition.CORAL_L1;
                armPositionTarget = ArmPosition.HORIZONTAL; 
                break;
            case CORAL_L2:       
                liftLevelTarget = ElevatorPosition.CORAL_L2;
                armPositionTarget = ArmPosition.DELIVER;
                break;
            // @ start of the robot, elevator at the bottom & arm stowed -- reset encoders! 
            default: 
                liftLevelTarget = ElevatorPosition.BOTTOM;
                armPositionTarget = ArmPosition.STOW;
                break;   
        }

        return Commands.runOnce( 
            () -> {
                moveToPositionCommand( () -> liftLevelTarget, () -> armPositionTarget);
            }); 
    }

    /**
     * Creates a command stops the motor and sets it to coast mode, to allow for
     * moving the mechanism manually.
     * IMPORTANT: once used, motors will be in coast until robot code restarted
     */
    public void configureToCoastMotors() {
        liftMotor.stopMotor();
        ElevatorSubsystemConfigs.liftConfig.idleMode(IdleMode.kCoast);
        liftMotor.configure(ElevatorSubsystemConfigs.liftConfig,
                            ResetMode.kNoResetSafeParameters, 
                            PersistMode.kNoPersistParameters);
    }

    /**
     * Note: need to test this behavior
     */
    public Command coastMotorsThenBreakCommand() {
        return runOnce(liftMotor::stopMotor)
                .andThen(() -> {
                    ElevatorSubsystemConfigs.liftConfig.idleMode(IdleMode.kCoast);
                    liftMotor.configure(ElevatorSubsystemConfigs.liftConfig, 
                                        ResetMode.kNoResetSafeParameters, 
                                        PersistMode.kNoPersistParameters);
                })
                .finallyDo((d) -> {
                    ElevatorSubsystemConfigs.liftConfig.idleMode(IdleMode.kBrake);
                    liftMotor.configure(ElevatorSubsystemConfigs.liftConfig, 
                                        ResetMode.kNoResetSafeParameters, 
                                        PersistMode.kNoPersistParameters);
                    liftPidController.reset(getLiftPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("elevator.coastMotorsThenBreakCommand");
    }
}