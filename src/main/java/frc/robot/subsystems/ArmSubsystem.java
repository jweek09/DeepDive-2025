package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final SparkPIDController leftController;
    private final SparkPIDController rightController;

    private final DigitalInput frontLimitSwitch;
    private final DigitalInput backLimitSwitch;

    private double setpoint;

    private TrapezoidProfile profile;
    private final Timer timer;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private TrapezoidProfile.State targetState;
    private double feedforward;
    private double manualValue;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this ArmSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ArmSubsystem INSTANCE = new ArmSubsystem();

    /**
     * Returns the Singleton instance of this ArmSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ArmSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this ArmSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ArmSubsystem() {

        leftMotor = new CANSparkMax(ArmConstants.leftCanID, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.setInverted(ArmConstants.leftInverted);
        leftMotor.setSmartCurrentLimit(ArmConstants.currentLimit);
        leftMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        leftMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmConstants.backLimit);
        leftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmConstants.frontLimit);

        rightMotor = new CANSparkMax(ArmConstants.rightCanID, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setInverted(ArmConstants.rightInverted);
        rightMotor.setSmartCurrentLimit(ArmConstants.currentLimit);
        rightMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rightMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmConstants.backLimit);
        rightMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmConstants.frontLimit);

        // set up the motor encoder including conversion factors to convert to radians and radians per second for position and velocity
        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPositionConversionFactor(ArmConstants.positionFactor);
        leftEncoder.setVelocityConversionFactor(ArmConstants.velocityFactor);

        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPositionConversionFactor(ArmConstants.positionFactor);
        rightEncoder.setVelocityConversionFactor(ArmConstants.velocityFactor);

        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);

        leftController = leftMotor.getPIDController();
        PIDGains.setSparkMaxGains(leftController, ArmConstants.armPositionGains);

        rightController = rightMotor.getPIDController();
        PIDGains.setSparkMaxGains(rightController, ArmConstants.armPositionGains);

        leftMotor.burnFlash();
        rightMotor.burnFlash();

        frontLimitSwitch = new DigitalInput(ArmConstants.frontLimitSwitchPort);
        backLimitSwitch = new DigitalInput(ArmConstants.backLimitSwitchPort);

        setpoint = 0.0; // At initialization, it will hold its position

        timer = new Timer();
        timer.start();

        updateMotionProfile();

        targetState = new TrapezoidProfile.State(setpoint, 0.0); // Avoids nulls

        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Final Setpoint",   () -> setpoint, null);
        builder.addDoubleProperty("Left Position", leftEncoder::getPosition, null);
        builder.addDoubleProperty("Right Position", rightEncoder::getPosition, null);
        builder.addDoubleProperty("Encoder Reported Position", this::getEncoderPosition, null);
        builder.addDoubleProperty("Applied Output Left", leftMotor::getAppliedOutput, null);
        builder.addDoubleProperty("Applied Output Right", rightMotor::getAppliedOutput, null);

        builder.addBooleanProperty("Front Limit Switch Triggered", frontLimitSwitch::get, null);
        builder.addBooleanProperty("Back Limit Switch Triggered", backLimitSwitch::get, null);

        builder.addBooleanProperty("Arm Hold Enabled",
                () -> leftMotor.getIdleMode() == IdleMode.kBrake && rightMotor.getIdleMode() == IdleMode.kBrake,
                (boolean setting) -> { if (setting) { enableArmHold(); } else { disableArmHold(); } });

        builder.addDoubleProperty("Elapsed Time", timer::get, null);
//        builder.addDoubleProperty("Target Position", () -> targetState.position, null);
//        builder.addDoubleProperty("Target Velocity", () -> targetState.velocity, null);
        builder.addDoubleProperty("Feedforward", () -> feedforward, null);
        builder.addDoubleProperty("Manual Value", () -> manualValue, null);
        builder.addDoubleProperty("Setpoint", () -> setpoint, (val) -> setpoint = val);
    }

    public void enableArmHold() {
        if (leftMotor.getIdleMode() != IdleMode.kBrake // These should never be set without burning to flash
                || rightMotor.getIdleMode() != IdleMode.kBrake) { // So we check to avoid burning the flash unnecessarily
            leftMotor.setIdleMode(IdleMode.kBrake);
            rightMotor.setIdleMode(IdleMode.kBrake);

            leftMotor.burnFlash();
            rightMotor.burnFlash();
        }

        setTargetPosition(getEncoderPosition());
    }

    public void disableArmHold() {
        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);
    }

    public void resetEncodersBasedOnLimitSwitches() {
        if (frontLimitSwitch.get()) {
            resetEncoders(ArmConstants.frontLimit);
            setTargetPosition(ArmConstants.frontLimit);
            leftMotor.set(0.0);
            System.err.println("Hit front limit, adjusting encoders to " + ArmConstants.frontLimit + " radians.");
        }
        if (backLimitSwitch.get()) {
            resetEncoders(ArmConstants.backLimit);
            setTargetPosition(ArmConstants.backLimit);
            leftMotor.set(0.0);
            System.err.println("Hit back limit, adjusting encoders to " + ArmConstants.backLimit + " radians.");
        }
    }

    public void resetEncoders(double _rotation) {
        leftEncoder.setPosition(_rotation);
        rightEncoder.setPosition(_rotation);
    }

    public double getEncoderPosition() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }

    /**
     * Sets the target position and updates the motion profile if the target position changed.
     * @param _setpoint The new target position in radians.
     */
    public void setTargetPosition(double _setpoint) {
        if (_setpoint != setpoint) {
            setpoint = MathUtil.clamp(_setpoint, ArmConstants.frontLimit, ArmConstants.backLimit);
            updateMotionProfile();
        }
    }

    /**Update the motion profile variables based on the current setpoint and the pre-configured motion constraints.*/
    private void updateMotionProfile() {
        startState = new TrapezoidProfile.State(getEncoderPosition(),
                (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2); // Average velocity of both
        endState = new TrapezoidProfile.State(setpoint, 0.0);
        profile = new TrapezoidProfile(ArmConstants.armMotionConstraint);
        timer.reset();
    }

    /**
     * Drives the arm to a position using a trapezoidal motion profile.
     * This function is usually wrapped in a {@code RunCommand} which runs it repeatedly while the command is active.
     * <p>
     * This function updates the motor position control loop using a setpoint from the trapezoidal motion profile.
     * The target position is the last set position with {@code setTargetPosition}.
     */
    public void runAutomatic() {
        var prevTargetRotation = targetState.position;
        double elapsedTime = timer.get();
        if (profile.isFinished(elapsedTime)) {
            targetState = new TrapezoidProfile.State(setpoint, 0.0);
        } else {
            targetState = profile.calculate(elapsedTime, startState, endState);
        }

        feedforward =
                ArmConstants.armFeedforward.calculate(
                        getEncoderPosition(), targetState.velocity);
        if (!(frontLimitSwitch.get() && targetState.position < prevTargetRotation) // If either limit switch is hit
                && !(backLimitSwitch.get() && targetState.position > prevTargetRotation) // but we aren't trying to
        ) {                                                                             // move toward it
            leftController.setReference(
                    targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
            rightController.setReference(
                    targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
        } else {
            resetEncodersBasedOnLimitSwitches();
        }
    }

    /**
     * Drives the arm using the provided power value (usually from a joystick).
     * This also adds in the feedforward value which can help counteract gravity.
     * @param _power The motor power to apply.
     */
    public void runManual(double _power) {
        // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
        // passively
        setTargetPosition(getEncoderPosition());
        // update the feedforward variable with the newly zero target velocity
        feedforward =
                ArmConstants.armFeedforward.calculate(
                        getEncoderPosition(), targetState.velocity);
        if (!(frontLimitSwitch.get() && _power < 0)
                && !(backLimitSwitch.get() && _power > 0)) {
            // set the power of the motor
            leftMotor.set(_power + (feedforward / 12.0));
            rightMotor.set(_power + (feedforward / 12.0));
            manualValue = _power; // this variable is only used for logging or debugging if needed
        } else {
            resetEncodersBasedOnLimitSwitches();
        }
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
    }

    public Command GoToAngleCommand(double angle) {
        return Commands.runOnce(() -> setTargetPosition(angle), this).andThen( // Will set the target position
                Commands.waitUntil(() -> profile.isFinished(timer.get()))); // and then wait for the arm to get there
    }
}

