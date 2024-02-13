package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final DutyCycleEncoder absoluteEncoder;
    private final LinearFilter absoluteEncoderFilter = LinearFilter.singlePoleIIR(
            ArmConstants.absoluteEncoderTimeConstant, 0.02);
    private final PIDController pidController;
//    private final SparkPIDController leftController;
//    private final SparkPIDController rightController;
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
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        leftMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmConstants.maxSetpoint);
        leftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmConstants.minSetpoint);

        rightMotor = new CANSparkMax(ArmConstants.rightCanID, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setInverted(ArmConstants.rightInverted);
        rightMotor.setSmartCurrentLimit(ArmConstants.currentLimit);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rightMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmConstants.maxSetpoint);
        rightMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmConstants.minSetpoint);

        absoluteEncoder = new DutyCycleEncoder(ArmConstants.absoluteEncoderPort);

        // set up the motor encoder including conversion factors to convert to radians and radians per second for position and velocity
        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPositionConversionFactor(ArmConstants.positionFactor);
        leftEncoder.setVelocityConversionFactor(ArmConstants.velocityFactor);

        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPositionConversionFactor(ArmConstants.positionFactor);
        rightEncoder.setVelocityConversionFactor(ArmConstants.velocityFactor);

        resetEncoders();

//        leftController = leftMotor.getPIDController();
//        PIDGains.setSparkMaxGains(leftController, ArmConstants.armPositionGains);
//
//        rightController = rightMotor.getPIDController();
//        PIDGains.setSparkMaxGains(rightController, ArmConstants.armPositionGains);

        pidController = ArmConstants.armPositionGains.asPIDController();

        leftMotor.burnFlash();
        rightMotor.burnFlash();

        setpoint = getAbsoluteEncoderPosition(); // At initialization, it will hold its position

        timer = new Timer();
        timer.start();

        updateMotionProfile();

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
        builder.addDoubleProperty("Absolute Encoder Position", () -> absoluteEncoder.getAbsolutePosition()
                * ArmConstants.absoluteEncoderConversionFactor, null);
        builder.addDoubleProperty("Absolute Encoder Adjusted Position", this::getAbsoluteEncoderPosition, null);
        builder.addDoubleProperty("Applied Output Left", leftMotor::getAppliedOutput, null);
        builder.addDoubleProperty("Applied Output Right", rightMotor::getAppliedOutput, null);
        builder.addDoubleProperty("Elapsed Time", timer::get, null);
//        builder.addDoubleProperty("Target Position", () -> targetState.position, null);
//        builder.addDoubleProperty("Target Velocity", () -> targetState.velocity, null);
        builder.addDoubleProperty("Feedforward", () -> feedforward, null);
        builder.addDoubleProperty("Manual Value", () -> manualValue, null);
        builder.addDoubleProperty("Setpoint", () -> setpoint, (val) -> setpoint = val);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(getAbsoluteEncoderPosition());
        rightEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public double getAbsoluteEncoderPosition() {
        var rawValue = absoluteEncoderFilter.calculate(absoluteEncoder.getAbsolutePosition()) // We filter noise before multiplying
                * ArmConstants.absoluteEncoderConversionFactor // Because multiplying exacerbates the issue
                - ArmConstants.absoluteEncoderOffset;
        return (ArmConstants.absoluteEncoderInverted) ? 0 - rawValue : rawValue;
    }

    /**
     * Sets the target position and updates the motion profile if the target position changed.
     * @param _setpoint The new target position in radians.
     */
    public void setTargetPosition(double _setpoint) {
        if (_setpoint != setpoint) {
            setpoint = MathUtil.clamp(_setpoint, ArmConstants.minSetpoint, ArmConstants.maxSetpoint);
            updateMotionProfile();
        }
    }

    /**Update the motion profile variables based on the current setpoint and the pre-configured motion constraints.*/
    private void updateMotionProfile() {
        startState = new TrapezoidProfile.State(getAbsoluteEncoderPosition(),
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
        double elapsedTime = timer.get();
        if (profile.isFinished(elapsedTime)) {
            targetState = new TrapezoidProfile.State(setpoint, 0.0);
        } else {
            targetState = profile.calculate(elapsedTime, startState, endState);
        }

        feedforward =
                ArmConstants.armFeedforward.calculate(
                        getAbsoluteEncoderPosition(), targetState.velocity);

        double controllerOutput = pidController.calculate(getAbsoluteEncoderPosition(), targetState.position);
        double voltageOutput = controllerOutput + feedforward;

        leftMotor.set(voltageOutput);
        rightMotor.set(voltageOutput);

//        leftController.setReference(
//                targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
//        rightController.setReference(
//                targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
    }

    /**
     * Drives the arm using the provided power value (usually from a joystick).
     * This also adds in the feedforward value which can help counteract gravity.
     * @param _power The motor power to apply.
     */
    public void runManual(double _power) {
        // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
        // passively
        setpoint = getAbsoluteEncoderPosition();
        updateMotionProfile();
        // update the feedforward variable with the newly zero target velocity
        feedforward =
                ArmConstants.armFeedforward.calculate(
                        getAbsoluteEncoderPosition(), targetState.velocity);
        // set the power of the motor
        leftMotor.set(_power + (feedforward / 12.0));
        rightMotor.set(_power + (feedforward / 12.0));
        manualValue = _power; // this variable is only used for logging or debugging if needed
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
    }
}

