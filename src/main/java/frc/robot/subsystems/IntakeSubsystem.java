package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.PIDGains;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController controller;
    private final DigitalInput breakbeam = new DigitalInput(IntakeConstants.breakbeamPort);

    private boolean positionMode;
    private double targetPosition;
    private double power;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this IntakeSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static IntakeSubsystem INSTANCE = new IntakeSubsystem();

    /**
     * Returns the Singleton instance of this IntakeSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code IntakeSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static IntakeSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this IntakeSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private IntakeSubsystem() {

        motor = new CANSparkMax(IntakeConstants.canID, CANSparkLowLevel.MotorType.kBrushless);
        motor.clearFaults();
        motor.setInverted(IntakeConstants.motorInverted);
        motor.setSmartCurrentLimit(IntakeConstants.currentLimit);
        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        encoder.setPositionConversionFactor(IntakeConstants.motorPositionConversionFactorRotationToMeters);

        controller = motor.getPIDController();
        PIDGains.setSparkMaxGains(controller, IntakeConstants.positionGains);

        motor.burnFlash();

        positionMode = false;
        targetPosition = encoder.getPosition();
        power = 0.0;

        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Breakbeam Triggered", this::getBreakbeamBroken, null);
        builder.addBooleanProperty("Position Mode", () -> positionMode, null);
        builder.addDoubleProperty("Encoder Position", () -> encoder.getPosition(), null);
        builder.addDoubleProperty("Target Position", () -> targetPosition,
                (target) -> { positionMode = true; targetPosition = target; });
        builder.addDoubleProperty("Power", () -> power, this::setPower);
    }

    /**
     * Gets the reading from the breakbeam, inverting it based on {@link IntakeConstants#breakbeamTrueByDefault}
     * @return True if breakbeam is broken, false if not
     */
    public boolean getBreakbeamBroken() {
        return IntakeConstants.breakbeamTrueByDefault != breakbeam.get();
    }

    /**
     * Gets a trigger based on the reading of the breakbeam, using the {@link IntakeSubsystem#getBreakbeamBroken()} function
     * @return A trigger which will be true when the breakbeam is broken, false when unbroken
     */
    public Trigger getBreakbeamBrokenTrigger() {
        return new Trigger(this::getBreakbeamBroken);
    }

    /**
     * Set the power to spin the motor at.
     * This only applies outside of position mode.
     * @param _power The power to apply to the motor (from -1.0 to 1.0).
     */
    public void setPower(double _power) {
        positionMode = false;
        targetPosition = encoder.getPosition();
        power = MathUtil.clamp(_power, -1 , 1);
    }

    /**
     * Constructs a command that drives the rollers a specific distance (number of rotations)
     * from the current position and then ends the command.
     * @param distance The distance
     * @return The retract command
     */
    public Command retract(double distance) {
        Command newCommand =
                new Command() {
                    @Override
                    public void initialize() {
                        positionMode = true;
                        targetPosition = encoder.getPosition() - distance;
                    }

                    @Override
                    public boolean isFinished() {
                        return isNearTarget();
                    }
                };

        newCommand.addRequirements(this);

        return newCommand;
    }

    public Command RunUntilPickupCommand(double power) {
        return Commands.runOnce(() -> setPower(power), // Runs at the given power
                        this) // Requiring this subsystem
                .andThen(Commands.waitUntil(getBreakbeamBrokenTrigger())) // Until the beam is broken
                .finallyDo(() -> setPower(0.0)); // At which point it stops the motors
    }

    public Command IntakeNoteCommand(double power) {
        return RunUntilPickupCommand(power).andThen(retract(0.05));
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
        // if we've reached the position target, drop out of position mode
        if (positionMode && isNearTarget()) {
            positionMode = false;
            power = 0.0;
        }

        // update the motor power based on mode and setpoint
        if (positionMode) {
            controller.setReference(targetPosition, ControlType.kPosition);
        } else {
            motor.set(power);
        }
    }

    /**
     * Check if the encoder is within the position tolerance.
     * @return Whether the position is within the tolerance.
     */
    public boolean isNearTarget() {
        return Math.abs(encoder.getPosition() - targetPosition)
                < IntakeConstants.positionTolerance;
    }
}

