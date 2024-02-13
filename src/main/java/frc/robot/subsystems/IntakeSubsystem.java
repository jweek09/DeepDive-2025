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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController controller;

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
        motor.setInverted(IntakeConstants.motorInverted);
        motor.setSmartCurrentLimit(IntakeConstants.currentLimit);
        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

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
        builder.addBooleanProperty("Position Mode", () -> positionMode, null);
        builder.addDoubleProperty("Encoder Position", () -> encoder.getPosition(), null);
        builder.addDoubleProperty("Target Position", () -> targetPosition,
                (target) -> { positionMode = true; targetPosition = target; });
        builder.addDoubleProperty("Power", () -> power, this::setPower);
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
     * @return The retract command
     */
    public Command retract() {
        Command newCommand =
                new Command() {
                    @Override
                    public void initialize() {
                        positionMode = true;
                        targetPosition = encoder.getPosition() - IntakeConstants.retractDistance;
                    }

                    @Override
                    public boolean isFinished() {
                        return isNearTarget();
                    }
                };

        newCommand.addRequirements(this);

        return newCommand;
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

