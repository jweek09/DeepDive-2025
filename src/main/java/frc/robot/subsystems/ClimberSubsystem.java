package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final VictorSPX leftMotor;
    private final VictorSPX rightMotor;

    private boolean extended = false;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this ClimberSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ClimberSubsystem INSTANCE = new ClimberSubsystem();

    /**
     * Returns the Singleton instance of this ClimberSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ClimberSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ClimberSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this ClimberSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ClimberSubsystem() {
        leftMotor = new VictorSPX(ClimberConstants.leftCanId);
        leftMotor.clearStickyFaults();
        leftMotor.setInverted(ClimberConstants.leftInverted);
        leftMotor.setNeutralMode(NeutralMode.Brake);

        rightMotor = new VictorSPX(ClimberConstants.rightCanId);
        rightMotor.clearStickyFaults();
        rightMotor.setInverted(ClimberConstants.rightInverted);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    public boolean isExtended() {
        return extended;
    }

    /**
     * Runs the motors on the climber
     * @param speed The percentage to set the motors to. Should be between -1 and 1, where 0 is stopped
     */
    public void runMotors(double speed) {
        leftMotor.set(VictorSPXControlMode.PercentOutput, speed);
        rightMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    /**
     * Stops the motors on the climber by running the motors at a 0 percent output
     */
    public void stopMotors() {
        runMotors(0.0);
    }

    /**
     * Constructs a command to run the motors at the given speed for the given time, stopping the motors when it finishes
     * @param speed The percentage to set the motors to. Should be between -1 and 1
     * @param timeSeconds The time to run for
     * @return The command to run with the given values
     */
    public Command runForTime(double speed, double timeSeconds) {
        return Commands.run(() -> runMotors(speed))
                .withTimeout(timeSeconds)
                .finallyDo(this::stopMotors);
    }

    /**
     * Extends the climber, using the constants for {@link ClimberConstants#releaseSpeed releaseSpeed} and
     * {@link ClimberConstants#releaseToTopTimeSeconds releaseTime}.
     * This will not run already extended. Use {@link ClimberSubsystem#retractClimber()} if you need to retract it first.
     * @return A command to extend the climber
     */
    public Command extendClimber() {
        return (Commands.runOnce(() -> extended = true)
                .andThen(runForTime(ClimberConstants.releaseSpeed, ClimberConstants.releaseToTopTimeSeconds)))
                .unless(this::isExtended); // Only runs if the climber isn't already extended
    }

    /**
     * Retracts the climber, using the constants for {@link ClimberConstants#climbSpeed climbSpeed} and
     * {@link ClimberConstants#retractFullyDownTimeSeconds retractionTime}.
     * This will not run unless the climber is extended using {@link ClimberSubsystem#extendClimber()}.
     * @return A command to retract the climber (climb)
     */
    public Command retractClimber() {
        return (Commands.runOnce(() -> extended = false)
                .andThen(runForTime(ClimberConstants.climbSpeed, ClimberConstants.retractFullyDownTimeSeconds))
                        .onlyIf(this::isExtended)); // Only runs if the climber is already extended
    }
}

