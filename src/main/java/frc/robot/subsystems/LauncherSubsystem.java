package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {

    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;

    private double topPower = 0.0;
    private double bottomPower = 0.0;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this LauncherSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static LauncherSubsystem INSTANCE = new LauncherSubsystem();

    /**
     * Returns the Singleton instance of this LauncherSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code LauncherSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static LauncherSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this LauncherSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private LauncherSubsystem() {

        // create two new SPARK MAXs and configure them
        topMotor =
                new CANSparkMax(LauncherConstants.topCanId, CANSparkLowLevel.MotorType.kBrushless);
        topMotor.setInverted(LauncherConstants.topMotorInverted);
        topMotor.setSmartCurrentLimit(LauncherConstants.currentLimit);
        topMotor.setIdleMode(IdleMode.kBrake);

        topMotor.burnFlash();

        bottomMotor =
                new CANSparkMax(LauncherConstants.bottomCanId, CANSparkLowLevel.MotorType.kBrushless);
        bottomMotor.setInverted(LauncherConstants.bottomMotorInverted);
        bottomMotor.setSmartCurrentLimit(LauncherConstants.currentLimit);
        bottomMotor.setIdleMode(IdleMode.kBrake);

        bottomMotor.burnFlash();

        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Top Motor Power",
                () -> topPower, (power) -> runLauncher(power, bottomPower));
        builder.addDoubleProperty("Bottom Motor Power",
                () -> bottomPower, (power) -> runLauncher(topPower, power));
    }

    /**
     * Runs the launcher, with both wheels at the given target power.
     * Can be run once and the launcher will stay running or run continuously in a {@code RunCommand}.
     * @param targetPower The target power to set both the top and bottom motors to
     */
    public void runLauncher(double targetPower) {
        this.topPower = targetPower;
        this.bottomPower = targetPower;
    }

    /**
     * Runs the launcher, with each wheel set to the given target power.
     * Can be run once and the launcher will stay running or run continuously in a {@code RunCommand}.
     * @param topPower The target power to set the top motor to
     * @param bottomPower The target power to set the bottom motor to
     */
    public void runLauncher(double topPower, double bottomPower) {
        this.topPower = MathUtil.clamp(topPower, 0, 1);
        this.bottomPower = MathUtil.clamp(bottomPower, 0, 1);
    }

    /**
     * Turns the launcher off.  Can be run once and the launcher will stay running or run continuously in a {@code RunCommand}.
     */
    public void stopLauncher() {
        this.topPower = 0;
        this.bottomPower = 0;
    }

    @Override
    public void periodic() {  // this method will be called once per scheduler run
        // set the launcher motor powers based on whether the launcher is on or not
        topMotor.set(topPower);
        bottomMotor.set(bottomPower);
    }

    /**
     * A command that shoots the launcher, running until the breakbeam in the {@link IntakeSubsystem} is cleared
     * with a small delay to let it clear defined by {@link Constants.IntakeConstants#clearLauncherTime}
     * @param launcherSpeed The speed at which to run the shooter
     * @return A command to run the intake and launcher until the note clears the breakbeam
     */
    public Command shootWithSmartFeed(double launcherSpeed) {
        var intakeSubsystem = IntakeSubsystem.getInstance();

        return Commands.run(() -> {
                    intakeSubsystem.setPower(Constants.IntakeConstants.intakePower); // Runs the intake
                    runLauncher(launcherSpeed); // and launcher,
                }, intakeSubsystem, this)
                .raceWith( // stopping it when the breakbeam is no longer broken
                        Commands.waitUntil(intakeSubsystem.getBreakbeamBrokenTrigger().negate())
                                .withTimeout(3) // or it times out,
                                .andThen(Commands.waitSeconds(Constants.IntakeConstants.clearLauncherTime)) // then waiting 0.2 seconds to let it clear the launcher,
                ).finallyDo(() -> { // finally stopping the intake and shooter when everything is done
                    intakeSubsystem.setPower(0.0);
                    stopLauncher();
                });
    }

    /**
     * Runs the intake and launcher to feed a shot, finishing after the given runTime.
     * {@link #shootWithSmartFeed(double launcherSpeed)} should be used instead unless something is wrong with the breakbeam sensor
     * @param launcherSpeed The speed at which to run the launcher
     * @param runTime The time to run for
     * @return A command to run the intake and launcher for the given amount of time
     */
    public Command shootWithDumbFeed(double launcherSpeed, double runTime) {
        var intakeSubsystem = IntakeSubsystem.getInstance();

        return Commands.run(() -> {
                    intakeSubsystem.setPower(Constants.IntakeConstants.intakePower); // Runs the intake
                    runLauncher(launcherSpeed); // and launcher,
                }, intakeSubsystem, this)
                .withTimeout(runTime) // until the given timeout,
                .finallyDo(() -> { // finally stopping the intake and shooter when everything is done
                    intakeSubsystem.setPower(0.0);
                    stopLauncher();
                });
    }
}

