package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;


public class FeedLauncherCommand extends Command {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final LauncherSubsystem launcherSubsystem = LauncherSubsystem.getInstance();

    private final double launcherSpeed;

    public FeedLauncherCommand(double launcherSpeed) {
        this.launcherSpeed = launcherSpeed;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intakeSubsystem, this.launcherSubsystem);
    }

    private Timer timer;

    @Override
    public void initialize() {
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        intakeSubsystem.setPower(IntakeConstants.intakePower);
        launcherSubsystem.runLauncher(launcherSpeed);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > IntakeConstants.shotFeedTime;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setPower(0.0);
    }
}
