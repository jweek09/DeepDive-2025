package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNoteCommand extends SequentialCommandGroup {
    /**
     * Moves the arm to the intake position, then runs the intake until it picks up a note
     */
    public IntakeNoteCommand() {
        super(ArmSubsystem.getInstance().GoToAngleCommand(Constants.ArmConstants.frontLimit),
                IntakeSubsystem.getInstance().runUntilPickup(Constants.IntakeConstants.intakePower));
    }
}