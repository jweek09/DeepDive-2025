// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;


public final class Autos {
    /** Example static factory for an autonomous command. */
    public static Command exampleAuto(ExampleSubsystem subsystem) {
        return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    }

    public static Command pickupCommand(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) {
        return Commands.run(() -> intakeSubsystem.setPower(Constants.IntakeConstants.intakePower), intakeSubsystem).withTimeout(1.5)
                .andThen(Commands.run(() -> intakeSubsystem.setPower(-Constants.IntakeConstants.intakePower))
                        .alongWith(Commands.run(() -> launcherSubsystem.runLauncher(-0.1))).withTimeout(0.15))
                .finallyDo(() -> {
                    intakeSubsystem.setPower(0);
                    launcherSubsystem.stopLauncher();
                });
    }
    
    private Autos()
    {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
