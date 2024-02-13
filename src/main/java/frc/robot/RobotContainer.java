// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FeedLauncherCommand;
import frc.robot.commands.SwerveControllerDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final LauncherSubsystem launcherSubsystem = LauncherSubsystem.getInstance();
    private double defaultLauncherSpeed = 0.5;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        SmartDashboard.putData(driveSubsystem);
        SmartDashboard.putData(intakeSubsystem);
        SmartDashboard.putData(launcherSubsystem);

        SmartDashboard.putData("Default Shooter Speed", builder -> builder.addDoubleProperty(
                "Speed (0-1)", () -> defaultLauncherSpeed,
                (double speed) -> defaultLauncherSpeed = MathUtil.clamp(speed, 0, 1))
        );
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(exampleSubsystem));
        
        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        // driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());

        driverController.back().onTrue(Commands.runOnce(() -> driveSubsystem.resetWheelEncoders()));

        var alliance = DriverStation.getAlliance();
        double invertFieldOrientedControls =
                (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) // Invert field-oriented controls if on red alliance
                        ? -1 : 1; // Multiply by -1 if true (flip controls)

        driveSubsystem.setDefaultCommand(new SwerveControllerDriveCommand(
                () -> invertFieldOrientedControls * -MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.leftYDeadband),
                () -> invertFieldOrientedControls * -MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.leftXDeadband),
                () -> -driverController.getRightX(),
                () -> !driverController.getHID().getYButton() // Switch to robot oriented when Y is held
        ));

        driverController.x().whileTrue(driveSubsystem.getPathPlannerFollowCommand("Super Simple", true));

        // set the intake to stop (0 power) when no other command is running
        intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.setPower(0.0), intakeSubsystem));

        driverController.b()
                .whileTrue(new RunCommand(() -> {
                    intakeSubsystem.setPower(-Constants.IntakeConstants.intakePower);
                    launcherSubsystem.runLauncher(-0.1);
                }, intakeSubsystem, launcherSubsystem));

        driverController.leftBumper()
                    .whileTrue(Commands.run(() -> intakeSubsystem.setPower(Constants.IntakeConstants.intakePower), intakeSubsystem));

        // configure the launcher to stop when no other command is running
        launcherSubsystem.setDefaultCommand(new RunCommand(launcherSubsystem::stopLauncher, launcherSubsystem));

        // launcher controls (button to pre-spin the launcher and button to launch)
        driverController.rightBumper()
                .whileTrue(new RunCommand(() -> launcherSubsystem.runLauncher(defaultLauncherSpeed), launcherSubsystem));

        driverController.a()
                .onTrue(new FeedLauncherCommand(defaultLauncherSpeed));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(exampleSubsystem);
    }
}
