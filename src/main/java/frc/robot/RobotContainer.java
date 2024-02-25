// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SwerveControllerDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


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
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final LauncherSubsystem launcherSubsystem = LauncherSubsystem.getInstance();
    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    private DoubleSupplier defaultLauncherSpeed;
    private final BooleanSupplier isRedAlliance = () ->
            DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final SendableChooser<Command> autonomousCommand;

    private double temporaryArmRotation = armSubsystem.getEncoderPosition();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        registerAutonomousCommands();

        autonomousCommand = AutoBuilder.buildAutoChooser("");

        configureDashboard();

        // Configure the trigger bindings
        configureBindings();

        var underStageArmConstraints = new ArrayList<Pair<Rotation2d, Rotation2d>>(1);
        underStageArmConstraints.add(
                Pair.of(Rotation2d.fromRadians(0), Rotation2d.fromRadians(Constants.ArmConstants.underStageLimit)));

        new Trigger(() -> driveSubsystem.getPose().getTranslation().getDistance(Constants.FieldConstants.blueStageCenter)
                    < Constants.FieldConstants.stageDangerRadius)
            .or(() -> driveSubsystem.getPose().getTranslation().getDistance(Constants.FieldConstants.redStageCenter)
                    < Constants.FieldConstants.stageDangerRadius)
            .onTrue(
                Commands.runOnce(() -> armSubsystem.setArmRotationConstraints(underStageArmConstraints,
                        new Trigger(() -> driveSubsystem.getPose().getTranslation().getDistance(Constants.FieldConstants.blueStageCenter)
                                > Constants.FieldConstants.stageDangerRadius)
                            .and(() -> driveSubsystem.getPose().getTranslation().getDistance(Constants.FieldConstants.redStageCenter)
                                > Constants.FieldConstants.stageDangerRadius)
            ), armSubsystem));

        SmartDashboard.putData(driveSubsystem);
        SmartDashboard.putData(armSubsystem);
        SmartDashboard.putData(intakeSubsystem);
        SmartDashboard.putData(launcherSubsystem);
    }

    private void registerAutonomousCommands() {
        NamedCommands.registerCommand("ShootCommand",
                new ShootCommand(() -> driveSubsystem.getPose().getTranslation(), () -> false, isRedAlliance));
        NamedCommands.registerCommand("IntakeNoteCommand", new IntakeNoteCommand());
    }

    /**
     * Configures the Shuffleboard
     */
    private void configureDashboard() {
        var shootTuningTab = Shuffleboard.getTab("Shooting Position Tuning");

        GenericEntry shootTuningIsRedAlliance = shootTuningTab.add("On Red Alliance Side",
                isRedAlliance)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withSize(3, 1)
                .withPosition(0,0)
                .getEntry();

        GenericEntry shooterSpeed = shootTuningTab.add("Launcher Speed", 0.5)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(3, 1)
                .withPosition(0,1)
                .getEntry();

        defaultLauncherSpeed = () -> shooterSpeed.get().getDouble();

        shootTuningTab.add("Intake Note", Commands.runOnce(() -> temporaryArmRotation = armSubsystem.getEncoderPosition())
                .andThen(new IntakeNoteCommand())
                .andThen(armSubsystem.GoToAngleCommand(temporaryArmRotation)))
                .withWidget(BuiltInWidgets.kCommand)
                .withSize(3, 1)
                .withPosition(0,2);

        shootTuningTab.addString("Shooting Position Config", () ->
                "Pose: " + ((shootTuningIsRedAlliance.getBoolean(false)) ?
                        GeometryUtil.flipFieldPose(driveSubsystem.getPose()).toString() :
                        driveSubsystem.getPose().toString()) + "\n---\n" +
                String.format("Arm Angle: %.4f", armSubsystem.getEncoderPosition()) + "\n---\n" +
                String.format("Shooter Speed: %.2f", defaultLauncherSpeed.getAsDouble())
        ).withPosition(3, 0).withSize(3, 3);

        var autonomousTab = Shuffleboard.getTab("Autonomous");

        autonomousTab.add("Autonomous Command", autonomousCommand);
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

        // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running
        armSubsystem.setDefaultCommand(new RunCommand(armSubsystem::runAutomatic, armSubsystem));

        new Trigger(() ->
                Math.abs(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()) > OperatorConstants.armManualDeadband
        ).whileTrue(new RunCommand(
                () ->
                        armSubsystem.runManual((driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()) * OperatorConstants.armManualScale)
                , armSubsystem));

        // set the intake to stop (0 power) when no other command is running
        intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.setPower(0.0), intakeSubsystem));

        driverController.b()
                .whileTrue(new RunCommand(() -> {
                    intakeSubsystem.setPower(-Constants.IntakeConstants.intakePower);
                    launcherSubsystem.runLauncher(-0.1);
                }, intakeSubsystem, launcherSubsystem));

        driverController.leftBumper()
                .onTrue(intakeSubsystem.runUntilPickup(Constants.IntakeConstants.intakePower));
//                .whileTrue(Commands.run(() -> intakeSubsystem.setPower(Constants.IntakeConstants.intakePower), intakeSubsystem));

        // configure the launcher to stop when no other command is running
        launcherSubsystem.setDefaultCommand(new RunCommand(launcherSubsystem::stopLauncher, launcherSubsystem));

        // launcher controls (button to pre-spin the launcher and button to launch)
        driverController.rightBumper()
                .whileTrue(new RunCommand(() -> launcherSubsystem.runLauncher(defaultLauncherSpeed.getAsDouble()), launcherSubsystem));

        driverController.a()
                .onTrue(launcherSubsystem.shootWithSmartFeed(defaultLauncherSpeed.getAsDouble()));

        climberSubsystem.setDefaultCommand(Commands.run(climberSubsystem::stopMotors, climberSubsystem));

        driverController.pov(0)
                .whileTrue(Commands.run(() -> climberSubsystem.runMotors(Constants.ClimberConstants.releaseSpeed)));

        driverController.pov(180)
                .whileTrue(Commands.run(() -> climberSubsystem.runMotors(Constants.ClimberConstants.climbSpeed)));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autonomousCommand.getSelected();
    }
}
