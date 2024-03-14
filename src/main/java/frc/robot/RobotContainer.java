// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
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

    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final LauncherSubsystem launcherSubsystem = LauncherSubsystem.getInstance();
    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    
    private DoubleSupplier defaultLauncherSpeed;
    private final BooleanSupplier isRedAlliance = () ->
            DriverStation.getAlliance().filter(value -> value == Alliance.Red).isPresent();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private boolean teleopDriveSpeedReduced = false;

    private final CommandXboxController operatorController =
            new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> autonomousCommand;
    private GenericEntry autonomousDelayTime;

    private double temporaryArmRotation = armSubsystem.getEncoderPosition();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        registerAutonomousCommands();

        autonomousCommand = AutoBuilder.buildAutoChooser("Do Nothing In Front Of Speaker");

        configureDashboard();

        // Configure the trigger bindings
        configureBindings();


        SmartDashboard.putData(driveSubsystem);
        SmartDashboard.putData(armSubsystem);
        SmartDashboard.putData(intakeSubsystem);
        SmartDashboard.putData(launcherSubsystem);
    }

    private void registerAutonomousCommands() {
        NamedCommands.registerCommand("ShootCommand", new ShootCommand(() -> driveSubsystem.getPose().getTranslation(), () -> false, isRedAlliance)
                .andThen(armSubsystem.GoToAngleCommand(Constants.ArmConstants.armWithinFramePosition)));
        NamedCommands.registerCommand("IntakeArmPositionCommand", armSubsystem.GoToIntakePositionCommand());
        NamedCommands.registerCommand("IntakeNoteCommand", Autos.pickupCommand(intakeSubsystem, launcherSubsystem));
        NamedCommands.registerCommand("ShootSubwooferNoSafetyMoveCommand",
                new ShootCommand(0.22, 0.5));
        NamedCommands.registerCommand("ShootSubwooferCommand",
                new ShootCommand(0.22, 0.5)
                        .andThen(armSubsystem.GoToAngleCommand(Constants.ArmConstants.armWithinFramePosition)));
        NamedCommands.registerCommand("ShootNearStageCommand",
                new ShootCommand(Constants.PositionConstants.ShootingPositions.stageShot)
                        .andThen(armSubsystem.GoToAngleCommand(Constants.ArmConstants.armWithinFramePosition)));
    }

    /**
     * Configures the Shuffleboard
     */
    private void configureDashboard() {
        /* 
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
 */
        var autonomousTab = Shuffleboard.getTab("Autonomous");

        autonomousTab.add("Autonomous Command", autonomousCommand)
                .withPosition(0,0)
                .withSize(3, 1);
        autonomousDelayTime = autonomousTab.add("Autonomous Delay Time", 0.0)
                .withPosition(0, 1)
                .withSize(3, 1)
                .getEntry(); // This creates an entry that we can read the NetworkTable for
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
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
        
               //  new Trigger(Constants)
               // .onTrue(new shootingPositions.stageShot(Constants));
        
        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        // driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());


        BooleanSupplier invertDriverControls = () -> DriverStation.getAlliance().filter(value -> value == Alliance.Red).isPresent();

        // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running
        armSubsystem.setDefaultCommand(new RunCommand(armSubsystem::runAutomatic, armSubsystem));

        // set the intake to stop (0 power) when no other command is running
        intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.setPower(0.0), intakeSubsystem));

        // configure the launcher to stop when no other command is running
        launcherSubsystem.setDefaultCommand(new RunCommand(launcherSubsystem::stopLauncher, launcherSubsystem));

        climberSubsystem.setDefaultCommand(Commands.run(climberSubsystem::stopMotors, climberSubsystem));

        driveSubsystem.setDefaultCommand(new SwerveControllerDriveCommand(
                () -> (invertDriverControls.getAsBoolean() ? -1 : 1)
                        * (teleopDriveSpeedReduced ? 0.33 : 1)
                        * -MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.leftYDeadband),
                () -> (invertDriverControls.getAsBoolean() ? -1 : 1)
                        * (teleopDriveSpeedReduced ? 0.33 : 1)
                        * -MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.leftXDeadband),
                () -> -driverController.getRightX(),
                () -> !driverController.getHID().getYButton() // Switch to robot oriented when Y is held
        ));

        driverController.x().onTrue(new InstantCommand(() -> teleopDriveSpeedReduced = true));
        driverController.a().onTrue(new InstantCommand(() -> teleopDriveSpeedReduced = false));

        driverController.leftBumper()
                .onTrue(Commands.run(() -> intakeSubsystem.setPower(Constants.IntakeConstants.intakePower), intakeSubsystem));

        driverController.b()
                .whileTrue(new RunCommand(() -> {
                    intakeSubsystem.setPower(-Constants.IntakeConstants.intakePower);
                    launcherSubsystem.runLauncher(-0.1);
                }, intakeSubsystem, launcherSubsystem));

        driverController.rightBumper()
                .onTrue(launcherSubsystem.shootWithDumbFeed(
                        Constants.PositionConstants.ShootingPositions.inFrontOfSpeaker.launcherSpeed, 1.5));

        driverController.leftTrigger()
                .whileTrue(Commands.run(() -> intakeSubsystem.setPower(driverController.getLeftTriggerAxis()), intakeSubsystem));

        driverController.rightTrigger()
                .whileTrue(Commands.run(() -> launcherSubsystem.runLauncher(driverController.getRightTriggerAxis() * 0.55), launcherSubsystem));

        driverController.pov(0)
                .whileTrue(Commands.run(() -> climberSubsystem.runMotors(Constants.ClimberConstants.releaseSpeed), climberSubsystem));

        driverController.pov(180)
                .whileTrue(Commands.run(() -> climberSubsystem.runMotors(Constants.ClimberConstants.climbSpeed), climberSubsystem));

                driverController.start().onTrue(Commands.runOnce(() -> {
                        driveSubsystem.resetOdometry(new Pose2d(
                                driveSubsystem.getPose().getTranslation(),
                                (isRedAlliance.getAsBoolean()) ? new Rotation2d(Math.PI) : new Rotation2d(0.0))
                        );
                }));

        driverController.back().onTrue(Commands.runOnce(() -> driveSubsystem.resetWheelEncoders()));

        new Trigger(() ->
                Math.abs(operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis()) > OperatorConstants.armManualDeadband
        ).whileTrue(new RunCommand(
                () ->
                        armSubsystem.runManual((operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis()) * OperatorConstants.armManualScale)
                , armSubsystem));

        operatorController.a().onTrue(new InstantCommand(() -> armSubsystem.setTargetPosition(Constants.PositionConstants.ShootingPositions.inFrontOfSpeaker.getArmAngle())));
        operatorController.x().onTrue(new InstantCommand(() -> armSubsystem.setTargetPosition(Constants.PositionConstants.ShootingPositions.stageShot.getArmAngle())));
        operatorController.y().onTrue(new InstantCommand(() -> armSubsystem.setTargetPosition(Constants.PositionConstants.ShootingPositions.farStageShot.getArmAngle())));
        operatorController.b().onTrue(new InstantCommand(() -> armSubsystem.setTargetPosition(Constants.PositionConstants.ShootingPositions.ampScore.getArmAngle())));
        operatorController.rightBumper().onTrue(new InstantCommand(() -> armSubsystem.setTargetPosition(Constants.PositionConstants.ShootingPositions.theSource.getArmAngle())));
        operatorController.leftBumper().onTrue(armSubsystem.GoToIntakePositionCommand());
        operatorController.povUp().onTrue(new InstantCommand(() -> armSubsystem.setTargetPosition(0.452)));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.waitSeconds(MathUtil.clamp(autonomousDelayTime.getDouble(0.0), 0, 15))
                .andThen(autonomousCommand.getSelected());
    }
}
