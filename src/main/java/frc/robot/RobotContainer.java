// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

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
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    
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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        registerAutonomousCommands();

        autonomousCommand = AutoBuilder.buildAutoChooser("Do Nothing In Front Of Speaker");

        configureDashboard();

        // Configure the trigger bindings
        configureBindings();


//        SmartDashboard.putData(driveSubsystem);
//        SmartDashboard.putData(armSubsystem);
//        SmartDashboard.putData(intakeSubsystem);
//        SmartDashboard.putData(launcherSubsystem);
    }

    private void registerAutonomousCommands() {
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
    };

    
    
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
