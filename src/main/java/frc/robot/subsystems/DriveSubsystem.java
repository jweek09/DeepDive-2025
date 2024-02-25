package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final Field2d field = new Field2d();

    private final SwerveModule frontLeft = new SwerveModule(
            SwerveConstants.PortConstants.frontLeftDriveMotorPort,
            SwerveConstants.PortConstants.frontLeftTurningMotorPort,
            SwerveConstants.PhysicalConstants.frontLeftDriveEncoderReversed,
            SwerveConstants.PhysicalConstants.frontLeftTurningEncoderReversed,
            SwerveConstants.PortConstants.frontLeftDriveAbsoluteEncoderPort,
            SwerveConstants.PhysicalConstants.frontLeftDriveAbsoluteEncoderOffsetRad,
            SwerveConstants.PhysicalConstants.frontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            SwerveConstants.PortConstants.frontRightDriveMotorPort,
            SwerveConstants.PortConstants.frontRightTurningMotorPort,
            SwerveConstants.PhysicalConstants.frontRightDriveEncoderReversed,
            SwerveConstants.PhysicalConstants.frontRightTurningEncoderReversed,
            SwerveConstants.PortConstants.frontRightDriveAbsoluteEncoderPort,
            SwerveConstants.PhysicalConstants.frontRightDriveAbsoluteEncoderOffsetRad,
            SwerveConstants.PhysicalConstants.frontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            SwerveConstants.PortConstants.backLeftDriveMotorPort,
            SwerveConstants.PortConstants.backLeftTurningMotorPort,
            SwerveConstants.PhysicalConstants.backLeftDriveEncoderReversed,
            SwerveConstants.PhysicalConstants.backLeftTurningEncoderReversed,
            SwerveConstants.PortConstants.backLeftDriveAbsoluteEncoderPort,
            SwerveConstants.PhysicalConstants.backLeftDriveAbsoluteEncoderOffsetRad,
            SwerveConstants.PhysicalConstants.backLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            SwerveConstants.PortConstants.backRightDriveMotorPort,
            SwerveConstants.PortConstants.backRightTurningMotorPort,
            SwerveConstants.PhysicalConstants.backRightDriveEncoderReversed,
            SwerveConstants.PhysicalConstants.backRightTurningEncoderReversed,
            SwerveConstants.PortConstants.backRightDriveAbsoluteEncoderPort,
            SwerveConstants.PhysicalConstants.backRightDriveAbsoluteEncoderOffsetRad,
            SwerveConstants.PhysicalConstants.backRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            SwerveConstants.swerveDriveKinematics,
            new Rotation2d(0),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this DriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DriveSubsystem INSTANCE = new DriveSubsystem();

    /**
     * Returns the Singleton instance of this DriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DriveSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this DriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DriveSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        SmartDashboard.putData("NavX Gyroscope", gyro);
        SmartDashboard.putData("Front Left Swerve Module", frontLeft);
        SmartDashboard.putData("Front Right Swerve Module", frontRight);
        SmartDashboard.putData("Back Left Swerve Module", backLeft);
        SmartDashboard.putData("Back Right Swerve Module", backRight);

        SmartDashboard.putData("Field", field);

        new Thread(() -> { // Don't block anything else while sleeping
            try {
                Thread.sleep(1000); // Waits for the gyro to boot up before resetting the heading
                zeroHeading();
            } catch (Exception e) {
                System.err.println(
                        "Failed to zero gyro heading. Something went wrong while sleeping the thread: \n\t" + e);
            }
        }).start();

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(
                                SwerveConstants.AutoConstants.translationP,
                                SwerveConstants.AutoConstants.translationI,
                                SwerveConstants.AutoConstants.translationD
                        ), // Translation PID constants
                        new PIDConstants(
                                SwerveConstants.AutoConstants.rotationP,
                                SwerveConstants.AutoConstants.rotationI,
                                SwerveConstants.AutoConstants.rotationD
                        ), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        SwerveConstants.PhysicalConstants.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Gets a PathPlanner path follower.
     * Events, if registered elsewhere using {@link com.pathplanner.lib.auto.NamedCommands}, will be run.
     *
     * @param pathName The PathPlanner path name, as configured in the configuration
     * @param setOdomToStart If true, will set the odometry to the start of the path when this command is initialized
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command
     */
    public Command getPathPlannerFollowCommand(String pathName, boolean setOdomToStart) {
        // Loads the path from the GUI name given
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if (setOdomToStart) {
            resetOdometry(new Pose2d(path.getPoint(0).position, getGyroRotation2d()));
        }

        // Creates a path following command using AutoBuilder. This will execute any named commands when running
        return AutoBuilder.followPath(path);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveConstants.swerveDriveKinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        );
    }

    public void zeroHeading() {
        gyro.reset();
    }

    /** Gets the rotation reported by the heading in degrees
     * @return The reported angle, in degrees */
    private double getHeading() {
        return -Math.IEEEremainder(gyro.getAngle(), 360);
    }

    private Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation2d getOdometryHeading() {
        return getPose().getRotation();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }, pose);
        field.setRobotPose(odometry.getPoseMeters());
    }

    public void resetWheelEncoders() {
        var pose = odometry.getPoseMeters(); // Save it so the pose isn't reset
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
        resetOdometry(pose);
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
        field.setRobotPose(odometry.getPoseMeters());
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        setModuleStates(SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /** Stops all modules, setting their velocity to 0 and instructing them to hold their current rotation */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.PhysicalConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}

