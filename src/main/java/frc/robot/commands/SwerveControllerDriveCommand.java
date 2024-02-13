package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;


public class SwerveControllerDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private final Supplier<Double> xSpeedSupplier, ySpeedSupplier, rotationSpeedSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private final ProfiledPIDController thetaController;

   /**
    * Creates a new command to drive the robot using the controller
    * @param xSpeedSupplier A supplier for the x direction of the robot. Should have a deadband applied
    * @param ySpeedSupplier A supplier for the y direction of the robot. Should have a deadband applied
    * @param rotationSpeedSupplier A supplier for the rotation of the robot, between -1 and 1
    * @param fieldOrientedSupplier A supplier for whether to drive in the field-oriented or robot-oriented space
    */
    public SwerveControllerDriveCommand(
            Supplier<Double> xSpeedSupplier,
            Supplier<Double> ySpeedSupplier,
            Supplier<Double> rotationSpeedSupplier,
            Supplier<Boolean> fieldOrientedSupplier) {
       this.xSpeedSupplier = xSpeedSupplier;
       this.ySpeedSupplier = ySpeedSupplier;
       this.rotationSpeedSupplier = rotationSpeedSupplier;
       this.fieldOrientedSupplier = fieldOrientedSupplier;

        this.xLimiter = new SlewRateLimiter(SwerveConstants.TeleopConstants.teleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(SwerveConstants.TeleopConstants.teleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(SwerveConstants.TeleopConstants.teleDriveMaxAngularAccelerationUnitsPerSecond);

        thetaController = new ProfiledPIDController(
                SwerveConstants.TeleopConstants.teleopDrivingPTheta,
                SwerveConstants.TeleopConstants.teleopDrivingITheta,
                SwerveConstants.TeleopConstants.teleopDrivingDTheta,
                SwerveConstants.TeleopConstants.teleopDrivingThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double xSpeed = xSpeedSupplier.get();
        double ySpeed = ySpeedSupplier.get();
        double turningSpeed = rotationSpeedSupplier.get();

        xSpeed = xLimiter.calculate(xSpeed) * SwerveConstants.TeleopConstants.teleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * SwerveConstants.TeleopConstants.teleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
            * SwerveConstants.TeleopConstants.teleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds = (fieldOrientedSupplier.get()) ?
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, driveSubsystem.getOdometryHeading()) :
                ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, turningSpeed, driveSubsystem.getOdometryHeading());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        driveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
