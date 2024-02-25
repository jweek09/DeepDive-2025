package frc.robot.commands;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ShootCommand extends SequentialCommandGroup {
    /**
     * Constructs a new command to shoot a piece, moving the arm to the correct position
     * @param armAngle The angle, in radians, of the arm
     * @param launcherSpeed The speed to run the launcher at. Should be between 0 and 1
     */
    public ShootCommand(double armAngle, double launcherSpeed) {
        super(ArmSubsystem.getInstance().GoToAngleCommand(armAngle),
                LauncherSubsystem.getInstance().shootWithSmartFeed(launcherSpeed));
        addRequirements(ArmSubsystem.getInstance(), LauncherSubsystem.getInstance());
    }

    /**
     * Constructs a new command to shoot a piece, moving the arm to the correct position
     * and pathfinding to the correct position on the field
     * @param position The position and rotation to move to on the field
     * @param armAngle The arm angle, in radians, to move to
     * @param launcherSpeed The speed to run the launcher at. Should be between 0 and 1
     */
    public ShootCommand(Pose2d position, double armAngle, double launcherSpeed) {
        super(DriveSubsystem.getInstance().pathfindToPosition(position),
                new ShootCommand(armAngle, launcherSpeed));
        addRequirements(DriveSubsystem.getInstance());
    }

    /**
     * Constructs a new command to shoot a piece, moving the arm to the correct position
     * and optionally pathfinding to the correct position on the field, mirrored if on red alliance
     * @param shootingPosition The {@link frc.robot.Constants.PositionConstants.ShootingPositions.ShootingPosition}
     *                         configuration for the position
     * @param moveToPosition A {@link BooleanSupplier} for whether to move the robot before shooting.
     *                       If this returns false, the robot will not move
     * @param mirrorToRedAlliance A {@link BooleanSupplier} for whether to mirror this point to the red alliance side.
     *                            This uses PathPlanner's built in {@link GeometryUtil#flipFieldPose(Pose2d)} to flip,
     *                            which mirrors translations over the center of the field and flips rotations 180°.
     */
    public ShootCommand(
            Constants.PositionConstants.ShootingPositions.ShootingPosition shootingPosition,
            BooleanSupplier moveToPosition,
            BooleanSupplier mirrorToRedAlliance) {
            super((moveToPosition.getAsBoolean()) ? new ShootCommand((mirrorToRedAlliance.getAsBoolean()) ? // If mirror
                    GeometryUtil.flipFieldPose(shootingPosition.getPose()) : shootingPosition.getPose(), // flip pose
                    shootingPosition.getArmAngle(), shootingPosition.getLauncherSpeed()) :
                    new ShootCommand(shootingPosition.getArmAngle(), shootingPosition.getLauncherSpeed()));
    }

    /**
     * Constructs a new command to shoot a piece, moving the arm, running the shooter, and optionally moving the robot
     * to the closest given point in {@link frc.robot.Constants.PositionConstants.ShootingPositions#validSpeakerShootingPositions}
     * using the {@link frc.robot.Constants.PositionConstants.ShootingPositions#getNearestPosition(Translation2d, boolean)}
     * function.
     *
     * @param currentRobotPosition A supplier for the robot's translation on the field
     * @param moveToPosition Whether to move the robot to the given position before moving the arm and shooting
     * @param isRedAlliance Whether to find positions on the red alliance side of the field
     */
    public ShootCommand(Supplier<Translation2d> currentRobotPosition, BooleanSupplier moveToPosition, BooleanSupplier isRedAlliance) {
        super(new ShootCommand(
                Constants.PositionConstants.ShootingPositions.getNearestPosition(
                        currentRobotPosition.get(), isRedAlliance.getAsBoolean()),
                moveToPosition, isRedAlliance));
    }
}