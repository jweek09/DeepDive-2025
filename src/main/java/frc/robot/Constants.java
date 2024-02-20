// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.PIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;

        public static final double leftXDeadband = 0.01;
        public static final double leftYDeadband = 0.01;
        public static final double rightXDeadband = 0.01;
        public static final double rightYDeadband = 0.01;
    }

    public static class SwerveConstants {
        public static class ModuleConstants {
            public static final double wheelDiameterMeters = Units.inchesToMeters(4);
            public static final double driveMotorGearRatio = 1 / 6.75; // 6.75:1
            public static final double turningMotorGearRatio = 1 / (150.0 / 7); // 150/7:1
            /** The conversion factor for drive encoders from rotations to meters */
            public static final double driveEncoderRotToMeter = driveMotorGearRatio * Math.PI * wheelDiameterMeters;
            /** The conversion factor for turning encoders from rotations to radians */
            public static final double turningEncoderRotToRad = turningMotorGearRatio * 2 * Math.PI;
            /** The conversion factor for drive encoders from rotations per minute to meters per second */
            public static final double driveEncoderRPMToMeterPerSec = driveEncoderRotToMeter / 60;
            /** The conversion factor for turning encoders from rotations per minute to radians per second */
            public static final double turningEncoderRPMToRadPerSec = turningEncoderRotToRad / 60;

            /** The free speed of a drive wheel, in meters per second?
             * Retrieved from <a href=https://www.swervedrivespecialties.com/products/mk4i-swerve-module>SDS Specifications</a> */
            // Note: didn't change name of variable to meters per second, because I'm not really sure what it's in
            // This is a carry-over from REVLib, where it was implemented based on a ton of gear ratio conversions
            // https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/169c7e33edac0a2fa3568d5442b6e73c79c33389/src/main/java/frc/robot/Constants.java#L78C5-L85C34
            // I did some rough math and figured the two numbers were arrived at the same way
            // ((motorFreeSpeed * wheelCircumference) / Gear reduction)
            public static final double driveWheelFreeSpeedRps = Units.feetToMeters(15.1);

            public static final double drivingP = 0.04;
            public static final double drivingI = 0;
            public static final double drivingD = 0;
            public static final double drivingFF = 1 / driveWheelFreeSpeedRps;
            public static final double drivingMinOutput = -1;
            public static final double drivingMaxOutput = 1;

            public static final double turningP = 1;
            public static final double turningI = 0;
            public static final double turningD = 0;
            public static final double turningFF = 0;
            public static final double turningMinOutput = -1;
            public static final double turningMaxOutput = 1;

            public static final IdleMode drivingMotorIdleMode = IdleMode.kBrake;
            public static final IdleMode turningMotorIdleMode = IdleMode.kBrake;

            public static final int drivingMotorCurrentLimit = 50; // amps
            public static final int turningMotorCurrentLimit = 20; // amps
        }
        public static class PortConstants {
            public static final int frontLeftDriveMotorPort = 6;
            public static final int backLeftDriveMotorPort = 3;
            public static final int frontRightDriveMotorPort = 9;
            public static final int backRightDriveMotorPort = 12;
            public static final int frontLeftTurningMotorPort = 4;
            public static final int backLeftTurningMotorPort = 1;
            public static final int frontRightTurningMotorPort = 7;
            public static final int backRightTurningMotorPort = 10;
            public static final int frontLeftDriveAbsoluteEncoderPort = 5;
            public static final int backLeftDriveAbsoluteEncoderPort = 2;
            public static final int frontRightDriveAbsoluteEncoderPort = 8;
            public static final int backRightDriveAbsoluteEncoderPort = 11;
        }
        public static class PhysicalConstants {
            // Chassis configuration
            // Distance between front and back wheels on robot
            public static final double trackWidth = Units.inchesToMeters(23.75);
            // Distance between centers of right and left wheels on robot
            public static final double wheelBase = Units.inchesToMeters(23.75);

            public static final double driveBaseRadius = Math.hypot(trackWidth / 2, wheelBase / 2);

            public static final boolean frontLeftTurningEncoderReversed = true;
            public static final boolean backLeftTurningEncoderReversed = true;
            public static final boolean frontRightTurningEncoderReversed = true;
            public static final boolean backRightTurningEncoderReversed = true;
            public static final boolean frontLeftDriveEncoderReversed = true;
            public static final boolean backLeftDriveEncoderReversed = true;
            public static final boolean frontRightDriveEncoderReversed = true;
            public static final boolean backRightDriveEncoderReversed = true;
            public static final boolean frontLeftDriveAbsoluteEncoderReversed = false;
            public static final boolean backLeftDriveAbsoluteEncoderReversed = false;
            public static final boolean frontRightDriveAbsoluteEncoderReversed = false;
            public static final boolean backRightDriveAbsoluteEncoderReversed = false;
            public static final double frontLeftDriveAbsoluteEncoderOffsetRad = 3.45145677;
            public static final double backLeftDriveAbsoluteEncoderOffsetRad = 2.61543724;
            public static final double frontRightDriveAbsoluteEncoderOffsetRad = 5.46250558;
            public static final double backRightDriveAbsoluteEncoderOffsetRad = 4.71699094;
            public static final double physicalMaxSpeedMetersPerSecond = 4.8;
            public static final double physicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        }

        public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(PhysicalConstants.wheelBase / 2, PhysicalConstants.trackWidth / 2),
                new Translation2d(PhysicalConstants.wheelBase / 2, -PhysicalConstants.trackWidth / 2),
                new Translation2d(-PhysicalConstants.wheelBase / 2, PhysicalConstants.trackWidth / 2),
                new Translation2d(-PhysicalConstants.wheelBase / 2, -PhysicalConstants.trackWidth / 2));

        public static class TeleopConstants {
            public static final double teleDriveMaxSpeedMetersPerSecond = PhysicalConstants.physicalMaxSpeedMetersPerSecond / 4;
            public static final double teleDriveMaxAngularSpeedRadiansPerSecond = //
                    PhysicalConstants.physicalMaxAngularSpeedRadiansPerSecond / 4;
            public static final double teleDriveMaxAccelerationUnitsPerSecond = 3;
            public static final double teleDriveMaxAngularAccelerationUnitsPerSecond = 3;
            public static final double teleopDrivingPTheta = 0.4;
            public static final double teleopDrivingITheta = 0;
            public static final double teleopDrivingDTheta = 0;
            public static final TrapezoidProfile.Constraints teleopDrivingThetaControllerConstraints =
                    new TrapezoidProfile.Constraints(
                            PhysicalConstants.physicalMaxAngularSpeedRadiansPerSecond / 5,
                            Math.PI);
        }

        public static class AutoConstants {
            public static final double maxSpeedMetersPerSecond = PhysicalConstants.physicalMaxSpeedMetersPerSecond / 4;
            public static final double maxAngularSpeedRadiansPerSecond =
                    PhysicalConstants.physicalMaxAngularSpeedRadiansPerSecond / 10;
            public static final double maxAccelerationMetersPerSecondSquared = 3;
            public static final double maxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
            public static final double translationP = 5.0;
            public static final double translationI = 0.0;
            public static final double translationD = 0.0;
            public static final double rotationP = 5.0;
            public static final double rotationI = 0.0;
            public static final double rotationD = 0.0;

            public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                    new TrapezoidProfile.Constraints(
                            maxAngularSpeedRadiansPerSecond,
                            maxAngularAccelerationRadiansPerSecondSquared);
        }
    }

    public static class IntakeConstants {
        public static final int canID = 13;
        public static final boolean motorInverted = false;
        public static final int currentLimit = 20;
        public static final double intakeWheelDiameter = Units.inchesToMeters(2.0);
        public static final double motorPositionConversionFactorRotationToMeters = intakeWheelDiameter * Math.PI;

        public static final int breakbeamPort = 3;
        /** If the breakbeam reads true by default (when not blocked) */
        public static final boolean breakbeamTrueByDefault = true;

        public static final PIDGains positionGains = new PIDGains(1.0, 0.0, 0.0);
        public static final double positionTolerance = 0.5;

        public static final double intakePower = 0.7;

        /** The time after which a note past the breakbeam is clear of the launcher */
        public static double clearLauncherTime = 0.2;
    }

    public static class LauncherConstants {
        public static final int topCanId = 14;
        public static final int bottomCanId = 15;
        public static final boolean topMotorInverted = false;
        public static final boolean bottomMotorInverted = false;

        public static final int currentLimit = 80;
    }
}
