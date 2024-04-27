package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.SwerveConstants.ModuleConstants;

public class SwerveModule implements Sendable {

    private final CANSparkMax drivingSparkMax;
    private final CANSparkMax turningSparkMax;

    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder turningEncoder;

    private final SparkPIDController drivingPIDController;
    private final SparkPIDController turningPIDController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    /** If reading the CANcoder is unsuccessful, this is how many times it will retry */
    private final int maximumCANcoderReadRetries = 5;
    /** Wait time for status frames to show up. */
    public static double STATUS_TIMEOUT_SECONDS = 0.02;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        drivingSparkMax = new CANSparkMax(driveMotorId, CANSparkLowLevel.MotorType.kBrushless);
        turningSparkMax = new CANSparkMax(turningMotorId, CANSparkLowLevel.MotorType.kBrushless);

        drivingSparkMax.clearFaults();
        turningSparkMax.clearFaults();

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        drivingSparkMax.restoreFactoryDefaults();
        turningSparkMax.restoreFactoryDefaults();

        drivingSparkMax.setInverted(driveMotorReversed);
        turningSparkMax.setInverted(turningMotorReversed);

        drivingEncoder = drivingSparkMax.getEncoder();
        turningEncoder = turningSparkMax.getEncoder();
        drivingPIDController = drivingSparkMax.getPIDController();
        turningPIDController = turningSparkMax.getPIDController();
        drivingPIDController.setFeedbackDevice(drivingEncoder);
        turningPIDController.setFeedbackDevice(turningEncoder);

        drivingEncoder.setPositionConversionFactor(ModuleConstants.driveEncoderRotToMeter);
        drivingEncoder.setVelocityConversionFactor(ModuleConstants.driveEncoderRPMToMeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.turningEncoderRotToRad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.turningEncoderRPMToRadPerSec);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        // This is roughly the same as enabling PIDContinuous on a WPI PIDController
        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(0);
        turningPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        drivingPIDController.setP(ModuleConstants.drivingP);
        drivingPIDController.setI(ModuleConstants.drivingI);
        drivingPIDController.setD(ModuleConstants.drivingD);
        drivingPIDController.setFF(ModuleConstants.drivingFF);
        drivingPIDController.setOutputRange(ModuleConstants.drivingMinOutput,
                ModuleConstants.drivingMaxOutput);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        turningPIDController.setP(ModuleConstants.turningP);
        turningPIDController.setI(ModuleConstants.turningI);
        turningPIDController.setD(ModuleConstants.turningD);
        turningPIDController.setFF(ModuleConstants.turningFF);
        turningPIDController.setOutputRange(ModuleConstants.turningMinOutput,
                ModuleConstants.turningMaxOutput);

        drivingSparkMax.setIdleMode(ModuleConstants.drivingMotorIdleMode);
        turningSparkMax.setIdleMode(ModuleConstants.turningMotorIdleMode);
        drivingSparkMax.setSmartCurrentLimit(ModuleConstants.drivingMotorCurrentLimit);
        turningSparkMax.setSmartCurrentLimit(ModuleConstants.turningMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        drivingSparkMax.burnFlash();
        turningSparkMax.burnFlash();

        new Thread(() -> { // Don't block anything else while sleeping
            try {
                Thread.sleep(500); // Gives the absolute encoders half-a-second to get started
                                        // TODO: See if this actually solves the problem of wheels being randomly skewed after starting
                resetEncoders();
            } catch (Exception e) {
                System.err.println(
                        "Failed to calibrate " + turningMotorId + " turning motor from absolute encoder. " +
                                "Something went wrong while sleeping the thread: \n\t" + e);
            }
        }).start();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");

        builder.addDoubleProperty(
                "Raw Absolute Encoder Position",
                this::getRawAbsoluteEncoderRad,
                null);
        builder.addDoubleProperty(
                "Adjusted Absolute Encoder Position",
                () -> getAbsoluteEncoderRad(false),
                null);
        builder.addDoubleProperty("Drive Motor Position", this::getDrivePosition, null);
        builder.addDoubleProperty("Drive Motor Velocity", this::getDriveVelocity, null);
        builder.addDoubleProperty("Turn Motor Position", this::getTurningPosition, null);
        builder.addDoubleProperty("Turn Motor Velocity", this::getTurningVelocity, null);
    }

    public double getDrivePosition() {
        return drivingEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return drivingEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /** Gets the raw, unadjusted absolute encoder rotation from the CANcoder cache */
    public double getRawAbsoluteEncoderRad() {
        StatusSignal<Double> rotations = absoluteEncoder.getAbsolutePosition();

        return rotations.getValue() // The value returned by the sensor is in rotations
                * (2 * Math.PI); // And we will invert, depending on the reversed state
    }

    /** Gets the absolute encoder rotation from the CANcoder
     * @param shouldRetry If true, the function will try up to
     * {@link SwerveModule#maximumCANcoderReadRetries maximumCANcoderReadRetries} times
     * to get a successful read from the sensor. Otherwise, it will just try to read immediately from the cached value*/
    public double getAbsoluteEncoderRad(boolean shouldRetry) {
        StatusSignal<Double> rotations = absoluteEncoder.getAbsolutePosition();

        if (shouldRetry) {
            // Taken from democat's library.
            // Source:
            // https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
            for (int i = 0; i < maximumCANcoderReadRetries; i++) {
                if (rotations.getStatus() == StatusCode.OK) {
                    break;
                }
                rotations = rotations.waitForUpdate(STATUS_TIMEOUT_SECONDS);
            }
        }

        return rotations.getValue() // The value returned by the sensor is in rotations
                * (2 * Math.PI) // We want radians
                - absoluteEncoderOffsetRad
                * (absoluteEncoderReversed ? -1 : 1); // And we will invert, depending on the reversed state
    }

    /** Resets drive encoder to 0, and turning encoder to the reading of the absolute encoder */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad(true));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(),
                Rotation2d.fromRadians(getTurningPosition())
        );
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle); // Ensures the wheel never has to move more than 90°
        drivingPIDController.setReference(state.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
        turningPIDController.setReference(state.angle.getRadians(), CANSparkBase.ControlType.kPosition);


    }

    /** Stops motors, telling drive motor to go to a velocity of 0 and the turning motor to hold its current rotation */
    public void stop() {
        drivingPIDController.setReference(0, CANSparkBase.ControlType.kVelocity);
        turningPIDController.setReference(getState().angle.getRadians(), CANSparkBase.ControlType.kPosition);
    }
}

