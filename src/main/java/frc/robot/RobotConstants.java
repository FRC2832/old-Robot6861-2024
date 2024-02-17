package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class RobotConstants {

	public static final int DRIVER_CONTROLLER = 0;
	public static final int OPERATOR_CONTROLLER = 1;
	// Base Swerve Constants
	// Module Constants
	public static final double kDriveMotorGearRatio = 1 / 6.75;
	public static final double kTurningMotorGearRatio = 1 / (150/7);
	public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
	public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
	public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
	public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
	public static final double kPTurning = 0.5;
	public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
	public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0;
	// Front Left
	public static final int kFrontLeftDriveMotorPort = 11;
	public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 13;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.0;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
	public static final int kFrontLeftTurningMotorPort = 12;

    // Front Right
    public static final int kFrontRightDriveMotorPort = 21;
    public static final int kFrontRightTurningMotorPort = 22;
    public static final boolean kFrontRightDriveEncoderReversed = false;
	public static final boolean kFrontRightTurningEncoderReversed = true;
	public static final int kFrontRightDriveAbsoluteEncoderPort = 23;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.0;
	public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

    // Back Left
    public static final int kBackLeftDriveMotorPort = 31;
    public static final int kBackLeftTurningMotorPort = 32;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 33;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.0;
	public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

	// Back Right
    public static final int kBackRightTurningMotorPort = 42;
    public static final int kBackRightDriveMotorPort = 41;
	public static final int kBackRightDriveAbsoluteEncoderPort = 43;
	public static final boolean kBackRightTurningEncoderReversed = false;
	public static final boolean kBackRightDriveEncoderReversed = true;
	public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.0;
	public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

	public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.0;
	public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.0;
    public static final double kDeadband = 0.05;
	public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;

	// Swerve Kinematics
	public static final double kWheelBase = 25.0; // inches
	public static final double kTrackWidth = 25.0; // inches
	public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
	public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =  kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

	public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

	public static final int kDriverXAxis = 0;
	public static final int kDriverYAxis = 1;
	public static final int kDriverFieldOrientedButtonIdx = 1;
	public static final int kDriverRotAxis = 4;

	
	// 
    
	private RobotConstants() {
		// no-op. This is a utility class, not to be instantiated.

		//Base Swerve Constants
		// 
	}
}
