// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class BaseSwerveSubsystem extends SubsystemBase {
    /** Creates a new BaseSwerveSubsystem. */

    private final BaseSwerveModuleSubsystem frontLeft = new BaseSwerveModuleSubsystem(
            RobotConstants.kFrontLeftDriveMotorPort,
            RobotConstants.kFrontLeftTurningMotorPort,
            RobotConstants.kFrontLeftDriveEncoderReversed,
            RobotConstants.kFrontLeftTurningEncoderReversed,
            RobotConstants.kFrontLeftDriveAbsoluteEncoderPort,
            RobotConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            RobotConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final BaseSwerveModuleSubsystem frontRight = new BaseSwerveModuleSubsystem(
            RobotConstants.kFrontRightDriveMotorPort,
            RobotConstants.kFrontRightTurningMotorPort,
            RobotConstants.kFrontRightDriveEncoderReversed,
            RobotConstants.kFrontRightTurningEncoderReversed,
            RobotConstants.kFrontRightDriveAbsoluteEncoderPort,
            RobotConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            RobotConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final BaseSwerveModuleSubsystem backLeft = new BaseSwerveModuleSubsystem(
            RobotConstants.kBackLeftDriveMotorPort,
            RobotConstants.kBackLeftTurningMotorPort,
            RobotConstants.kBackLeftDriveEncoderReversed,
            RobotConstants.kBackLeftTurningEncoderReversed,
            RobotConstants.kBackLeftDriveAbsoluteEncoderPort,
            RobotConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            RobotConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final BaseSwerveModuleSubsystem backRight = new BaseSwerveModuleSubsystem(
            RobotConstants.kBackRightDriveMotorPort,
            RobotConstants.kBackRightTurningMotorPort,
            RobotConstants.kBackRightDriveEncoderReversed,
            RobotConstants.kBackRightTurningEncoderReversed,
            RobotConstants.kBackRightDriveAbsoluteEncoderPort,
            RobotConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            RobotConstants.kBackRightDriveAbsoluteEncoderReversed);

    // TODO: Confirm CANID on robot
    final PigeonIMU gyro = new PigeonIMU(50);
    // private final SwerveDriveOdometry odometer = new
    // SwerveDriveOdometry(RobotConstants.kDriveKinematics,
    // new Rotation2d(0));

    public BaseSwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        // TODO: Check if FusedHeading works or needs to be changed to Yaw or Compass
        gyro.setFusedHeading(0);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getFusedHeading(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // public Pose2d getPose() {
        // return odometer.getPoseMeters();
    // }

    // public void resetOdometry(Pose2d pose) {
       // odometer.resetPosition(pose, getRotation2d());
    // }

    @Override
    public void periodic() {
       // odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                // backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, RobotConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}