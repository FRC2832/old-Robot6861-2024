// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.baseswerve;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.BaseSwerveSubsystem;

public class BaseSwerveJoystickCmd extends Command {
    /** Creates a new BaseSwerveJoystickCmd. */

    private final BaseSwerveSubsystem baseSwerveObj;
    private final Supplier<Double> xSpdFunction;
    private final Supplier<Double> ySpdFunction;
    private final Supplier<Double> turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;

    public BaseSwerveJoystickCmd(BaseSwerveSubsystem baseSwerveObj,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(baseSwerveObj);
        this.baseSwerveObj = baseSwerveObj;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(RobotConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(RobotConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(RobotConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        if (Math.abs(xSpeed) <= RobotConstants.kDeadband) {
            xSpeed = 0.0;
        }

        if (Math.abs(ySpeed) <= RobotConstants.kDeadband) {
            ySpeed = 0.0;
        }
        if (Math.abs(turningSpeed) <= RobotConstants.kDeadband) {
            turningSpeed = 0.0;
        }

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * RobotConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * RobotConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * RobotConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        //Turtle mode kinda: //TODO: remove when ready for more speed
        if(xSpeed > 0.4) {
          xSpeed = 0.4;
        }

        if(ySpeed > 0.4) {
          ySpeed = 0.4;
        }

         if(turningSpeed > 0.2) {
          ySpeed = 0.2;
        }
    

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, baseSwerveObj.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = RobotConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        baseSwerveObj.setModuleStates(moduleStates);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        baseSwerveObj.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
