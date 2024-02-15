package org.livoniawarriors.swerve;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.DoubleSubscriber;
// import edu.wpi.first.wpilibj.XboxjoystickSubsystemObj;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.JoystickSubsystem;

/**
 * Drive the robot with xbox joystickSubsystemObj
 */
public class DriveXbox extends Command {
    private SwerveDriveTrain swerveDriveTrainObj;
    private JoystickSubsystem joystickSubsystemObj;
    private DoubleSubscriber deadband;

    /**
     * Inject the drivetrain and joystickSubsystemObj to use
     * 
     * @param swerveDriveTrainObj Drivetrain to command
     * @param cont  joystickSubsystemObj to read from
     */
    public DriveXbox(SwerveDriveTrain swerveDriveTrainObj, JoystickSubsystem joystickSubsystemObj) {
        this.swerveDriveTrainObj = swerveDriveTrainObj;
        this.joystickSubsystemObj = joystickSubsystemObj;
        deadband = UtilFunctions.getSettingSub("DriveXbox/Deadband", 0.13);
        addRequirements(swerveDriveTrainObj);
    }

    @Override
    public void initialize() {
        swerveDriveTrainObj.swerveDrive(0.0, 0.0, 0.0, false);
    }

    //TODO: change false to true for field oriented?

    @Override
    public void execute() {
        // driver clicked field reset stick
        if (joystickSubsystemObj.getDriverLeftStickButtonPressed()) {
            swerveDriveTrainObj.resetFieldOriented();
        }

        double dead = deadband.get();
        double xSpeed = UtilFunctions.deadband(-joystickSubsystemObj.getDriverLeftY(), dead);
        double ySpeed = UtilFunctions.deadband(-joystickSubsystemObj.getDriverLeftX(), dead);
        double turn = UtilFunctions.deadband(-joystickSubsystemObj.getDriverRightX(), dead);
                
        // Turtle mode (45%)
        if (joystickSubsystemObj.getDriverRightTrigger().getAsBoolean()) {
            swerveDriveTrainObj.swerveDrive(
                xSpeed * swerveDriveTrainObj.getMaxDriverSpeed() * 0.45,
                ySpeed * swerveDriveTrainObj.getMaxDriverSpeed() * 0.45,
                turn * swerveDriveTrainObj.getMaxDriverOmega() * 0.45);
        }
        // Regular drive
        else {
            swerveDriveTrainObj.swerveDrive(
                xSpeed * swerveDriveTrainObj.getMaxDriverSpeed(),
                ySpeed * swerveDriveTrainObj.getMaxDriverSpeed(),
                turn * swerveDriveTrainObj.getMaxDriverOmega());
        }
    }

    @Override
    public boolean isFinished() {
        // never end
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
