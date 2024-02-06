package org.livoniawarriors.swerve;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.DoubleSubscriber;
// import edu.wpi.first.wpilibj.XboxdriverController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.JoystickSubsystem;

/**
 * Drive the robot with xbox driverController
 */
public class DriveXbox extends Command {
    private SwerveDriveTrain drive;
    private JoystickSubsystem driverController;
    private DoubleSubscriber deadband;

    /**
     * Inject the drivetrain and driverController to use
     * 
     * @param drive Drivetrain to command
     * @param cont  driverController to read from
     */
    public DriveXbox(SwerveDriveTrain drive, JoystickSubsystem driverController) {
        this.drive = drive;
        this.driverController = driverController;
        deadband = UtilFunctions.getSettingSub("DriveXbox/Deadband", 0.13);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.swerveDrive(0.0, 0.0, 0.0, false);
    }

    @Override
    public void execute() {
        // driver clicked field reset stick
        if (driverController.getDriverLeftStickButtonPressed()) {
            drive.resetFieldOriented();
        }

        var dead = deadband.get();
        double xSpeed = UtilFunctions.deadband(-driverController.getDriverLeftY(), dead);
        double ySpeed = UtilFunctions.deadband(-driverController.getDriverLeftX(), dead);
        double turn = UtilFunctions.deadband(-driverController.getDriverRightX(), dead);
        drive.swerveDrive(
                xSpeed * drive.getMaxDriverSpeed(),
                ySpeed * drive.getMaxDriverSpeed(),
                turn * drive.getMaxDriverOmega());
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
