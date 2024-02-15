// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.livoniawarriors.odometry.Odometry;
import org.livoniawarriors.swerve.DriveXbox;
import org.livoniawarriors.swerve.MoveWheels;
import org.livoniawarriors.swerve.SwerveDriveSim;
import org.livoniawarriors.swerve.SwerveDriveTrain;
import org.livoniawarriors.swerve.SwerveHw24;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.baseswerve.BaseSwerveJoystickCmd;
import frc.robot.subsystems.BaseSwerveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    
    // Characterization Code 
    private final BaseSwerveSubsystem baseSwerveObj = new BaseSwerveSubsystem();
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

    
    
    
   // TODO: uncomment these once characterization is done 
    //private SwerveDriveTrain swerveDriveObj;
    //private Odometry odometryObj;
    //private JoystickSubsystem joystickSubsystemObj;
    // private LedSubsystem leds;

    // private XboxController driverController;

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // Characteriztation Code
         baseSwerveObj.setDefaultCommand(new BaseSwerveJoystickCmd(
                baseSwerveObj,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();

    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    }


        
        // driverController = new XboxController(0);
        String serNum = RobotController.getSerialNumber();
        SmartDashboard.putString("Serial Number", serNum);
        //known Rio serial numbers:
        //031b525b = buzz
        //03064db7 = big buzz

        //subsystems used in all robots
        //joystickSubsystemObj = new JoystickSubsystem(); //TODO: Uncomment all Odometry once swerve is working
        // odometryObj = new Odometry(); //TODO: Uncomment all Odometry once swerve is working
        // leds = new LedSubsystem(0, 10);
        // new VisionSystem(odometryObj); //not making variable as we won't change this subsystem 
            // TODO: Uncomment all vision once swerve is working 
        

        //TODO: uncomment lines 76-84 when characterization is done
        //build the robot based on the Rio ID of the robot
        //if (RobotBase.isSimulation() || (serNum.equals("031b525b")) || (serNum.equals("03064db7"))) {
            //either buzz or simulation
            //swerveDriveObj = new SwerveDriveTrain(new SwerveDriveSim(), odometryObj);
            //odometryObj.setGyroHardware(new SimSwerveGyro(swerveDrive));
        //} else {
            //competition robot
            //swerveDriveObj = new SwerveDriveTrain(new SwerveHw24(), odometryObj);
            //odometryObj.setGyroHardware(new Pigeon2Gyro(0));
        //}
        
        //odometryObj.setSwerveDrive(swerveDrive);
        //odometryObj.setStartingPose(new Pose2d(1.92, 2.79, new Rotation2d(0)));

        //add some buttons to press for development
        SmartDashboard.putData("Wheels Straight", new MoveWheels(swerveDriveObj, MoveWheels.wheelsStraight()));
        SmartDashboard.putData("Wheels Crossed", new MoveWheels(swerveDriveObj, MoveWheels.wheelsCrossed()));
        SmartDashboard.putData("Wheels Diamond", new MoveWheels(swerveDriveObj, MoveWheels.wheelsDiamond()));
        SmartDashboard.putData("Drive Wheels Straight", new MoveWheels(swerveDriveObj, MoveWheels.driveWheelsStraight()));
        SmartDashboard.putData("Drive Wheels Diamond", new MoveWheels(swerveDriveObj, MoveWheels.driveWheelsDiamond()));
        // SmartDashboard.putData("Test Leds", new TestLeds(leds));

        // Register Named Commands for PathPlanner
        // NamedCommands.registerCommand("flashRed", new LightningFlash(leds, Color.kFirstRed));
        // NamedCommands.registerCommand("flashBlue", new LightningFlash(leds, Color.kFirstBlue));

        // Configure the AutoBuilder
        //AutoBuilder.configureHolonomic( //TODO: uncomment this to use Pathplanner!!!!
            //odometryObj::getPose, // Robot pose supplier
            //odometryObj::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            //swerveDriveObj::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
           // swerveDriveObj::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
           // new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                //new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                //new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
               // swerveDriveObj.getMaxSpeed(), // Max module speed, in m/s
               // swerveDriveObj.getDriveBaseRadius(), // Drive base radius in meters. Distance from robot center to furthest module.
               // new ReplanningConfig() // Default path replanning config. See the API for the options here
          //  ),
           // odometryObj::shouldFlipAlliance, //shouldFlipPath Supplier that determines if paths should be flipped to the other side of the field. This will maintain a global blue alliance origin.
            //swerveDriveObj // Reference to this subsystem to set requirements
       // );

        // Build an auto chooser. This will use Commands.none() as the default option.
        //autoChooser = AutoBuilder.buildAutoChooser();
        //SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    public void configureBindings() {
        //setup default commands that are used for driving
        //swerveDriveObj.setDefaultCommand(new DriveXbox(swerveDriveObj, joystickSubsystemObj));
        // leds.setDefaultCommand(new RainbowLeds(leds));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
