package org.livoniawarriors.swerve;

import org.livoniawarriors.Logger;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;

@SuppressWarnings("removal")
public class SwerveHw24 implements ISwerveDriveIo {
    //measuring the robot, we got 11114 counts/90*, the theoretical amount is 10971.428/90* (150/7:1 gear ratio, 2048 counts/rev)
    private static final double COUNTS_PER_DEGREE = 121.9; //using theoretical amount

    //measuring the robot, we got 13899 counts/rev, theoretical is 13824 counts/rev (L2 gear set at 6.75:1 ratio)
    //needs to be scaled * 39.37 (in/m) / (4"*Pi wheel diameter) / 10 (units per 100ms)
    private static final double COUNTS_PER_METER = 4331.1 / 0.94362;     //velocity units
    private static final double DIST_PER_METER = COUNTS_PER_METER * 10;        //distance units

    //motors and sensors
    private CANSparkMax[] turnMotors;  
    private CANSparkMax[] driveMotors;
    private CANCoder[] absSensors;  //Yes, we are using CTRE!  Use Phoenix 5 for now.  
    private PIDController[] turnPids;
    private SparkPIDController[] drivePids;
    
    //sensor value buffers
    private double[] absSensorValues;
    private double[] driveWheelVelocities;
    private double[] driveWheelDistances;
    private double[] turnMotorAngles;
    private double[] correctedAngles;

    //Swerve corner locations for kinematics
    //trackwidth = 25" /2 = 12.5"
    //wheelbase = 25" /2 = 12.5"
    private Translation2d[] swervePositions = {
        new Translation2d(0.3175, 0.3175), // FL  meters
        new Translation2d(0.3175, -0.3175), // FR  meters
        new Translation2d(-0.3175, 0.3175), // RL   meters
        new Translation2d(-0.3175, -0.3175)   // RR  meters
    };

    private String[] moduleNames = {
        "FL",   //left side intake. ID 11,12,13
        "FR",   // ID 21,22,23
        "RL",   // ID 31, 32, 33
        "RR"    // ID 41, 42, 43
    };

    public SwerveHw24() {
        //initialize array sizes
        int numWheels = swervePositions.length;

        //initialize sensor buffers
        turnMotors = new CANSparkMax[numWheels];
        driveMotors = new CANSparkMax[numWheels];
        absSensors = new CANCoder[numWheels];
        absSensorValues = new double[numWheels];
        driveWheelVelocities = new double[numWheels];
        driveWheelDistances = new double[numWheels];
        turnMotorAngles = new double[numWheels];
        correctedAngles = new double[numWheels];

        //software turn PID setup

        // turnPids = new PIDController[numWheels];
        //for (int i = 0; i < turnPids.length; i++) {
            //turnPids[i] = new PIDController(0, 0, 0.0); //TODO: change to SparkMax PID controller?? 
                // TODO: original kp = 5, ki = 1.8, set kp, ki = 0 for tuning
       // }

        
        //initialize each motor/sensor
        driveMotors[0] = new CANSparkMax(11, MotorType.kBrushless);
        driveMotors[1] = new CANSparkMax(21, MotorType.kBrushless);
        driveMotors[2] = new CANSparkMax(31, MotorType.kBrushless);
        driveMotors[3] = new CANSparkMax(41, MotorType.kBrushless);

        turnMotors[0] = new CANSparkMax(12, MotorType.kBrushless);
        turnMotors[1] = new CANSparkMax(22, MotorType.kBrushless);
        turnMotors[2] = new CANSparkMax(32, MotorType.kBrushless);
        turnMotors[3] = new CANSparkMax(42, MotorType.kBrushless);

        absSensors[0] = new CANCoder(13);
        absSensors[1] = new CANCoder(23);
        absSensors[2] = new CANCoder(33);
        absSensors[3] = new CANCoder(43);

        // initialize the drive PID controllers
        // drivePids = new SparkPIDController[numWheels];
       // for (int i = 0; i < drivePids.length; i++) {
            //drivePids[i] = driveMotors[i].getPIDController(); 
           // drivePids[i].setP(0); // TODO: Original = 0.5. Tune these values. May need values specific per motor.
            //drivePids[i].setI(0); //TODO: original = 0.03.
            //drivePids[i].setD(0);
           //drivePids[i].setFF(1023 / (5 * COUNTS_PER_METER));
            //drivePids[i].setIZone(0);
        //}
        
        for (CANCoder sensor : absSensors) {
            sensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 18);
            sensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);
        }

        for (CANSparkMax motor : driveMotors) {
            //motors MUST be reset every power up!!!
            motor.restoreFactoryDefaults();
            //old software pid values are p 0.5, i 0.03, d 0
            //allConfigs.slot0.kP = 1024 / (0.5 * COUNTS_PER_METER);
            //old value of 0.03 means if we had 1m/s error for 1 second, add 0.03V to the PID (or 1.705 counts of 1023)
            //unknown why we need 2/3, but then the math works...
            //allConfigs.slot0.kI = 0.03 * Constants.CTRE_P_RES / COUNTS_PER_METER * (2./3);
            //allConfigs.slot0.kI = 0;
            //allConfigs.slot0.kD = 0;
            //this works out to 1023 / Max speed in counts
            //allConfigs.slot0.kF = 1023 / (5 * COUNTS_PER_METER);

            //allConfigs.slot0.integralZone = 0;
            //allConfigs.slot0.allowableClosedloopError = 0;

            //the maximum velocity we want the motor to go
            //allConfigs.motionCruiseVelocity = 5 * COUNTS_PER_METER;
            //the maximum acceleration we want the motor to go
            //allConfigs.motionAcceleration = 5 * COUNTS_PER_METER;

            motor.setSmartCurrentLimit(20); // TODO: May need to comment these out since the CTRE version sets cfg to disabled.     
            motor.setSecondaryCurrentLimit(30); // TODO: Increase at somepoint 

            //motor.configAllSettings(allConfigs);
            // motor.setSelectedSensorPosition(0); Replaced with setPosition(0) for CANCoders below
            motor.setControlFramePeriodMs(40);
            //motor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
        }

        for (CANSparkMax motor : turnMotors) {
            //motors MUST be reset every power up!!!
            //motor.configFactoryDefault();
            motor.restoreFactoryDefaults();
            //motor.getAllConfigs(allConfigs);
            //allConfigs.slot1.kP = 0.2;
            //allConfigs.slot1.kI = 0.0005;
            //allConfigs.slot1.kD = 40;
            //allConfigs.slot1.kF = 0;
            //allConfigs.slot1.integralZone = 0;
            //allConfigs.slot1.allowableClosedloopError = 300;
            //allConfigs.motionCruiseVelocity = 20960;
            //allConfigs.motionAcceleration = 40960;
            //motor.configAllSettings(allConfigs);
            // motor.selectProfileSlot(1, 0); TODO: May be vendor-specific. Need to check, though.

            //this stator current limit helps stop neutral brake faults
            //StatorCurrentLimitConfiguration cfg = new StatorCurrentLimitConfiguration();
            //cfg.enable = false;
            //cfg.currentLimit = 20;
            //cfg.triggerThresholdCurrent = 40;
            motor.setSmartCurrentLimit(20); // TODO: May need to comment these out since the CTRE version sets cfg to disabled.
            motor.setSecondaryCurrentLimit(40);
            
            //motor.configStatorCurrentLimit(cfg);
            motor.setControlFramePeriodMs(40);
            //motor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
        }

        for (CANCoder encoder : absSensors) {
            encoder.setPosition(0);
        }

        //register stuff for logging
        for (int wheel = 0; wheel < numWheels; wheel++) {
            final int wheelFinal = wheel;
            Logger.registerCanSparkMax(null, null); //TODO: Replace with our CANSparkMax objects.
            Logger.registerCanSparkMax(null, null);
            Logger.registerSensor(null, null); // TODO: Replace with our RelativeEncoders
            Logger.registerSensor(moduleNames[wheel] + " Speed", () -> getCornerSpeed(wheelFinal));
            Logger.registerSensor(moduleNames[wheel] + " Turn Pos", () -> getCornerAngle(wheelFinal));
            Logger.registerSensor(moduleNames[wheel] + " Drive Dist", () -> getCornerDistance(wheelFinal));
        }
    }

    @Override
    public void updateInputs() {
        for (int i = 0; i < swervePositions.length; i++) {
            absSensorValues[i] = absSensors[i].getPosition();
            driveWheelVelocities[i] = absSensors[i].getVelocity() / COUNTS_PER_METER;
            driveWheelDistances[i] = absSensors[i].getPosition() / DIST_PER_METER;
            turnMotorAngles[i] = -absSensors[i].getPosition() / COUNTS_PER_DEGREE;
        }
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        IdleMode mode;

        if (brakeOn) {
            mode = IdleMode.kBrake;
        } else {
            mode = IdleMode.kCoast;
        }

        for (CANSparkMax motor : turnMotors) {
            motor.setIdleMode(mode);
        }
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        IdleMode mode;

        if (brakeOn) {
            mode = IdleMode.kBrake;
        } else {
            mode = IdleMode.kCoast;
        }
        
        for (CANSparkMax motor : driveMotors) {
            motor.setIdleMode(mode);
        }
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        return absSensorValues[wheel];
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnMotorAngles[wheel];
    }

    @Override
    public double getCornerSpeed(int wheel) {
        return driveWheelVelocities[wheel];
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        //driveMotors[wheel].set(ControlMode.Velocity, swerveModuleState.speedMetersPerSecond * COUNTS_PER_METER);
        //driveMotors[wheel].set(swerveModuleState.speedMetersPerSecond * COUNTS_PER_METER, CANSparkBase.ControlType.kDutyCycle);
        // Does not work ^

        // drivePids[wheel].setReference(swerveModuleState.speedMetersPerSecond * COUNTS_PER_METER, ControlType.kVelocity); //TODO: was kVelocity, changed to open loop per 
        //REv instructions to go open loop for checking CANcoder direction. Plus, leave OPen loop for determing Feedforward and PID constants

        //we need the request to be within the boundaries, not wrap around the 180 point
        double turnRequest = MathUtil.inputModulus(swerveModuleState.angle.getDegrees(), correctedAngles[wheel] - 180, correctedAngles[wheel] + 180);
        if (Math.abs(correctedAngles[wheel] - turnRequest) < 1) {
            //reset the PID to remove all the I term error so we don't overshoot and rebound
            turnPids[wheel].reset();  //TODO: robot code won't enable because this is null. 
            //TODO: (continued from above) Need to figure out how to set turnPIDS to open loop - just set all PID gains to 0?
        }
        double turnVolts = -turnPids[wheel].calculate(Math.toRadians(correctedAngles[wheel]), Math.toRadians(turnRequest));
        //turnMotors[wheel].set(ControlMode.PercentOutput, turnVolts / RobotController.getBatteryVoltage());
        turnMotors[wheel].set(turnVolts / RobotController.getBatteryVoltage());  //TODO: looks like alreadyt open loop?  open loop per 
        //REv instructions to go open loop for checking CANcoder direction. Plus, leave OPen loop for determing Feedforward and PID constants
        // TODO: add position control for turn motors?
    }

    @Override
    public double getCornerDistance(int wheel) {
        return driveWheelDistances[wheel];
    }

    @Override
    public Translation2d[] getCornerLocations() {
        return swervePositions;
    }

    @Override
    public String[] getModuleNames() {
        return moduleNames;
    }

    @Override
    public void setCorrectedAngle(int wheel, double angle) {
        correctedAngles[wheel] = angle;
    }
} 
