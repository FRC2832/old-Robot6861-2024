// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Neo 550 with SparkMax.  MUST have very low current limit to start!!!  5A
             // NEO 550's burn up in 7 sec!  It smells awful.
    }
}
