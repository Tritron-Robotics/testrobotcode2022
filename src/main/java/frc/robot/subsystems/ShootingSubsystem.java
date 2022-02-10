// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem for the motors that shoot the ball */
public class ShootingSubsystem extends SubsystemBase {

    //Motor controllers
    private CANSparkMax intakeMotor;
    private CANSparkMax topLeftShootMotor;
    private CANSparkMax topRightShootMotor;

    // Initialize the motors
    public ShootingSubsystem(CANSparkMax intakeMotor, CANSparkMax topLeftShootMotor, CANSparkMax topRightShootMotor) {
        this.intakeMotor = intakeMotor;
        this.topLeftShootMotor = topLeftShootMotor;
        this.topRightShootMotor = topRightShootMotor;

        SmartDashboard.putNumber("Shooter speed", 8);
        SmartDashboard.putNumber("Intake speed", 7);
    }

    /**
     * Shoots the ball
     * @param input Controller input
     */
    public void shoot(double input)
    {
        topLeftShootMotor.setVoltage(input * SmartDashboard.getNumber("Shooter speed", 8));
        topRightShootMotor.setVoltage(-input * SmartDashboard.getNumber("Shooter speed", 8));
    }

    /**
     * Activates the intake system
     * @param input Controller input
     */
    public void spinIntake(double input)
    {
        intakeMotor.setVoltage(input * SmartDashboard.getNumber("Intake speed", 7)); 
    }

    public void stopMotors()
    {
        topLeftShootMotor.setVoltage(0);
        topRightShootMotor.setVoltage(0);
        intakeMotor.setVoltage(0);
    }
}
