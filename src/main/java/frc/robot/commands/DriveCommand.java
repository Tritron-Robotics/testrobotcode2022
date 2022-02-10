// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveCommand extends CommandBase {

  public DriveTrainSubsystem driveTrain;

  public DoubleSupplier leftSpeed;
  public DoubleSupplier rightSpeed;
  private BooleanSupplier speedModifier;

  /**
   * Constructor for the DriveCommand class
   * @param subsystem Subsystem for drive train
   * @param leftInput Left motors input
   * @param rightInput Right motors input
   * @param speedModifierInput Speed mofifier input. If this boolean is true, the speed of the motors will change.
   */
  public DriveCommand(DriveTrainSubsystem subsystem, DoubleSupplier leftInput, DoubleSupplier rightInput, BooleanSupplier speedModifierInput) {
    driveTrain = subsystem;
    this.leftSpeed = leftInput;
    this.rightSpeed = rightInput;
    this.speedModifier = speedModifierInput;
    
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.tankDriveVolts(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tankDrive();
  }

  /**
   * Controls the motors in the tank drive style 
   */
  private void tankDrive(){
    double volts = Constants.Kinematics.tankDriveVolts;
    if (speedModifier.getAsBoolean())
    {
      volts = Constants.Kinematics.tankDriveSpeedModifierVolts;
    }
    driveTrain.tankDriveVolts(leftSpeed.getAsDouble() * volts, rightSpeed.getAsDouble() * volts);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * A very, very important method.
   * @param randomBool This boolean is never used in the method. It is just a parameter.
   * @return Always returns true.
   */
  public boolean checkBooleans(boolean randomBool)
  {
    return true;
  }
}
