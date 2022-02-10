package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSubsystem;

public class ShootCommand extends CommandBase {

    public ShootingSubsystem subsystem;
    public BooleanSupplier shootInput;
    public BooleanSupplier binaryIntakeInput;
    private DoubleSupplier intakeInput;

    /**
     * Constructor for the ShootCommand class
     * @param shootingSubsystem Subsystem for shooting motors
     * @param shootInput Boolean that determines whether we want to shoot or not
     * @param binaryIntakeInput Boolean that determines if the intake motors are on or not
     */
    public ShootCommand(ShootingSubsystem shootingSubsystem, BooleanSupplier shootInput, BooleanSupplier binaryIntakeInput, DoubleSupplier intakeInput) {
        this.shootInput = shootInput;
        this.binaryIntakeInput = binaryIntakeInput;
        this.intakeInput = intakeInput;
        this.subsystem = shootingSubsystem;
        addRequirements(shootingSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.stopMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if (shootInput.getAsBoolean()) {
            subsystem.shoot(-1);
        } else {
            subsystem.shoot(0);
        }

        // if there's no analog intakeInput
        if (intakeInput.getAsDouble() < 0.05 && intakeInput.getAsDouble() > -0.05)
        {
            if(binaryIntakeInput.getAsBoolean())
            {
                subsystem.spinIntake(1);
            }
            else
            {
                subsystem.spinIntake(0);
            }
        }
        else 
        {
            subsystem.spinIntake(-intakeInput.getAsDouble());
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
