package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ParallelCommands extends ParallelCommandGroup {
    public ParallelCommands(DriveTrainSubsystem driveTrainSubsystem)
    {
        AutoDriveCommand autoDriveCommand = new AutoDriveCommand(driveTrainSubsystem);
        addCommands(
        // Drive forward the specified distance
        autoDriveCommand,
        new BullCommand(driveTrainSubsystem, autoDriveCommand));
    }
}
