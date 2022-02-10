// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ParallelCommands;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.BullCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class RobotContainer {
    // Drivetrain subsystem.
    private DriveTrainSubsystem robotDrive; 
    private ShootingSubsystem shootingSubsystem;

    private final Joystick controller = new Joystick(0);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private AutoDriveCommand autoDrive; 
    private ParallelCommands parallelCommands;
    private BullCommand surpriseCommand;

    /**
     * Initializes all robot commands.
     */
    public RobotContainer() {

        InitializeSubsystems();
        AssignCommands();
    }

    private void InitializeSubsystems()
    {
        CANSparkMax rearLeft = new CANSparkMax(Constants.MotorConstants.rearLeftPort, MotorType.kBrushless); 
        CANSparkMax frontLeft = new CANSparkMax(Constants.MotorConstants.frontLeftPort, MotorType.kBrushless); 
        CANSparkMax rearRight = new CANSparkMax(Constants.MotorConstants.rearRightPort, MotorType.kBrushless); 
        CANSparkMax frontRight = new CANSparkMax(Constants.MotorConstants.frontRightPort, MotorType.kBrushless); 
        robotDrive = new DriveTrainSubsystem(rearLeft, frontLeft, rearRight, frontRight);

        CANSparkMax intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
        CANSparkMax topLeftShootMotor = new CANSparkMax(8, MotorType.kBrushed);
        CANSparkMax topRightShootMotor = new CANSparkMax(7, MotorType.kBrushed);
        shootingSubsystem = new ShootingSubsystem(intakeMotor, topLeftShootMotor, topRightShootMotor);       
    }

    private void AssignCommands()
    {
        Command driveCommand = new DriveCommand(
            robotDrive, 
            () -> -controller.getY(), 
            () -> -controller.getRawAxis(3), 
            () -> controller.getRawButton(Constants.Controller.leftBumper));

        Command arcadeDriveCommand = new ArcadeDriveCommand(
            robotDrive, 
            () -> -controller.getY(), 
            () -> controller.getX(), 
            () -> controller.getRawButton(Constants.Controller.leftBumper));

        Command shootCommand = new ShootCommand(
            shootingSubsystem,
            () -> controller.getRawButton(Constants.Controller.leftTrigger),
            () -> controller.getRawButton(Constants.Controller.rightTrigger),          
            () -> controller.getRawAxis(3));       

        
        shootingSubsystem.setDefaultCommand(shootCommand);

        autoDrive = new AutoDriveCommand(robotDrive);
        parallelCommands = new ParallelCommands(robotDrive);
        //surpriseCommand = new BullCommand(robotDrive);
        
        robotDrive.setDefaultCommand(arcadeDriveCommand);

        autoChooser.setDefaultOption("Default Auto", autoDrive);
        autoChooser.addOption("Surprise Auto", parallelCommands);
        SmartDashboard.putData(autoChooser);
    }

    public Command getAutonomousCommand() {
        
        return autoChooser.getSelected();
    }

    // Trajectory Generation
    public Command getAutoTrajectoryFollowCommand() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.Kinematics.ksVolts,
                                    Constants.Kinematics.kvVoltSecondsPerMeter,
                                    Constants.Kinematics.kaVoltSecondsSquaredPerMeter),
            Constants.Kinematics.kDriveKinematics,
            10);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(Constants.Kinematics.kMaxSpeedMetersPerSecond,
                                Constants.Kinematics.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.Kinematics.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            robotDrive::getPose,
            new RamseteController(Constants.Kinematics.kRamseteB, Constants.Kinematics.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.Kinematics.ksVolts,
                                       Constants.Kinematics.kvVoltSecondsPerMeter,
                                       Constants.Kinematics.kaVoltSecondsSquaredPerMeter),
            Constants.Kinematics.kDriveKinematics,
            robotDrive::getWheelSpeeds,
            new PIDController(Constants.Kinematics.kPDriveVel, 0, 0),
            new PIDController(Constants.Kinematics.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            robotDrive::tankDriveVolts,
            robotDrive
        );       

        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
    }
}
