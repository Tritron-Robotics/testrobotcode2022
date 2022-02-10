package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class Kinematics
    {
        public static final double tankDriveVolts = 4.0;
        public static final double arcadeDriveVolts = 0.5;

        public static final double arcadeDriveSpeedModifierVolts = 0.7;
        public static final double tankDriveSpeedModifierVolts = 0.9;


        // PLACEHOLDERS: Use FRC Characterization Tool for values.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
        // Example value only - as above, this must be tuned for your drive.
        public static final double kPDriveVel = 8.5;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds.
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Track Robot Velocity, pass in horizontal distance between wheels.
        public static final double kTrackwidthMeters = Units.inchesToMeters(28);
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    }

    public static final class MotorConstants
    {
        public static final int rearLeftPort = 1;
        public static final int frontLeftPort = 3;
        public static final int rearRightPort = 2;
        public static final int frontRightPort = 4;
    }

    public static final class Controller
    {
        // port numbers for controller buttons
        public static final int rightTrigger = 8;
        public static final int leftTrigger = 7;
        
        public static final int y = 1;
        public static final int rightBumper = 6;
        public static final int leftBumper = 5;

        
        // to get left stick y-axis: -controller.getY()
        // to get right stick y-axis:  -controller.getRawAxis(3)
        // to get any button that corresponds to a port number(like the right trigger): controller.getRawButton(portnumber)
    }
}
