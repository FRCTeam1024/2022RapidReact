// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double PI = 3.14159;
    public static final boolean CompBot = Robot.isCompBot();

    public static final int PCMID = 3;  // CAN ID for PCM

    // IDs for physical input devices
    // Make sure order matches that of DriverStation
    public static final class Inputs {
        public static final int driverControllerID = 0; // ID for Xbox/Logitech controller
        public static final int operatorControllerID = 1;
    }

    // Drivetrain
    public static final class DriveConstants{
        // Drivetrain motor IDs
        public static final int driveLeftFollowerID = 11; 
        public static final int driveLeftLeaderID = 12;
        public static final int driverRightFollowerID = 13;
        public static final int driveRightLeaderID = 14;   

        //example values pulled from tutorial, need to characterize Falcon Drive to find appropriate values
        public static final double ksVolts = 0.59;
        public static final double kvVoltSecondsPerMeter = 1.71;
        public static final double kaVoltSecondsSquaredPerMeter = 0.137;
        public static final double kPDriveVel = 2.23;

        public static final double kTrackwidthMeters = 0.626; //need to measure trackwidth, or characterize drive
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 1; //cut these in half while testing
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        
        public static final double kGearing = 7.56;
        public static final double kMetersPerRotation = 0.48878;
        public static final double kSensorUnitsPerRotation = 2048*kGearing;
        
    }

    // Shooter related constants
    public static final class ShooterConstants {
        public static final int launchValveA = 1;
        public static final int launchValveB = 2;
        public static final int launchValveC = 3;
    }

    // Intake related constants
    public static final class IntakeConstants {
        public static final int intakeValve = 3;
        
        public static final double kCollectSpeed = 0.5;  //No idea if this is what we want
        public static final double kEjectSpeed = -0.2;  //Also no idea if this is what we want
    }

    // Limelight related constants
    public static final class LimelightConstants {
        public static final int driverPipe = 1;
        public static final int targetPipe = 0;
    }
}