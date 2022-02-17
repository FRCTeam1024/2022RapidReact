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

        //need to characterize to find appropriate values
        public static final double ksVolts = 0.64048;
        public static final double kvVoltSecondsPerMeter = 2.3491;
        public static final double kaVoltSecondsSquaredPerMeter = 0.25828;
        public static final double kPDriveVel = 8;

        public static final double kTrackwidthMeters = 0.626; 
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 1; 
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        
        public static final double kGearing = 10.86;
        public static final double kMetersPerRotation = 0.48878;
        public static final double kSensorUnitsPerRotation = 2048*kGearing;
        
        // minimum and maximum drivetrain speed in meters per second
        public static final double minSpeed = -1;
        public static final double maxSpeed = 1;
    }

    // Shooter related constants
    public static final class ShooterConstants {
        public static final int launchValveA = 7;
        public static final int launchValveB = 3;
        public static final int aimValveUp = 5;
        public static final int aimValveDown = 6;
        public static final int loadMotorID = 17;

        public static final int kPressureAnalogID = 0;

        public static final double kInputVoltage = 5.0;

        public static final double kLoadSpeed = 0.5;  //No idea if this is what we want
    }

    // Intake related constants
    public static final class IntakeConstants {
        public static final int intakeValve = 4;
        public static final int intakeMotorID = 15;
        public static final int shifterMotorID = 16;
        
        public static final double kCollectSpeed = 0.5;  //No idea if this is what we want
        public static final double kEjectSpeed = -0.2;  //Also no idea if this is what we want
        public static final double kShifterSpeed = 0.5;  //No idea if this is what we want
        
    }

    // Hanger related constants
    public static final class HangerConstants {
        public static final int hookLiftLeaderID = 21;
        public static final int hookLiftFollowerID = 22;

        public static final int monkeyArmValveOut = 1;
        public static final int monkeyArmValveIn = 2;

        public static final int topLimitDigID = 0;
        public static final int bottomLimitDigID = 1;

        public static final double minTravelMeters = -0.05;
        public static final double maxTravelMeters = 0.8; 

        public static final double kMaxSpeedMetersPerSecond = 1.5; 
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.75;

        public static final double ksVolts = 0.60992;
        public static final double kgVolts = 0.080289;
        public static final double kvVoltSecondsPerMeter = 10.998;
        public static final double kaVoltSecondsSquaredPerMeter = 0.39154;
        public static final double kP = 0.6711; //kP and kI may need to be tuned a little bit more just to get a better hang
        public static final double kI = 0.00;

        public static final double kGearing = 16.37; 
        public static final double kMetersPerRotation = 0.15; 
        public static final double kSensorUnitsPerRotation = 2048*kGearing;
    }

    // Limelight related constants
    public static final class LimelightConstants {
        public static final int driverPipe = 1;
        public static final int targetPipe = 0;
        
        public static final double kP = 0.03;
        public static final double kI = 0;
        public static final double kD = 0;

        //Reduced these for safety until we are comfortable withe autoaim command
        public static final double minOutput = -0.5;
        public static final double maxOutput = 0.5;
        // 1 degree angle of error which is considered tolerable for the PID
        public static final double threshold = 1.0;
    }
}