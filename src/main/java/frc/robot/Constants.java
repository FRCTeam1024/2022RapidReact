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
        public static final int aimValveA = 3;
        public static final int aimValveB = 7;
        public static final int loadMotorID = 17;

        public static final int kPressureAnalogID = 0;

        public static final double kInputVoltage = 5.0;

        public static final double kLoadSpeed = 0.5;  //No idea if this is what we want
    }

    // Intake related constants
    public static final class IntakeConstants {
        public static final int intakeValve = 5;
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

        public static final int monkeyArmValveA = 4;
        public static final int monkeyArmValveB = 8;

        public static final int topLimitDigID = 0;
        public static final int bottomLimitDigID = 1;

        public static final double kMaxSpeedMetersPerSecond = 0.5; 
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;

        //All these copied from drivetrain, need to characterize hanger
        public static final double ksVolts = 0.59;
        public static final double kgVolts = .5;  //yet another total guess
        public static final double kvVoltSecondsPerMeter = 1.71;
        public static final double kaVoltSecondsSquaredPerMeter = 0.137;
        public static final double kP = 2.23;

        public static final double kGearing = 7.56; //Need to confirm this
        public static final double kMetersPerRotation = .05; //complete guess probably wrong
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