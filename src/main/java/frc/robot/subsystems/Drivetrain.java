// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX driveLeftFollower = new WPI_TalonFX(Constants.DriveConstants.driveLeftFollowerID);
  private final WPI_TalonFX driveLeftLeader = new WPI_TalonFX(Constants.DriveConstants.driveLeftLeaderID);
  private final WPI_TalonFX driveRightFollower = new WPI_TalonFX(Constants.DriveConstants.driverRightFollowerID);
  private final WPI_TalonFX driveRightLeader = new WPI_TalonFX(Constants.DriveConstants.driveRightLeaderID);

  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(driveRightLeader, driveRightFollower);
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(driveLeftLeader, driveLeftFollower);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final PigeonIMU pigeon;

  private final DifferentialDriveOdometry m_odometry;

  // Array which will contain the yaw, pitch, and roll values of the drivetrain will be stored.
  // Should be passed as pigeon.getYawPitchRoll(yawPitchRoll);
  private double[] yawPitchRoll = new double[3];

  /** Creates a new DriveTrain. */
  public Drivetrain() {

    // Reset to default
    driveLeftFollower.configFactoryDefault();
    driveLeftLeader.configFactoryDefault();
    driveRightFollower.configFactoryDefault();
    driveRightLeader.configFactoryDefault();

    driveLeftFollower.setNeutralMode(NeutralMode.Brake);
    driveLeftLeader.setNeutralMode(NeutralMode.Brake);
    driveRightFollower.setNeutralMode(NeutralMode.Brake);
    driveRightLeader.setNeutralMode(NeutralMode.Brake);

    driveLeftFollower.setInverted(false);
    driveLeftLeader.setInverted(false);
    driveRightFollower.setInverted(true); // these were set to true for driving. When testing with the trajectory code,
                                          // they're having issues when set to true.
    driveRightLeader.setInverted(true);

    driveLeftFollower.follow(driveLeftLeader);
    driveRightFollower.follow(driveRightLeader);

    driveLeftLeader.setSensorPhase(false);
    driveRightLeader.setSensorPhase(false);

    pigeon = new PigeonIMU(4);
    resetEncoders();
    zeroHeading();

    m_odometry = new DifferentialDriveOdometry(getRotation2d()); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(getRotation2d(), 
        (driveLeftLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation),
        (driveRightLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation));
      
        SmartDashboard.putNumber("Raw Left Encoder", driveLeftLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("Raw Right Encoder", driveRightLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("Average Encoder Distance", getAverageEncoderDistance());
        SmartDashboard.putNumber("Gyro Angle", getHeading()); 

  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

   /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    driveLeftLeader.setVoltage(leftVolts);
    driveRightLeader.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Returns the speed on both sides of the drivetrain, as well as reporting to SmartDashboard
   * 
   * @return Speed of both sides of the drivetrain
   */

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(10 * driveLeftLeader.getSelectedSensorVelocity() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation, 
                                            10 * driveRightLeader.getSelectedSensorVelocity() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation);
  }

  /**
   * @return Average distance of the two sides of the drivetrain
   */

  public double getAverageEncoderDistance() {
    return ((driveLeftLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation) 
            + (driveRightLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation)) / 2.0;
  }

  /**
   * Returns a current Rotation2d object
   * 
   * @return the current Rotation2d based on gyro yaw 
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(Constants.PI * getHeading()/180);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees
   */
  public double getHeading() {
    pigeon.getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[0]; 
  }

  public void zeroHeading() {
    pigeon.setYaw(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getRotation2d());
  }

  public void resetEncoders() {
    driveLeftLeader.setSelectedSensorPosition(0);
    driveRightLeader.setSelectedSensorPosition(0);
  }

  /* 
   * Drive command for use with joystick drive and debugging purpose
   * Do not use for auto routines as this will not be repeatable
   */
  public void drive(double leftPower, double rightPower) {
    driveLeftLeader.set(ControlMode.PercentOutput, leftPower);
    driveRightLeader.set(ControlMode.PercentOutput, rightPower);
    m_drive.feed();
  }
}
