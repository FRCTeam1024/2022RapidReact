// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX driveLeftFollower = new WPI_TalonFX(Constants.DriveConstants.driveLeftFollowerID);
  private final WPI_TalonFX driveLeftLeader = new WPI_TalonFX(Constants.DriveConstants.driveLeftLeaderID);
  private final WPI_TalonFX driveRightFollower = new WPI_TalonFX(Constants.DriveConstants.driverRightFollowerID);
  private final WPI_TalonFX driveRightLeader = new WPI_TalonFX(Constants.DriveConstants.driveRightLeaderID);

  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(driveRightLeader, driveRightFollower);
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(driveLeftLeader, driveLeftFollower);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final WPI_PigeonIMU pigeon;

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
    driveRightFollower.setInverted(true); 
    driveRightLeader.setInverted(true);

    driveLeftFollower.follow(driveLeftLeader);
    driveRightFollower.follow(driveRightLeader);

    driveLeftFollower.set(ControlMode.Follower, Constants.DriveConstants.driveLeftLeaderID);
    driveRightFollower.set(ControlMode.Follower, Constants.DriveConstants.driveRightLeaderID);

    driveLeftLeader.setSensorPhase(false);
    driveRightLeader.setSensorPhase(false);

    pigeon = new WPI_PigeonIMU(4);
    resetEncoders();
    zeroHeading();

    m_odometry = new DifferentialDriveOdometry(getGyroRotation2d()); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(getGyroRotation2d(), 
        (driveLeftLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation),
        (driveRightLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation));
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
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
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
   * @return Distance of the left drivetrain motors
   */
  public double getLeftEncoderDistance() {
    return driveLeftLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation;
  }

  /**
   * @return Distance of the right drivetrain motors
   */
  public double getRightEncoderDistance() {
    return driveRightLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation;
  }

  /**
   * @return Average distance of the two sides of the drivetrain
   */
  public double getAverageEncoderDistance() {
    return ((driveLeftLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation) 
            + (driveRightLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation)) / 2.0;
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getGyroRotation2d());
  }

  public void resetEncoders() {
    driveLeftLeader.setSelectedSensorPosition(0);
    driveRightLeader.setSelectedSensorPosition(0);
  }

  /** 
   * Drive command for use with joystick drive and debugging purpose
   * Do not use for auto routines as this will not be repeatable
   */
  public void drive(double leftPower, double rightPower) {
    m_leftMotors.set(leftPower);
    m_rightMotors.set(rightPower);
    m_drive.feed();
  }

  /**
   * Returns the robot heading in degrees
   * 
   * @return the robot heading in degrees
   */
  public double getHeading() {
    return getPose().getRotation().getDegrees();
  }

  /**
   * @return Speed of left side of drivetrain
   */

  public double getLeftWheelSpeed(){
    return driveLeftLeader.getSelectedSensorVelocity();
  }

  /**
   * @return Speed of right side of drivetrain
   */

  public double getRightWheelSpeed(){
    return driveRightLeader.getSelectedSensorVelocity();
  }

    /**
   * Returns a current Rotation2d object
   * 
   * @return the current Rotation2d based on gyro yaw 
   */
  private Rotation2d getGyroRotation2d() {
    return new Rotation2d(Constants.PI * getGyroAngle()/180);
  }

  /**
   * Returns the gyro angle
   *
   * @return the robot's heading in degrees
   */
  private double getGyroAngle() {
    pigeon.getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[0]; 
  }

  /**
   * Resets the gyro to a heading of 0 degrees
   * USE WITH CAUTION - WILL UPSET ODOMETRY
   */
  private void zeroHeading() {
    pigeon.setYaw(0);
  }
}
