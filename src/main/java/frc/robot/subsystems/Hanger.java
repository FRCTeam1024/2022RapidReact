// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import frc.robot.Constants.HangerConstants;

public class Hanger extends ProfiledPIDSubsystem {  //DP: See WPIlib examples, I think we want to use this

  private final WPI_TalonFX hookLiftLeader = new WPI_TalonFX(Constants.HangerConstants.hookLiftLeaderID);
  private final WPI_TalonFX hookLiftFollower = new WPI_TalonFX(Constants.HangerConstants.hookLiftFollowerID);

  //Creating a motor group so as not to rely on the follower remaining configured to follow
  //as we have seen the follower suddenly drop out of follow mode in the drivetrain.
  private final MotorControllerGroup hookLiftMotors = new MotorControllerGroup(hookLiftLeader,
                                                                              hookLiftFollower);                         

  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
                                                      HangerConstants.ksVolts,
                                                      HangerConstants.kgVolts,
                                                      HangerConstants.kvVoltSecondsPerMeter,
                                                      HangerConstants.kaVoltSecondsSquaredPerMeter);

  /** Creates a new Climber. */
  /** Motors */
  private final DigitalInput limitSwitch = new DigitalInput(0);

  public Hanger() {

    super(
          new ProfiledPIDController(
              HangerConstants.kP,0,0,
              new TrapezoidProfile.Constraints(
                HangerConstants.kMaxSpeedMetersPerSecond,
                HangerConstants.kMaxAccelerationMetersPerSecondSquared)),

              0);

    hookLiftLeader.configFactoryDefault();
    hookLiftFollower.configFactoryDefault();

    hookLiftLeader.setNeutralMode(NeutralMode.Brake);
    hookLiftFollower.setNeutralMode(NeutralMode.Brake);

    hookLiftFollower.follow(hookLiftLeader);

    hookLiftLeader.setSelectedSensorPosition(0);

    setGoal(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) { 
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    //DP:  Add a check here of the limit switch and only apply voltage if limit switch
    // is not detected in the direction that is being commanded.  If limit is detected 
    // then don't apply voltage and call disable().
    hookLiftMotors.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return hookLiftLeader.getSelectedSensorPosition()*HangerConstants.kSensorUnitsPerRotation
                                                    *HangerConstants.kMetersPerRotation;
  }

  /**
   * Moves the carriage to the requested position (meters from bottom)
   * 
   * @param pos the desired position in meters
   */
  public void moveCarriage(double pos) {
    setGoal(pos);
    enable();
  }

  /** 
   * Runs motors to move carriage (and hook)
   */
  public void driveCarriage(double power) {
    //disable the PID controller so we aren't fighting it while trying to 
    //directly control the motors
    disable();

    //DP:  added a check of the limit switch here first
    hookLiftMotors.set(power);
  }

  /**
   * Runs the the horizontal hook motors up to the next bar
   */
  private void extendHook() {

  }

  // DP: If we only usse a single switch we need to create a way
  // within these two methods to determine wheather the carriage is at the top or bottom
  // of its travel so that motion is only restricted in one direction depending on
  // which limit is hit.
  // Need to determine if there will be 1 or 2 limit switches.
  private boolean atTopLimit(){
    return false;
  }

  private boolean atBottomLimit(){
    return false;
  }

  /**
   * Returns the limit switch state, don't call directly, use atTopLimit and atBottomLimit 
   * In order to know which extreme is hit.
   * 
   * @return the state of the limit switch
   */
  private boolean getLimitSwitch(){
    return limitSwitch.get();
  }
  /**
   * Runs the horizontal hook motors to pull the robot to the next bar
   */
  private void climbNextBar() {

  }

  /**
   * Resets motors to original state before initiating climb
   */
  private void resetClimb() {

  }


}
