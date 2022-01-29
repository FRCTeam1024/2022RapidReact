// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
                                                      HangerConstants.ksVolts,
                                                      HangerConstants.kgVolts,
                                                      HangerConstants.kvVoltSecondsPerMeter,
                                                      HangerConstants.kaVoltSecondsSquaredPerMeter);

  /** Creates a new Climber. */
  public Hanger() {

    super(
          new ProfiledPIDController(
              HangerConstants.kP,0,0,
              new TrapezoidProfile.Constraints(
                HangerConstants.kMaxSpeedMetersPerSecond,
                HangerConstants.kMaxAccelerationMetersPerSecondSquared)),

              0);

  //DP: work on implementing the rest of constructor
  // and overriding the needed methods as shown in the examples.

    hookLiftLeader.configFactoryDefault();
    hookLiftFollower.configFactoryDefault();

    hookLiftLeader.setNeutralMode(NeutralMode.Brake);
    hookLiftFollower.setNeutralMode(NeutralMode.Brake);

    hookLiftFollower.follow(hookLiftLeader);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** 
   * Runs motors to lift the hook up
   */
  private void liftHook() {

  }

  /**
   * 
   * 
   * Runs motors opposite direction to lift the robot up
   */
  private void robotClimb(){

  }

  /**
   * Runs the the horizontal hook motors up to the next bar
   */
  private void extendHook() {

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
