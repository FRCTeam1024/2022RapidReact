// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import frc.robot.Constants.HangerConstants;

public class Hanger extends ProfiledPIDSubsystem { 

  private final WPI_TalonFX hookLiftLeader = new WPI_TalonFX(Constants.HangerConstants.hookLiftLeaderID);
  private final WPI_TalonFX hookLiftFollower = new WPI_TalonFX(Constants.HangerConstants.hookLiftFollowerID);

  private final MotorControllerGroup hookLiftMotors = new MotorControllerGroup(hookLiftLeader,
                                                                              hookLiftFollower);        
                                                                              
  private final DoubleSolenoid monkeyArm = new DoubleSolenoid(Constants.PCMID,PneumaticsModuleType.CTREPCM, Constants.HangerConstants.monkeyArmValveOut, Constants.HangerConstants.monkeyArmValveIn);

  private final Solenoid powerHook = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.HangerConstants.powerHook);

  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
                                                      HangerConstants.ksVolts,
                                                      HangerConstants.kgVolts,
                                                      HangerConstants.kvVoltSecondsPerMeter,
                                                      HangerConstants.kaVoltSecondsSquaredPerMeter);

  /** Creates a new Climber. */
  /** Motors */
  private final DigitalInput topLimit = new DigitalInput(HangerConstants.topLimitDigID);
  private final DigitalInput bottomLimit = new DigitalInput(HangerConstants.bottomLimitDigID);

  private boolean hangMode = true;

  public Hanger() {

    super(
          new ProfiledPIDController(
              HangerConstants.kP,HangerConstants.kI,0,
              new TrapezoidProfile.Constraints(
                HangerConstants.kMaxSpeedMetersPerSecond,
                HangerConstants.kMaxAccelerationMetersPerSecondSquared)),
              0);

    hookLiftLeader.configFactoryDefault();
    hookLiftFollower.configFactoryDefault();

    hookLiftLeader.setNeutralMode(NeutralMode.Brake);
    hookLiftFollower.setNeutralMode(NeutralMode.Brake);

    hookLiftFollower.follow(hookLiftLeader);
    hookLiftFollower.setStatusFramePeriod(StatusFrame.Status_1_General, 1000);
    hookLiftFollower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);


    hookLiftLeader.setSelectedSensorPosition(0);
    hookLiftFollower.setSelectedSensorPosition(0);

    setGoal(0);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) { 
    double feedforward = m_feedforward.calculate(setpoint.velocity);
    double volts = output + feedforward;
 
    //Check limits switches and apply power if not at limit
    if(atTopLimit() && volts > 0 ) 
        hookLiftMotors.setVoltage(0);
    else if(atBottomLimit() && volts < 0)
        hookLiftMotors.setVoltage(0);
    else{
        hookLiftMotors.setVoltage(volts);
    }
  }

  @Override
  public double getMeasurement() {
    return hookLiftLeader.getSelectedSensorPosition()/HangerConstants.kSensorUnitsPerRotation
                                                    *HangerConstants.kMetersPerRotation;
  }

  /**
   * Moves the carriage to the requested position (meters from bottom)
   * 
   * @param pos the desired position in meters
   */
  public void moveCarriage(double pos) {
    setGoal(MathUtil.clamp(pos,HangerConstants.minTravelMeters,HangerConstants.maxTravelMeters));
    enable();
  }

   /** 
   * Runs motors to move carriage (and hook)
   * 
   * @param power the percent power to apply (+ carriage up)
   */
  public void driveCarriage(double power) {
    //disable the PID controller so we aren't fighting it while trying to 
    //directly control the motors
    disable();

    //Check limits switches and apply power if not at limit
    if(power > 0) {
      if(!atTopLimit() && getMeasurement() < HangerConstants.maxTravelMeters - 0.025) 
        if(power < 0.1){
          hookLiftMotors.set(power);
        } else {
          hookLiftMotors.set(0.1);
        }
      else{
        hookLiftMotors.set(0);
      }
    }
    else if(power < 0) {
      if(!atBottomLimit())
      if(power > -0.1){
        hookLiftMotors.set(power);
      } else {
        hookLiftMotors.set(-0.1);
      }
    else{
      hookLiftMotors.set(0);
    }
    }
    else {
      hookLiftMotors.set(0);
    }
  }

  public void stopCarriage() {
    disable();
  }

  /**
   * Extends Solenoid, allowing horizontal hook to reach the next bar
   */
  public void extendHook() {
    monkeyArm.set(Value.kReverse); //should set solenoid to extend the pneumatics, and reach the next bar.
  }

  /**
   * Reverses Solenoid, bringing the horizontal hook backwards
   */
  public void retractHook(){
    monkeyArm.set(Value.kForward);
  }

  public Value getMonkeyHook(){
    return monkeyArm.get();
  }

  public void openPowerHook(){
    powerHook.set(true);
  }

  public void closePowerHook(){
    powerHook.set(false);
  }

  public boolean atTopLimit(){
    return !topLimit.get();
  }

  public boolean atBottomLimit(){
    return !bottomLimit.get();
  }

  public void setNeutral() {
    monkeyArm.set(Value.kOff);
  }

  public void setHangMode(boolean hang){
    hangMode = hang;
  }

  public boolean getHangMode(){
    return hangMode;
  }

  /**
   * Resets encoders to 0 at bottom carriage position
   * 
   * DP: Not sure if we want to use this or not.
   * 
   */
  public void resetCarriage() {
    if(atBottomLimit())
      hookLiftLeader.setSelectedSensorPosition(0);
  }

  public void lowerToBottom(){
    if(!atBottomLimit()){
      driveCarriage(-0.05);
    }else{
      driveCarriage(0);
    }
  }

}
