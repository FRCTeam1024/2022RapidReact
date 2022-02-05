// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Intake extends SubsystemBase {

  private final UsbCamera intakeCamera;

  //We need to create these, and use them in the methods outlined below

  //Intake rollers:  1 falcon 
  //cargo shifter is a NEO550
  //extend/retract cylinder (intakeValve)

  private final WPI_TalonFX intake = new WPI_TalonFX(Constants.IntakeConstants.intakeMotorID);
  private final CANSparkMax shifter = new CANSparkMax(Constants.IntakeConstants.shifterMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final Solenoid intakeValve = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.intakeValve);

  /** Creates a new Intake. */
  public Intake() {
    intakeCamera = CameraServer.startAutomaticCapture();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * WIP - Run intake to speed and direction sepecified.  This is the only
   * place the intake motor should be set.  Probaly will make this private.
   * 
   * @param speed %power for intake motor (+ = collect)
   */
  public void runIntake(double speed) {

  }

  /**
   * WIP - Run the cargo shifter to speed and direction specified.  This is
   * the only place the shifter motor should be set.  Probably will make this private.
   * 
   * @param speed %power for the shifter motor (+ towards launcher)
   */
  public void runShifter(double speed) {

  }

  /**
  * Run intake forward to collect Cargo at speed defined by constant
  * */
  public void collect() {
    runIntake(IntakeConstants.kCollectSpeed);
    runShifter(IntakeConstants.kShifterSpeed);
  }

  /**
   * Run intake in reverse to eject/unjam Cargo at speed defined by constant
   */
  public void eject() {
    runIntake(IntakeConstants.kEjectSpeed);
    runShifter(-IntakeConstants.kShifterSpeed);
  }

  /**
   * Extend the intake, use the cylinder
   */
  public void extend() {

  }

  /**
   * Retract the intake, use the cylinder
   */
  public void retract() {

  }

  /**
   * Retract the intake and stop rotation
   */
  public void stow() {
    runIntake(0);
    runShifter(0);
    retract();
  }

  /**
   * Retract and stop the intake (if not already) but keep running the 
   * shifter in order to feed cargo to the launcher
   */
  public void feedLauncher() {
    runIntake(0);
    runShifter(IntakeConstants.kShifterSpeed);
    retract();
  }

  /**
   * Extend the intake and run in collect mode
   */
  public void deploy() {
    extend();
    collect();
  }

  // Alex: see line 14
  public UsbCamera getFeed() {
    return intakeCamera;
  }
}
