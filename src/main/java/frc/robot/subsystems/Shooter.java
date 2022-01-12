// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;

public class Shooter extends SubsystemBase {
  
  public Solenoid launcherLeft = new Solenoid(Constants.ShooterConstants.PCMID, PneumaticsModuleType.CTREPCM, 1);
  public Solenoid launcherRight = new Solenoid(Constants.ShooterConstants.PCMID, PneumaticsModuleType.CTREPCM, 2);

  /** Creates a new Shooter. */
  public Shooter() {

  }

  public void launchShooter() {
    launcherLeft.set(true);
    launcherRight.set(true);
  }

  public void retractShooter() {
    launcherLeft.set(false);
    launcherRight.set(false);
  }

  public void shoot() {
    launchShooter();
    Timer.delay(0.010);
    retractShooter();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
