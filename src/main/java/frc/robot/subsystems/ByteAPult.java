// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

public class ByteAPult extends SubsystemBase {
  
  public Solenoid launcherLeft = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveA);
  public Solenoid launcherRight = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveB);
  public Solenoid launchPivot = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveC);

  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  /** Creates a new Shooter. */
  public ByteAPult() {

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
    double onTime = .1;  //Seconds
    double offTime = .1;  //Seconds
    int cycles = 10; //# of on off cycles
    int i = 1;
    while(i <= cycles) {
      launchShooter();
      Timer.delay(onTime);
      retractShooter();
      Timer.delay(offTime);
      i = i+1;
    }
  }

  // Extend launch pivot to shoot higher
  public void setHigh() {
    launchPivot.set(true);
  }

  // Retract launch pivot to shoot lower
  public void setLow() {
    launchPivot.set(false);
  }

  public Color getColor() {
    return colorSensor.getColor();
  }

  public int getDistance() {
    return colorSensor.getProximity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
