// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

public class ByteAPult extends SubsystemBase {

  private final Solenoid launcherLeft = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveA);
  private final Solenoid launcherRight = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveB);
  private final Solenoid launchPivot = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveC);

  private final ColorSensorV3 cargoSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorSensorV3 armSensor = new ColorSensorV3(I2C.Port.kMXP);

  private final DigitalInput limitSwitch = new DigitalInput(1);

  /** Creates a new Shooter. */
  public ByteAPult() {
    // Set pulse duration for a quarter of a second
  }

  /**
   * Sets the time in between opening and closing the solenoid (a pulse)
   * @param delay the time between opening and closing, in seconds
   */
  private void setLaunchDuration(double delay) {
    launcherLeft.setPulseDuration(delay);
    launcherRight.setPulseDuration(delay);
  }

  public void launch() {
    setLaunchDuration(0.25);
    launcherLeft.startPulse();
    launcherRight.startPulse();
  }

  public void retract() {
    launcherLeft.set(false);
    launcherRight.set(false);
  }

  // Extend launch pivot to shoot higher
  public void setHigh() {
    launchPivot.set(true);
  }

  // Retract launch pivot to shoot lower
  public void setLow() {
    launchPivot.set(false);
  }

  /**
   * WIP - This method should use the cargosensor to 
   * determine if a ball is loaded or not
   * 
   * @return TRUE if a ball is in the ByteAPult
   */
  public boolean loaded() {
    return true;
  }

  /**
   * WIP - this method should use armSensor to determine
   * that the ByteAPult is retracted.  Then if it is also empty
   * it is ready to load
   * 
   * @return
   */
  public boolean readyToLoad() {
    return true;
  }

  public Color getColor() {
    return cargoSensor.getColor();
  }

  public int getDistance() {
    return cargoSensor.getProximity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }

}
