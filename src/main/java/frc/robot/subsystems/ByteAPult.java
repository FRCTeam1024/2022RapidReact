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
import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class ByteAPult extends SubsystemBase {

  private final Solenoid launcherLeft = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveA);
  private final Solenoid launcherRight = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveB);
  private final Solenoid launchPivot = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveC);

  private final ColorSensorV3 cargoSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorSensorV3 armSensor = new ColorSensorV3(I2C.Port.kMXP);

  private final ColorMatch cargoColorMatcher = new ColorMatch();

  //Basic setup values for red and blue - obviously will need to be finetuned a bit
  private final Color red = new Color(0.808, 0.215, 0.160);
  private final Color blue = new Color(0, 0, 1);

  private final AnalogInput pressureSensor = new AnalogInput(Constants.ShooterConstants.kPressureAnalogID);

  /** Creates a new Shooter. */
  public ByteAPult() {
    // Set pulse duration for a quarter of a second
    cargoColorMatcher.addColorMatch(red);
    cargoColorMatcher.addColorMatch(blue);
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
  // Alex: Program keeps having trouble finding the color sensor, even if it is plugged into the correct port, etc.
  //       It also keeps mentioning that there are known issues with the onboard I2C port, and that it might be better to use the MXP board.
  //       It's possible that the known issues with the I2C port could cause the code to simply ignore what's plugged in? Not certain.
  public boolean loaded() {
    /**if(cargoColorMatcher.matchColor(cargoSensor.getColor()).equals(new ColorMatchResult(red, 0.95))){
      return true;
    }else{
      return false;
    }**/
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
}
