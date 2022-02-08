// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class ByteAPult extends SubsystemBase {

  private final Solenoid launcherLeft = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveA);
  private final Solenoid launcherRight = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveB);
  private final Solenoid launchPivot = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveC);

  private final CANSparkMax loadGate = new CANSparkMax(Constants.ShooterConstants.loadMotorID, MotorType.kBrushless);

  private final ColorSensorV3 cargoSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorSensorV3 armSensor = new ColorSensorV3(I2C.Port.kMXP);

  private final ColorMatch cargoColorMatcher = new ColorMatch();

  //Basic setup values for red and blue - obviously will need to be finetuned a bit
  private final Color red = new Color(0.808, 0.215, 0.160);
  private final Color blue = new Color(0, 0, 1);

  private final AnalogInput pressureSensor = new AnalogInput(Constants.ShooterConstants.kPressureAnalogID);

  private boolean armRetracted;
  private boolean cargoLoaded;
  private int lastArm;
  private int lastCargo;

  /** Creates a new Shooter. */
  public ByteAPult() {
    cargoColorMatcher.addColorMatch(red);
    cargoColorMatcher.addColorMatch(blue);

    //get in initial positions
    retract();
    setLow();
    closeGate();

    //Set intitial states and counters
    armRetracted = true;
    cargoLoaded = false;
    lastArm = 0;
    lastCargo = 0; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Sensor debouncers
    if(armSensor.getProximity() > 700) { //700 is assumed raw sensor value, adjust as needed
      armRetracted = false;
      lastArm = 0;
    }
    else if(lastArm < 5) {  //cycles before we accept the arm really is down
      lastArm++;
    }
    else {
      armRetracted = true;
    }
    
    //Sensor debouncers
    if(cargoSensor.getProximity() < 300) { //300 is assumed raw sensor value, adjust as needed
      cargoLoaded = true;
      lastCargo = 0;
    }
    else if(lastCargo < 25) { //cycles before we accept the cargo really is gone
      lastCargo++;
    }
    else {
      cargoLoaded = false;
    }
    
  }

  /**
   * Launches the Byte-A-Pult with the specified number of cylinders
   * fired at the specified pulse duration and only if minimum pressure
   * and load sensors are happy. Also makes sure the load gate is closed first.
   * The override parameter will fire regardless
   * of pressure or load sensors.
   * 
   * @param cylinders 1 or 2, the number of cylinders to fire
   * @param duration The fire pulse duration in seconds
   * @param minPressure The minimum pressure required to fire
   * @param override TRUE if sensors should be ignored
   */
  public void launch(int cylinders, double duration, double minPressure, boolean override) {
    setLaunchDuration(duration);
    closeGate();

    if (readyToLaunch(minPressure) || override) {
      if (cylinders == 1) {
        launcherLeft.startPulse();
      }
      else {
        launcherLeft.startPulse();
        launcherRight.startPulse();
      }
    }
  }

  public void retract() {
    launcherLeft.set(false);
    launcherRight.set(false);
  }

  // Extend launch pivot to shoot higher
  public void setHigh() {
    launchPivot.set(true);  //May need to reverse boolean
  }

  // Retract launch pivot to shoot lower
  public void setLow() {
    launchPivot.set(false);  //May need to reverse boolean
  }

  // Close the load gate. use the loadMotor
  public void closeGate() {
    //Stopping the motor closes the gate as the ball can not pass the 
    //load wheel when it is not spinning.
    loadGate.set(0);   
  }

  //Open the load gate. use the loadValve
  public void openGate() {
    loadGate.set(Constants.ShooterConstants.kLoadSpeed);    //Runs motor forwards to open gate, may need to be reversed depending on how the motor is setup
  }

  /**
   * This method should confirm we are ready to launch. By confirming the 
   * launcher is loaded and that sufficient pressure is available.
   * 
   * @return TRUE if a ball is in the ByteAPult
   */
  // Alex: Program keeps having trouble finding the color sensor, even if it is plugged into the correct port, etc.
  //       It also keeps mentioning that there are known issues with the onboard I2C port, and that it might be better to use the MXP board.
  //       It's possible that the known issues with the I2C port could cause the code to simply ignore what's plugged in? Not certain.
  public boolean readyToLaunch(double minPressure) {
    /**if(cargoColorMatcher.matchColor(cargoSensor.getColor()).equals(new ColorMatchResult(red, 0.95))){
      return true;
    }else{
      return false;
    }**/
    return armRetracted() & cargoPresent() & getPressure() > minPressure;
  }

  /**
   * This method should use armSensor to determine
   * that the ByteAPult is retracted.  Then if it is also empty
   * it is ready to load
   * 
   * @return TRUE if arms is retracted and no cargo is present
   */
  public boolean readyToLoad() {
    return armRetracted() & !cargoPresent();
  }

  /**
   * Checks if the arm is retracted
   * 
   * @return TRUE if arm retracted
   */
  public boolean armRetracted() {
    return armRetracted;
  }

  public Color getColor() {
    return cargoSensor.getColor();
  }

  /**
   * Checks if cargo is in the launcher.
   * 
   * @return TRUE if cargo present
   */
  public boolean cargoPresent() {
    return cargoLoaded;
  }

  /**
   * WIP - This method should read the analog pressureSensor value, 
   * convert it to units of PSI and return the value
   * 
   * @return The system high side pressure in PSI
   */
  public double getPressure() {
    return 250 * (pressureSensor.getVoltage() / Constants.ShooterConstants.kInputVoltage) - 25;
  }

  /**
   * For color sensor testing only
   * @return the proximity value from the color sensor in raw units
   */
  public double getDistance() {
    return cargoSensor.getProximity();
  }

  /**
   * Sets the time in between opening and closing the solenoid (a pulse)
   * @param delay the time between opening and closing, in seconds
   */
  private void setLaunchDuration(double delay) {
    launcherLeft.setPulseDuration(delay);
    launcherRight.setPulseDuration(delay);
  }


}
