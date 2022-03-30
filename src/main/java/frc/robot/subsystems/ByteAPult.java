// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.LinearFilter;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

public class ByteAPult extends SubsystemBase {

  private final Solenoid launcherLeft = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveA);
  private final Solenoid launcherRight = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.launchValveB);
  private final Solenoid launchPivotUp = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.aimValveUp);

  private final Compressor compressor = new Compressor(Constants.PCMID, PneumaticsModuleType.CTREPCM);

  private final CANSparkMax loadGate = new CANSparkMax(Constants.ShooterConstants.loadMotorID, MotorType.kBrushless);

  private final AnalogInput pressureSensor = new AnalogInput(Constants.ShooterConstants.kPressureAnalogID);
  

  private final DigitalInput loaded1 = new DigitalInput(Constants.ShooterConstants.loaded1DigID);
  private final DigitalInput loaded2 = new DigitalInput(Constants.ShooterConstants.loaded2DigID);

  private final DigitalInput armDown1 = new DigitalInput(Constants.ShooterConstants.arm1DigID);
  private final DigitalInput armDown2 = new DigitalInput(Constants.ShooterConstants.arm2DigID);

  private Timer lastLaunch;
  private LinearFilter pressureFilter = LinearFilter.singlePoleIIR(1,0.02);
 
  /** Creates a new Shooter. */
  public ByteAPult() {

    //get in initial positions
    retract();
    setNear();
    closeGate();

    //Set intitial states and timers
    lastLaunch = new Timer();

    lastLaunch.reset();
    lastLaunch.start();

    loadGate.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
    loadGate.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
    loadGate.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);

    pressureFilter.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
      lastLaunch.reset();
    }
  }

  public void retract() {
    launcherLeft.set(false);
    launcherRight.set(false);
  }

  // Extend launch pivot to shoot higher
  public void setNear() {
      launchPivotUp.set(false);
  }

  // Retract launch pivot to shoot lower
  public void setFar() {
    launchPivotUp.set(true);
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

  public void reverseGate(){
    loadGate.set(-Constants.ShooterConstants.kLoadSpeed);
  }

  public boolean armSensorConnected(){
    return armDown1.get() != armDown2.get();
  }

  /**
   * This method should confirm we are ready to launch. By confirming the 
   * launcher is loaded and that sufficient pressure is available.
   * 
   * @return TRUE if a ball is in the ByteAPult
   */
  
  public boolean readyToLaunch(double minPressure) {
    return cargoPresent() && getPressure() > minPressure;
  }

  /**
   * This method should use armSensor to determine
   * that the ByteAPult is retracted.  Then if it is also empty
   * it is ready to load
   * 
   * @return TRUE if arms is retracted and no cargo is present
   */
  public boolean readyToLoad() {
    return armRetracted() && !cargoPresent();
  }

  /**
   * Checks if the arm is retracted
   * 
   * @return TRUE if arm retracted
   */
  public boolean armRetracted() {
    //If neither launch valve is active and it has been at least 0.8 seconds 
    //since the last launch, then assume the arm is retracted.
    if(armSensorConnected()){
      if(!armDown1.get() && armDown2.get()){
        return true;
      }else if(armDown1.get() && !armDown2.get()){
        return false;
      }else{
        return true;
      }
    }else{
      return  !launcherLeft.get() && !launcherRight.get() && launchPivotUp.get() && lastLaunch.get() > 0.8;
    }
  }


  /**
   * Checks if cargo is in the launcher.
   * 
   * @return TRUE if cargo present
   */
  public boolean cargoPresent() {
    boolean cargoLoaded;

    if(loaded1.get() && !loaded2.get()){
      cargoLoaded = false;
    }else if(!loaded1.get() && loaded2.get()){
      cargoLoaded = true;
    }else{
      return true;
    }
    return cargoLoaded;
  }

  public boolean returnLoaded1(){
    return loaded1.get();
  }

  public boolean returnLoaded2(){
    return loaded2.get();
  }

  public boolean returnArm1(){
    return armDown1.get();
  }

  public boolean returnArm2(){
    return armDown2.get();
  }

  /**
   * This method should read the analog pressureSensor value, 
   * convert it to units of PSI and return the value
   * 
   * @return The system high side pressure in PSI
   */
  public double getPressure() {

    double pressure = 285 * (pressureSensor.getVoltage() / Constants.ShooterConstants.kInputVoltage) - 25;

    //Error check the sensor by looking for a descrepency between the reading and 
    //the state of the pressure switch.  If there is, assume the pressure switch is
    //correct and return a high value, otherwise return the measured pressure.
    if(pressure < 60 && compressor.getPressureSwitchValue()) {
      pressure = 120;
    }
    return Math.floor(pressureFilter.calculate(pressure));
  }

  /**
   * This method returns the output current of the feeder motor
   * 
   * @return The output current in amps
   * 
   */
  public double getGateCurrent() {
    return loadGate.getOutputCurrent();
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
