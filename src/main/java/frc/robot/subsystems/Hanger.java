// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hanger extends SubsystemBase {
  /** Creates a new Climber. */
  /** Motors */
  private final DigitalInput limitSwitch = new DigitalInput(0);

  public Hanger() {

  }

  /**
   * Runs motors to lift the hook up
   */
  private void liftHook() {
    
  }

  /**
   * Runs motors opposite direction to lift the robot up
   */
  private void robotClimb(){
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }
}
