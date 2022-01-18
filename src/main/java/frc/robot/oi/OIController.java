// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class OIController extends SubsystemBase {
  private final Logitech controller;

  // No need to create joysticks no dpad inputs, 
  // since their values can be grabbed via controller

  private final JoystickButton aButton;
  private final JoystickButton bButton;
  private final JoystickButton xButton;
  private final JoystickButton yButton;

  /*private final JoystickButton dPadUp;
  private final JoystickButton dPadDown;
  private final JoystickButton dPadLeft;
  private final JoystickButton dPadRight;*/

  private final JoystickButton leftTrigger;
  private final JoystickButton rightTrigger;

  private final JoystickButton leftBumper;
  private final JoystickButton rightBumper;
  
  private final JoystickButton backButton;
  private final JoystickButton startButton;

  /** Creates a new Controller. */
  public OIController(int controllerID) {
    controller = new Logitech(controllerID);

    aButton = new JoystickButton(controller, Constants.Inputs.aButtonID);
    bButton = new JoystickButton(controller, Constants.Inputs.bButtonID);
    xButton = new JoystickButton(controller, Constants.Inputs.xButtonID);
    yButton = new JoystickButton(controller, Constants.Inputs.yButtonID);

    leftTrigger = new JoystickButton(controller, Constants.Inputs.leftTriggerID);
    rightTrigger = new JoystickButton(controller, Constants.Inputs.rightTriggerID);

    leftBumper = new JoystickButton(controller, Constants.Inputs.leftBumperID);
    rightBumper = new JoystickButton(controller, Constants.Inputs.rightBumperID);

    backButton = new JoystickButton(controller, Constants.Inputs.backButtonID);
    startButton = new JoystickButton(controller, Constants.Inputs.startButtonID);
  }

  public double getLeftStickY() {
    return controller.getLeftStickY();
  }

  public double getRightStickY() {
    return controller.getRightStickY();
  }



  public JoystickButton aButton() {
    return aButton;
  }
  public JoystickButton bButton() {
    return bButton;
  }
  public JoystickButton xButton() {
    return xButton;
  }
  public JoystickButton yButton() {
    return yButton;
  }

  public JoystickButton leftTrigger() {
    return leftTrigger;
  }
  public JoystickButton rightTrigger() {
    return rightTrigger;
  }

  public JoystickButton leftBumper() {
    return leftBumper;
  }
  public JoystickButton rightBumper() {
    return rightBumper;
  }

  public JoystickButton backButton() {
    return backButton;
  }
  public JoystickButton startButton() {
    return startButton;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
