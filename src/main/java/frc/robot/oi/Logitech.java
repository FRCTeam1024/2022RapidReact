/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController; //DP: I wonder if we really need to extend XboxController 
                                             //or if we could just extend GenericHID
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class defines everything that is specific to the Logitech controller.
 * This should allow adding as many controllers as we want without repeating
 * any of this setup which is independent of command binding which will be handled
 * elsewhere.
 */
public class Logitech extends XboxController {
    private final double deadband = 0.1;

    //Moved these here since they are specific to the Logitech controller and will
    //not vary from robot to robot.
    private static final int X_BUTTON_ID = 1;
    private static final int A_BUTTON_ID = 2;
    private static final int B_BUTTON_ID = 3;        
    private static final int Y_BUTTON_ID = 4;
    // Bumpers
    private static final int LEFT_BUMPER_ID = 5;
    private static final int RIGHT_BUMPER_ID = 6;
    // Triggers
    private static final int LEFT_TRIGGER_ID = 7;
    private static final int RIGHT_TRIGGER_ID = 8;
    // Start and back button
    private static final int BACK_BUTTON_ID = 9;
    private static final int START_BUTTON_ID = 10;
    // Joystick buttons (press down on joystick)
    private static final int LEFT_STICK_BUTTON_ID = 11;
    private static final int RIGHT_STICK_BUTTON_ID = 12;

    public final  JoystickButton aButton;
    public final JoystickButton bButton;
    public final JoystickButton xButton;
    public final JoystickButton yButton;
  
    //I think we can use POVButton rather that getPOV()
    // and checking the angle.  This way these can work
    // just like all other buttons.
    public final POVButton dPadUp;
    public final POVButton dPadDown;
    public final POVButton dPadLeft;
    public final POVButton dPadRight;
  
    public final JoystickButton leftTrigger;
    public final JoystickButton rightTrigger;
  
    public final JoystickButton leftBumper;
    public final JoystickButton rightBumper;
    
    public final JoystickButton backButton;
    public final JoystickButton startButton;

    public final JoystickButton leftStickButton;
    public final JoystickButton rightStickButton;

    public Logitech(int port) {
        super(port);

        aButton = new JoystickButton(this, A_BUTTON_ID);
        bButton = new JoystickButton(this, B_BUTTON_ID);
        xButton = new JoystickButton(this, X_BUTTON_ID);
        yButton = new JoystickButton(this, Y_BUTTON_ID);

        dPadUp = new POVButton(this,0);
        dPadDown = new POVButton(this,180);
        dPadRight = new POVButton(this,90);
        dPadLeft = new POVButton(this,270);
    
        leftTrigger = new JoystickButton(this, LEFT_TRIGGER_ID);
        rightTrigger = new JoystickButton(this, RIGHT_TRIGGER_ID);
    
        leftBumper = new JoystickButton(this, LEFT_BUMPER_ID);
        rightBumper = new JoystickButton(this, RIGHT_BUMPER_ID);
    
        backButton = new JoystickButton(this, BACK_BUTTON_ID);
        startButton = new JoystickButton(this, START_BUTTON_ID);

        leftStickButton = new JoystickButton(this, LEFT_STICK_BUTTON_ID);
        rightStickButton = new JoystickButton(this, RIGHT_STICK_BUTTON_ID);
    }

    public double getLeftStickY() {
        //return super.getRawAxis(XboxController.Axis.kLeftY.value);
        if(Math.abs(getRawAxis(1)) > deadband) {
            return getRawAxis(1); // got this off driver station
        } else {
            return 0;
        }
    }

    public double getRightStickY() {
        // return super.getRawAxis(XboxController.Axis.kRightY.value);
        if(Math.abs(getRawAxis(3)) > deadband) {
            return getRawAxis(3); // got this off driver station
        } else {
            return 0;
        }
    }
}
