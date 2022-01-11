/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class Logitech extends XboxController {
    private final double deadband = 0.1;

    public enum DPadState {
        NEUTRAl, UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT, LEFT, UP_LEFT;
    }

    public Logitech(int port) {
        super(port);
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

    // gets angle of the DPad on the XBox controller pressed with increments of 45 degree angle. 
    // returns neutal or -1 when nothing is pressed
    public DPadState getDPadState() {
        
        int angle = getPOV();

        if(angle == 0) {
            return DPadState.UP;
        } 
        else if(angle == 45){
            return DPadState.UP_RIGHT;
        }
        else if(angle == 90) {
            return DPadState.RIGHT;
        }
        else if(angle == 135){
            return DPadState.DOWN_RIGHT;
        }
        else if(angle == 180) {
            return DPadState.DOWN;
        }
        else if(angle == 225){
            return DPadState.DOWN_LEFT;
        }
        else if(angle == 270) {
            return DPadState.LEFT;
        }
        else if(angle == 315){
            return DPadState.UP_LEFT;
        } 
        else {
            return DPadState.NEUTRAl;
        }
        
    }
}
