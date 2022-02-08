// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightAutoAim extends PIDCommand {
  private final Limelight limelight;
  private final Drivetrain drivetrain;

  /** Creates a new LimelightAutoAim. */
  public LimelightAutoAim(Limelight limelightParam, Drivetrain drivetrainParam) {
    super(
        // The controller that the command will use
        limelightParam.getPIDController(),
        // This should return the measurement
        () -> limelightParam.getXOffset(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          double adjustedOutput = MathUtil.clamp(output, Constants.LimelightConstants.minOutput, Constants.LimelightConstants.maxOutput);
          drivetrainParam.drive(-adjustedOutput, adjustedOutput); // We want to be turning, so outputs are opposite
        });
    // Use addRequirements() here to declare subsystem dependencies.
    limelight = limelightParam;
    drivetrain = drivetrainParam;
    addRequirements(limelight, drivetrain);

    limelight.enableLeds(); // Turn on the lights
    limelight.setTargetPipe(); // Target the hub
    // Configure additional PID options by calling `getController` here.
    limelight.getPIDController().setTolerance(Constants.LimelightConstants.threshold); // Sets the error which is considered tolerable for use with atSetpoint().
  }

  @Override
  public void end(boolean interrupted){
    limelight.disableLeds(); // Turn the lights off once command ends, since they are no longer necessary
    limelight.setDriverPipe(); // Switch back to driving mode for driver convenience
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.getPIDController().atSetpoint();
  }
}
