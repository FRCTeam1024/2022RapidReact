// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends CommandBase {
  private final Drivetrain drivetrain;
  private final Joystick leftJoystick, rightJoystick;
  
  private final SlewRateLimiter leftFilter = new SlewRateLimiter(10);
  private final SlewRateLimiter rightFilter = new SlewRateLimiter(10);

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drivetrain driveSubsystem, Joystick leftJoystickParam, Joystick rightJoystickParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = driveSubsystem;
    addRequirements(driveSubsystem);
    leftJoystick = leftJoystickParam;
    rightJoystick = rightJoystickParam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drivetrain.drive(leftFilter.calculate(leftJoystick.getY()), rightFilter.calculate(rightJoystick.getY()));
    drivetrain.drive(leftJoystick.getY(), rightJoystick.getY());
  }

  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
