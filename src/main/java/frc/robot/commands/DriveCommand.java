// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {

  private final Drivetrain drivetrain;
  private final double leftPower, rightPower;

  /** Creates a new DriveCommand. */
  public DriveCommand(Drivetrain driveSubsystem, double leftPowerParam, double rightPowerParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = driveSubsystem;
    addRequirements(driveSubsystem);

    leftPower = leftPowerParam;
    rightPower = rightPowerParam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(leftPower, rightPower);
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
