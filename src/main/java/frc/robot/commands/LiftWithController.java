// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.Logitech;
import frc.robot.subsystems.Hanger;

public class LiftWithController extends CommandBase {
  private final Hanger hanger;
  private final Logitech controller;

  private final SlewRateLimiter filter = new SlewRateLimiter(1);

  /** Creates a new DriveWithController. */
  public LiftWithController(Hanger hangSubsystem, Logitech controllerParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    hanger = hangSubsystem;
    addRequirements(hangSubsystem);
    controller = controllerParam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hanger.driveCarriage(filter.calculate(-controller.getLeftStickY()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hanger.driveCarriage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
