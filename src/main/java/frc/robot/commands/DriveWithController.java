// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.Logitech;
import frc.robot.subsystems.Drivetrain;

public class DriveWithController extends CommandBase {
  private final Drivetrain drivetrain;
  private final Logitech controller;

  private final int powerLevel;

  /** Creates a new DriveWithController. */
  public DriveWithController(Drivetrain driveSubsystem, Logitech controllerParam, int powerMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = driveSubsystem;
    addRequirements(driveSubsystem);
    controller = controllerParam;
    powerLevel = powerMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -controller.getLeftStickY();
    double y = -controller.getRightStickY();
    if(powerLevel == 2){
      drivetrain.drive(3*x/4,3*y/4);
    }else if(powerLevel == 1){
      drivetrain.drive(x/3,y/3);
    }else if(powerLevel == 0){
      drivetrain.drive(x/4, y/4);
    }
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
