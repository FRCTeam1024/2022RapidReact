// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ByteAPult;
import frc.robot.subsystems.Intake;

public class StowIntake extends CommandBase {
  /** Creates a new LoadByteAPult. */
  ByteAPult byteAPult;
  Intake intake;
  public StowIntake(Intake intakeParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    intake = intakeParam;
    addRequirements(intake);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    intake.stow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
