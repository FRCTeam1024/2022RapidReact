// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnToHeading extends CommandBase {
  /** Creates a new AutoCompareAngles. */
  double goalAngle;
  double readAngle;
  Drivetrain drive;
  boolean finished = false;
  public TurnToHeading(Drivetrain drivetrain, double goalHeading) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    drive = drivetrain;
    goalAngle = goalHeading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    readAngle = drive.getHeading();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //Check the current heading
    readAngle = drive.getHeading();

    //Check if the current angle is within range, command a turn if out of range
    if(readAngle < (goalAngle - 0.5)) {
      drive.drive(-0.1, 0.1); //DP is 0.1 enough power to make any movement?
    }else if(readAngle > (goalAngle + 0.5)) {
      drive.drive(0.1, -0.1);
    }else{
      finished = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
