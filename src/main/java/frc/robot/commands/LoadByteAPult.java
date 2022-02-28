// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ByteAPult;
import frc.robot.subsystems.Intake;

public class LoadByteAPult extends CommandBase {
  /** Creates a new LoadByteAPult. */
  ByteAPult byteAPult;
  Intake intake;
  public LoadByteAPult(ByteAPult byteAPultParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    byteAPult = byteAPultParam;
    addRequirements(byteAPult);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    byteAPult.setNear();
    
    if(byteAPult.readyToLoad()){
      byteAPult.openGate();
    } else{
      byteAPult.closeGate();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    byteAPult.closeGate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
