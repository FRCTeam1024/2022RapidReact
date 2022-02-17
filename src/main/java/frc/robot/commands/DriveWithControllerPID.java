// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.oi.Logitech;
import frc.robot.subsystems.Drivetrain;

public class DriveWithControllerPID extends CommandBase {
  private final Drivetrain drivetrain;
  private final Logitech controller;

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);
  private final PIDController drivePIDController = new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0);

  /** Creates a new DriveWithController. */
  public DriveWithControllerPID(Drivetrain driveSubsystem, Logitech controllerParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = driveSubsystem;
    addRequirements(driveSubsystem);
    controller = controllerParam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = MathUtil.clamp(controller.getLeftStickY(), Constants.DriveConstants.minSpeed, Constants.DriveConstants.maxSpeed);
    double rightSpeed =  MathUtil.clamp(controller.getRightStickY(), Constants.DriveConstants.minSpeed, Constants.DriveConstants.maxSpeed);
    drivetrain.drive(
        drivePIDController.calculate(drivetrain.getLeftWheelSpeed(), leftSpeed) + driveFeedforward.calculate(leftSpeed),
        drivePIDController.calculate(drivetrain.getRightWheelSpeed(), rightSpeed) + driveFeedforward.calculate(rightSpeed)
    );
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
