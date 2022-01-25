// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Arrays;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.oi.Logitech;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shooter shooter = new Shooter();
  private final Limelight limelight = new Limelight();

  // Operator Inputs
  private final Logitech driverController = new Logitech(Constants.Inputs.driverControllerID);
  private final Logitech operatorController = new Logitech(Constants.Inputs.operatorControllerID);

  // Default Commands
  private final DriveWithController driveWithController = new DriveWithController(drivetrain, driverController);

  //Create a chooser for auto
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the dashboard
    configureDashboard();
    // Assign default Commands
    drivetrain.setDefaultCommand(driveWithController);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverController.leftTrigger.whileActiveOnce(new InstantCommand(limelight::setTargetPipe, limelight));
    driverController.leftTrigger.whenInactive(new InstantCommand(limelight::setDriverPipe, limelight));
    
    driverController.rightTrigger.whileHeld(new InstantCommand(shooter::shoot, shooter));
    operatorController.xButton.whenPressed(new InstantCommand(limelight::toggleLeds, limelight));
  }

  /**
   * Use this method to configure the dashboard
   * 
   */
  private void configureDashboard() {
    //Create a ShuffleBoard Tab
    ShuffleboardTab tab = Shuffleboard.getTab("1024Dashboard");
    
    //Display the name and version number of the code.
    tab.add("Running Code Version:", BuildConfig.APP_NAME + " " + BuildConfig.APP_VERSION)
       .withSize(3,1)
       .withPosition(0,0);

    //Add commands to auto chooser, set default to null to avoid surprise operation
    m_AutoChooser.setDefaultOption("None", null);
    m_AutoChooser.addOption("Trajectory Generator Auto", getTrajectoryAuto()); 
    m_AutoChooser.addOption("Blue 3 Ball Auto", getBlueThreeBallAuto());
    m_AutoChooser.addOption("Basic Forward", getBasicForwardAuto());
    //Put the auto chooser on the dashboard
    tab.add("Auto Mode",m_AutoChooser)
       .withSize(2,1)
       .withPosition(6,0);

    //Add command status to dashboard
    tab.add("DrivetrainCommand",drivetrain)
       .withSize(2,1)
       .withPosition(8,0);

    tab.add("TurnToHeading", new TurnToHeading(drivetrain, 90))
        .withSize(3,1)
        .withPosition(3,0);

    tab.addNumber("GyroHeading", drivetrain::getHeading)
        .withSize(2,1)
        .withPosition(0,1);

    tab.add("Limelight", limelight.getFeed())
        .withSize(6,3)
        //.withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(0, 2);

    tab.add("LeftMotors", drivetrain.getLeftWheelSpeed())
        .withSize(2,1)
        .withPosition(2,1);

    tab.add("RightMotors", drivetrain.getRightWheelSpeed())
        .withSize(2,1)
        .withPosition(4,1);

    tab.addNumber("ColorSensorDistance", shooter::getDistance)
        .withSize(2,1)
        .withPosition(6,2);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_AutoChooser.getSelected();
  }

  /**
   * Create an auto command using the TrajectoryGenerator class
   * 
   * @return the auto command
   */
  private Command getTrajectoryAuto() {
    // An ExampleCommand will run in autonomous
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                             DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            //new Translation2d(1, 0)
           
            //new Translation2d(2, 0)
        ),
        new Pose2d(1, 0, new Rotation2d(90)),
        // Pass config
        config
    );

    // The actual command to follow the path
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drivetrain::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts,
        drivetrain
    );

    // Reset odometry to the starting pose of the trajectory, then Run path following command, then stop at the end.
    return ramseteCommand.beforeStarting(() -> drivetrain.resetOdometry(exampleTrajectory.getInitialPose()))
                          .andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  /**
   * Create an auto command using path(s) imported from pathweaver
   * Decorate with additional functions
   * Make copies of this method to generate alternate auto routines
   * 
   * @return the auto command
   */
  private Command getBlueThreeBallAuto() {

    //Choose paths and combine multiple as necessary
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("BlueFirstCargo.wpilib.json")]
                      .concatenate(Robot.pathList[Arrays.asList(Robot.fileList).indexOf("BlueSecondCargo.wpilib.json")])
                      .concatenate(Robot.pathList[Arrays.asList(Robot.fileList).indexOf("BlueReturnAndShoot.wpilib.json")]);

    // Reset odometry to the starting pose of the trajectory, then Run path following command, 
    // then stop at the end.
    return new PathweaverCommand(pathA,drivetrain)
                          .beforeStarting(() -> drivetrain.resetOdometry(pathA.getInitialPose()))
                          .beforeStarting(() -> drivetrain.setHeading(130))  //Define start heading (0 = Driver facing forward)
                          .andThen(() -> drivetrain.tankDriveVolts(0, 0))
                          .andThen(new TurnToHeading(drivetrain, 130));  // Enter desired end heading
  }

   /**
   * Create an auto command using path(s) imported from pathweaver
   * Decorate with additional functions
   * Make copies of this method to generate alternate auto routines
   * 
   * @return the auto command
   */
  private Command getBasicForwardAuto() {

    //Choose paths and combine multiple as necessary
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("MoveFromTarmac.wpilib.json")];
          
    // Reset odometry to the starting pose of the trajectory, then Run path following command, 
    // then stop at the end.
    return new PathweaverCommand(pathA,drivetrain)
                          .beforeStarting(() -> drivetrain.resetOdometry(pathA.getInitialPose()))
                          .beforeStarting(() -> drivetrain.setHeading(130))  //Define start heading (0 = driver facing forward)
                          .andThen(() -> drivetrain.tankDriveVolts(0, 0))
                          .andThen(new TurnToHeading(drivetrain, 130));  //Specify desired end heading
  }
}