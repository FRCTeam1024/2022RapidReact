// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.oi.Logitech;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final ByteAPult byteAPult = new ByteAPult();
  private final Limelight limelight = new Limelight();
  private final Intake intake = new Intake();
  private final Hanger hanger = new Hanger();

  // Operator Inputs
  private final Logitech driverController = new Logitech(Constants.Inputs.driverControllerID);
  private final Logitech operatorController = new Logitech(Constants.Inputs.operatorControllerID);

  // Default Commands
  private final DriveWithController driveWithController = new DriveWithController(drivetrain, driverController, 1);
  //private final DriveWithControllerPID driveWithPID = new DriveWithControllerPID(drivetrain, driverController);
  private final LoadByteAPult loadByteAPult = new LoadByteAPult(byteAPult);

  //Create a chooser for auto
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the dashboard
    configureDashboard();
    // Assign default Commands
    drivetrain.setDefaultCommand(driveWithController);
    byteAPult.setDefaultCommand(loadByteAPult);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * List any PID subsystems here so that they get disabled when the robot
   * is disabled and integral error doesn't accumulate.  Usually doesn't matter
   * since we often don't use integral gain but just in case.
   */
  public void disablePIDSubsystems() {
    hanger.disable();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**
     * Driver controls
     */

    // Limelight Pipelines
    driverController.leftTrigger.whenHeld(
      new SequentialCommandGroup(
        new InstantCommand(limelight::setTargetPipe),
        new LimelightAutoAim(limelight, drivetrain)
      )
    );


    //Turbo Mode
    driverController.rightBumper.whileHeld(new DriveWithController(drivetrain, driverController, 2));
    //Slow mode
    driverController.rightTrigger.whileHeld(new DriveWithController(drivetrain, driverController, 0));

    //Reset carriage encoders
    driverController.bButton.whileHeld(new InstantCommand(hanger::lowerToBottom, hanger));
    driverController.bButton.whenReleased(new InstantCommand(hanger::resetCarriage, hanger));

    //Move Hanger Carriage Manually, stop when buttons released
    driverController.dPadUp.whenPressed(
      new InstantCommand(() -> hanger.moveCarriage(HangerConstants.maxTravelMeters),hanger),false);
    driverController.dPadUp.whenReleased(
      new InstantCommand(() -> hanger.disable(),hanger),false);
    driverController.dPadDown.whenPressed(
      new InstantCommand(() -> hanger.moveCarriage(HangerConstants.minTravelMeters), hanger),false);
    driverController.dPadDown.whenReleased(
      new InstantCommand(() -> hanger.disable(),hanger),false);

    driverController.leftBumper.whenPressed(
      new InstantCommand(() -> hanger.moveCarriage(HangerConstants.maxTravelMeters),hanger),false);
    driverController.leftBumper.whenReleased(
      new InstantCommand(() -> hanger.disable(),hanger),false);

    //Move the monkey arm
    driverController.aButton.whenPressed(
      new InstantCommand(hanger::retractHook, hanger)
    );

    driverController.yButton.whenPressed(
      new InstantCommand(hanger::extendHook, hanger)
    );

    driverController.xButton.whileHeld(
      new InstantCommand(hanger::openPowerHook, hanger)
    );
    driverController.xButton.whenReleased(
      new InstantCommand(hanger::closePowerHook, hanger)
    );
    
 
    
    /**
     * Operator controls
     */

    //Deploy Intake to Collect
    operatorController.rightTrigger.whenPressed(
      new InstantCommand(intake::deploy,intake),false);
    operatorController.rightTrigger.whenReleased(
      new InstantCommand(intake::stow,intake),false);

    //Opens Gate while held
    operatorController.bButton.whenPressed(
      new InstantCommand(byteAPult::openGate, byteAPult),false);
    operatorController.bButton.whenReleased(
      new InstantCommand(byteAPult::closeGate, byteAPult),false);

    //Deploy intake to Eject
    operatorController.yButton.whileHeld(
      new InstantCommand(intake::eject,intake));
    operatorController.yButton.whenReleased(
      new InstantCommand(intake::stow, intake));

    //Reverse ballFeed
    operatorController.xButton.whileHeld(
      new InstantCommand(byteAPult::reverseGate, byteAPult));
    operatorController.xButton.whenReleased(
      new InstantCommand(byteAPult::closeGate, byteAPult));
      //Test this button, if it doesn't work, comment it out.


    //Near Shot in High Hub
    operatorController.leftTrigger.whenPressed(
      new SequentialCommandGroup(
          new InstantCommand(byteAPult::setNear,byteAPult),
          new InstantCommand(() -> byteAPult.launch(2,.25,65.0,false), byteAPult)),
        false);  

    //Far Shot in High Hub
    operatorController.leftBumper.whenPressed(
      new SequentialCommandGroup(
          new InstantCommand(byteAPult::setFar,byteAPult),
          new WaitCommand(0.5),
          new InstantCommand(() -> byteAPult.launch(2,.25,65.0,false), byteAPult),
          new WaitCommand(0.2),
          new InstantCommand(byteAPult::setNear,byteAPult)),
        false);
        
    //Launch near shot in low hub
    operatorController.aButton.whenPressed(
      new SequentialCommandGroup(
          new InstantCommand(byteAPult::setNear,byteAPult),
          new InstantCommand(() -> byteAPult.launch(1,.25,40.0,false), byteAPult)),
        false);  

    //Move launch pivot to near shot position
    operatorController.dPadUp.whenPressed(
      new InstantCommand(byteAPult::setNear,byteAPult));

    //Move launch pivot to far shot position
    operatorController.dPadDown.whenPressed(
      new InstantCommand(byteAPult::setFar,byteAPult));
  
  }

  /**
   * Use this method to configure the dashboard
   * 
   */
  private void configureDashboard() {
    //Create ShuffleBoard Tabs
    ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("1024Diagnostics");
    ShuffleboardTab driverTab = Shuffleboard.getTab("1024Driver");
    
    /**
     * Diagnostics for programmers
     */
    //Add command status to dashboard
    diagnosticsTab.add("DrivetrainCommand",drivetrain)
       .withSize(2,1)
       .withPosition(8,0);

    diagnosticsTab.add("HangerCommand", hanger)
       .withSize(2,1)
       .withPosition(8,1);
    
    diagnosticsTab.add("IntakeCommand", intake)
      .withSize(2,1)
      .withPosition(8,2);
    
    diagnosticsTab.add("Byte-A-PultCommand", byteAPult)
      .withSize(2,1)
      .withPosition(8,3);
    
    diagnosticsTab.add("LimelightCommand", limelight)
      .withSize(2,1)
      .withPosition(8,4);

    diagnosticsTab.add("TurnToHeading", new TurnToHeading(drivetrain, 90))
        .withSize(3,1)
        .withPosition(0,4);

    diagnosticsTab.addNumber("RobotHeading", drivetrain::getHeading)
        .withSize(2,1)
        .withPosition(0,0);

    diagnosticsTab.addNumber("LeftMotors", drivetrain::getLeftWheelSpeed)
        .withSize(2,1)
        .withPosition(2,0);

    diagnosticsTab.addNumber("RightMotors", drivetrain::getRightWheelSpeed)
        .withSize(2,1)
        .withPosition(4,0);

    diagnosticsTab.addBoolean("ReadytoLaunch", () -> byteAPult.readyToLaunch(60.0)) //I'm hoping this lambda thing works like this
        .withSize(1,1)
        .withPosition(4,1);

    diagnosticsTab.addNumber("Pressure", byteAPult::getPressure)
        .withSize(2,1)
        .withPosition(2,3);

    diagnosticsTab.addBoolean("BottomLimit", hanger::atBottomLimit)
        .withSize(1,1)
        .withPosition(0,3);

    diagnosticsTab.addBoolean("TopLimit", hanger::atTopLimit)
        .withSize(1,1)
        .withPosition(0,2);

    diagnosticsTab.addNumber("HangerPosition", hanger::getMeasurement)
        .withSize(2,1)
        .withPosition(0,1);

    diagnosticsTab.addBoolean("CargoLoaded", byteAPult::cargoPresent)
        .withSize(2,1)
        .withPosition(2,1);
    
    diagnosticsTab.addBoolean("Loaded1", byteAPult::returnLoaded1)
        .withSize(1,1)
        .withPosition(2,2);

    diagnosticsTab.addBoolean("Loaded2", byteAPult::returnLoaded2)
        .withSize(1,1)
        .withPosition(3,2);

    diagnosticsTab.addBoolean("ArmSensor", byteAPult::armRetracted)
        .withSize(1,1)
        .withPosition(4,2);
      
    diagnosticsTab.addNumber("LL Pipeline", limelight::getPipeline)
        .withSize(2, 1)
        .withPosition(4,3);

    diagnosticsTab.addBoolean("hangMode", hanger::getHangMode)
        .withSize(1,1)
        .withPosition(1,2);

    /**
     * Driver's operator interface
     */

    //Display the name and version number of the code.
    driverTab.add("Running Code Version:", BuildConfig.APP_NAME + " " + BuildConfig.APP_VERSION)
        .withSize(4,1)
        .withPosition(0,0);

    //Add commands to auto chooser, set default to null to avoid surprise operation
    m_AutoChooser.setDefaultOption("None", null);   
    m_AutoChooser.addOption("Extended 3.5 Ball Auto", getExtendedAuto());
    m_AutoChooser.addOption("Extended Auto - Testing Mode", getTestingExtendedAuto());
    m_AutoChooser.addOption("Shoot Move Shoot", getShootMoveShoot());
    m_AutoChooser.addOption("Shoot and Taxi", getShootAndTaxi());
    m_AutoChooser.addOption("Far Ball Run - might not be enough space in shop", getFarBall());
    //m_AutoChooser.addOption("Blue 3 Ball Auto", getBlueThreeBallAuto());
    //m_AutoChooser.addOption("Basic Forward", getBasicForwardAuto());
    //m_AutoChooser.addOption("Test", getTestAuto());

    //Put the auto chooser on the dashboard
    driverTab.add("Auto Mode",m_AutoChooser)
       .withSize(3,1)
       .withPosition(4,0);

    // Display the limelight's stream feed for the driver.
    driverTab.add("Limelight", limelight.getFeed())
        .withSize(7,6)
        //.withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(0, 1);

    // Display the intake's webcam feed for the driver.
    // Alex: commented out for now as RoboRio was having issues with "too many simultaneous client streams"
    driverTab.add("Intake", intake.getFeed())
        .withSize(6,6)
        .withPosition(7, 1);
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
   * Create an auto command using path(s) imported from pathweaver
   * Decorate with additional functions
   * Make copies of this method to generate alternate auto routines
   * 
   * @return the auto command
   */
  private Command getBlueThreeBallAuto() {

    //Choose paths and combine multiple as necessary
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points0-1.wpilib.json")];
    Trajectory pathB = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points1-2.wpilib.json")];
    Trajectory pathC = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points2-3.wpilib.json")];

    /**
    return new SequentialCommandGroup(
      //shoot first cargo
      new InstantCommand(byteAPult::setNear,byteAPult),
      new WaitCommand(0.5),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,true), byteAPult),
      //run first segment of path
      new PathweaverCommand(pathA, drivetrain).configure(),
      //deploy intake - may be moved to earlier because of intake interfering with catapult?
      new InstantCommand(intake::deploy, intake),
      //run segments 2 & 3 of path
      new PathweaverCommand(pathB, drivetrain).configure(),
      new PathweaverCommand(pathC, drivetrain).configure(),
      //shoot cargo 2 & 3
      new InstantCommand(byteAPult::setNear,byteAPult),
      new WaitCommand(0.5),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,true), byteAPult),
      new InstantCommand(byteAPult::setNear,byteAPult),
      new WaitCommand(0.5),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,true), byteAPult));
      */


    // Reset odometry to the starting pose of the trajectory, then Run path following command, 
    // then stop at the end.
    return new PathweaverCommand(pathA,drivetrain).configure();
  }

  private Command getTestingExtendedAuto() {

    //Choose paths and combine multiple as necessary
    Trajectory pathSetup = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points0-5.wpilib.json")];
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points5-6.wpilib.json")];
    Trajectory pathB = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points6-5.wpilib.json")];
    Trajectory pathC = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points5-7.wpilib.json")];
    Trajectory pathD = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points7-5.wpilib.json")];
    Trajectory pathE = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points5-7.wpilib.json")]
                        .concatenate(Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points7-8.wpilib.json")]);

    //Test routine to shoot the preloaded cargo and then run the autonomous path.
    return new SequentialCommandGroup(
      //lineup for first shot
      new PathweaverCommand(pathSetup, drivetrain).configure());
      //shoot cargo 1
      //new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      //pick up cargo 2 and return
      /**new InstantCommand(intake::deploy, intake),
      new InstantCommand(byteAPult::openGate, byteAPult),
      new PathweaverCommand(pathA, drivetrain).configure(),
      new PathweaverCommand(pathB, drivetrain).configure());
      //shoot cargo 2
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      //grab cargo 3 and return
      new PathweaverCommand(pathC, drivetrain).configure(),
      new PathweaverCommand(pathD, drivetrain).configure(),
      //shoot cargo 3
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      //stow intake before teleop
      new InstantCommand(intake::stow, intake),
      new InstantCommand(byteAPult::closeGate, byteAPult),
      //run back towards terminal
      new PathweaverCommand(pathE, drivetrain));**/

      

    // Reset odometry to the starting pose of the trajectory, then Run path following command, 
    // then stop at the end.
    //return new PathweaverCommand(pathA,drivetrain).configure();
  }

  /**
   * Create an auto command using path(s) imported from pathweaver
   * Decorate with additional functions
   * Make copies of this method to generate alternate auto routines
   * 
   * @return the auto command
   */
  private Command getExtendedAuto() {

    //Choose paths and combine multiple as necessary
    Trajectory pathSetup = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points0-5.wpilib.json")];
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points5-6.wpilib.json")];
    Trajectory pathB = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points6-5.wpilib.json")];
    Trajectory pathC = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points5-7.wpilib.json")];
    Trajectory pathD = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points7-5.wpilib.json")];
    Trajectory pathE = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points5-7.wpilib.json")]
                        .concatenate(Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points7-8.wpilib.json")]);

    //Test routine to shoot the preloaded cargo and then run the autonomous path.
    return new SequentialCommandGroup(
      //lineup for first shot
      new PathweaverCommand(pathSetup, drivetrain).configure(),
      //shoot cargo 1
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      //pick up cargo 2 and return
      new InstantCommand(intake::deploy, intake),
      new InstantCommand(byteAPult::openGate, byteAPult),
      new PathweaverCommand(pathA, drivetrain).configure(),
      new PathweaverCommand(pathB, drivetrain).configure(),
      //shoot cargo 2
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      //grab cargo 3 and return
      new PathweaverCommand(pathC, drivetrain).configure(),
      new PathweaverCommand(pathD, drivetrain).configure(),
      //shoot cargo 3
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      //stow intake before teleop
      new InstantCommand(intake::stow, intake),
      new InstantCommand(byteAPult::closeGate, byteAPult),
      //run back towards terminal
      new PathweaverCommand(pathE, drivetrain));

      

    // Reset odometry to the starting pose of the trajectory, then Run path following command, 
    // then stop at the end.
    //return new PathweaverCommand(pathA,drivetrain).configure();
  }

  private Command getShootMoveShoot() {

    Trajectory pathSetup = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points10-11.wpilib.json")];
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points11-12.wpilib.json")];
    Trajectory pathB = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points12-11.wpilib.json")];

    return new SequentialCommandGroup(
      new PathweaverCommand(pathSetup, drivetrain).configure(),
      //shoot cargo 1
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      //pick up cargo 2 and return
      new InstantCommand(intake::deploy, intake),
      new InstantCommand(byteAPult::openGate, byteAPult),
      new PathweaverCommand(pathA, drivetrain).configure(),
      new PathweaverCommand(pathB, drivetrain).configure(),
      //shoot cargo 2
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new InstantCommand(intake::stow, intake),
      new InstantCommand(byteAPult::closeGate, byteAPult));

  }

  private Command getFarBall(){
  
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("MoveFromTarmac.wpilib.json")];
    Trajectory pathB = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points13-14.wpilib.json")];
    Trajectory pathC = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points14-15.wpilib.json")];
    Trajectory pathD = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points15-16.wpilib.json")];
    Trajectory pathE = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points16-13.wpilib.json")];

    return new SequentialCommandGroup(
      new PathweaverCommand(pathA, drivetrain).configure(),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new PathweaverCommand(pathB, drivetrain).configure(),
      new PathweaverCommand(pathC, drivetrain).configure(),
      new InstantCommand(intake::deploy, intake),
      new InstantCommand(byteAPult::openGate, byteAPult),
      new PathweaverCommand(pathD, drivetrain).configure(),
      new InstantCommand(intake::stow, intake),
      new InstantCommand(byteAPult::closeGate, byteAPult),
      new PathweaverCommand(pathE, drivetrain).configure(),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult)
      
    );
  
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
    return new PathweaverCommand(pathA,drivetrain).configure();
  }

     /**
   * Create an auto command using path(s) imported from pathweaver
   * Decorate with additional functions
   * Make copies of this method to generate alternate auto routines
   * 
   * @return the auto command
   */
  private Command getTestAuto() {

    //Choose paths and combine multiple as necessary
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Testing.wpilib.json")];
          
    // Reset odometry to the starting pose of the trajectory, then Run path following command, 
    // then stop at the end.
    return new PathweaverCommand(pathA,drivetrain).configure();
  } 

  private Command getShootAndTaxi(){
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("MoveFromTarmac.wpilib.json")];

    return new SequentialCommandGroup(
      new PathweaverCommand(pathA, drivetrain).configure(),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.2),
      new PathweaverCommand(pathA, drivetrain).configure()
    );
  }
}