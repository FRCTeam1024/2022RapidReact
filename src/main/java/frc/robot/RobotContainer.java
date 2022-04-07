// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;


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
  private final StowIntake stowIntake = new StowIntake(intake);

  //Create a chooser for auto
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the dashboard
    configureDashboard();
    // Assign default Commands
    drivetrain.setDefaultCommand(driveWithController);
    byteAPult.setDefaultCommand(loadByteAPult);
    intake.setDefaultCommand(stowIntake);

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
        new InstantCommand(limelight::setTargetPipe, limelight),
        new WaitUntilCommand(limelight::hasTarget),
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

    driverController.xButton.whenPressed(hanger::togglePowerHook, hanger);


    //Code that ideally should automate our climbing process. - TEST ONE LINE AT A TIME TO SEE WHAT HAPPENS
    /*

    DP:  Why not make the whole SequentialCommandGroup part of a ConditionalCommand, then
    ConditionalCommand doesn't need to be repeated every line?  

    I think it is going to be hard to achieve a fully automated climb as we don't
    have enough feedback devices to check ourselves.  The driver will probably have to take over
    at some point most of the time anyway.  
    
    I would write this in a way that requires the driver to hit a button after each command 
    to indicate that the sequence should procede.  This way the driver has as much time as 
    needed to confirm the previous action was successful. You can then remove any WaitCommands.

    Short of a fully automated sequence, there are also sub-automations that could be helpful 
    to reduce the complexity of the manual climb
    sequence such as:
    -auto moving the power hook to a preset height
    -auto deploying the monkey hook when reaching the bottom limit
    -auto switch drivetrain to creep speed to aid with aligment after power hook is lifted.


    driverController.bButton.whenPressed(
      new SequentialCommandGroup(
        new ConditionalCommand(new RunCommand(() -> hanger.moveCarriage(HangerConstants.minTravelMeters),hanger).withInterrupt(hanger::atBottomLimit), null, hanger::getHangMode),
        new ConditionalCommand(new InstantCommand(hanger::extendHook, hanger), null, hanger::getHangMode),
        new ConditionalCommand(new WaitCommand(1), null, hanger::getHangMode),
        new ConditionalCommand(new InstantCommand(hanger::togglePowerHook, hanger), null, hanger::getHangMode),
        new ConditionalCommand(new WaitCommand(2), null, hanger::getHangMode),
        new ConditionalCommand(new RunCommand(() -> hanger.moveCarriage(HangerConstants.maxTravelMeters),hanger).withInterrupt(hanger::atTopLimit), null, hanger::getHangMode),
        new ConditionalCommand(new InstantCommand(hanger::togglePowerHook, hanger), null, hanger::getHangMode),
        new ConditionalCommand(new WaitCommand(1), null, hanger::getHangMode),
        new ConditionalCommand(new RunCommand(() -> hanger.moveCarriage(HangerConstants.minTravelMeters),hanger).withInterrupt(hanger::atBottomLimit), null, hanger::getHangMode),
        new ConditionalCommand(new WaitCommand(2), null, hanger::getHangMode),
        new ConditionalCommand(new InstantCommand(hanger::retractHook, hanger), null, hanger::getHangMode),
        new ConditionalCommand(new WaitCommand(1), null, hanger::getHangMode),
        new ConditionalCommand(new InstantCommand(hanger::togglePowerHook, hanger), null, hanger::getHangMode),
      )
    );

    driverController.leftBumper.whenPressed(
      new InstantCommand(hanger::toggleHangMode, hanger)
    );
    */
     
    
    
    
    /**
     * Operator controls
     */

    //Deploy Intake to Collect
    operatorController.rightTrigger.whileHeld(
      new InstantCommand(intake::deploy,intake),true);
    operatorController.rightTrigger.whenReleased(
      new SequentialCommandGroup(
        new InstantCommand(intake::stow, intake),
        new InstantCommand(byteAPult::reverseGate,byteAPult),//Need to require the byteAPult so that the commandgroup will override the default command
        new WaitCommand(0.2),
        new InstantCommand(byteAPult::closeGate,byteAPult)
      ),false
    );

    //Opens Gate while held and runs the shifter. but only if ReadyToLoad is true
    operatorController.bButton.whileHeld(
      new ParallelCommandGroup(new InstantCommand(byteAPult::openGate, byteAPult),
                               new InstantCommand(() -> intake.runShifter(IntakeConstants.kShifterSpeed),intake)
                              ));

    operatorController.bButton.whenReleased(
      new ParallelCommandGroup(new InstantCommand(byteAPult::closeGate, byteAPult),
                               new InstantCommand(() -> intake.runShifter(0),intake)));

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


    //Near Shot in High Hub
    operatorController.leftTrigger.whenPressed(
      new SequentialCommandGroup(
          new InstantCommand(byteAPult::setNear,byteAPult),
          new InstantCommand(() -> byteAPult.launch(2,.25,65.0,false), byteAPult),
          new WaitUntilCommand(byteAPult::armNotRetracted).withTimeout(1),
          ReloadCommand()
      ),
    false);  

    //Far Shot in High Hub
    operatorController.leftBumper.whenPressed(
      new SequentialCommandGroup(
          new InstantCommand(byteAPult::setFar,byteAPult),
          new WaitCommand(0.5),
          new InstantCommand(() -> byteAPult.launch(2,.25,65.0,false), byteAPult),
          new WaitCommand(0.3),
          new InstantCommand(byteAPult::setNear,byteAPult),
          ReloadCommand()
      ),
    false);
        
    //Launch near shot in low hub
    operatorController.aButton.whenPressed(
      new SequentialCommandGroup(
          new InstantCommand(byteAPult::setNear,byteAPult),
          new InstantCommand(() -> byteAPult.launch(1,.05,40,false), byteAPult),
          new WaitUntilCommand(byteAPult::armNotRetracted).withTimeout(1),
          ReloadCommand()
      ),
    false);  

    operatorController.rightBumper.whenPressed(
      new SequentialCommandGroup(
          new InstantCommand(byteAPult::setNear,byteAPult),
          new InstantCommand(() -> byteAPult.launch(1,.12,40,false), byteAPult),
          new WaitUntilCommand(byteAPult::armNotRetracted).withTimeout(1),
          ReloadCommand()
      ),
    false);

    //Move launch pivot to near shot position
    operatorController.dPadUp.whenPressed(
      new InstantCommand(byteAPult::setNear,byteAPult));

    //Move launch pivot to far shot position
    operatorController.dPadDown.whenPressed(
      new InstantCommand(byteAPult::setFar,byteAPult));
  
  }

  // A Utility command to reload the ByteAPult after any launch sequence
  private Command ReloadCommand() {
    return(new SequentialCommandGroup(
              new WaitCommand(0.2),
              new WaitUntilCommand(byteAPult::readyToLoad).withTimeout(1),
              new ConditionalCommand(new ParallelCommandGroup(new InstantCommand(byteAPult::openGate, byteAPult),
                                                              new InstantCommand(() -> intake.runShifter(IntakeConstants.kShifterSpeed),intake)),
                                    new ParallelCommandGroup(new InstantCommand(byteAPult::closeGate, byteAPult),
                                                              new InstantCommand(() -> intake.runShifter(0),intake)),
                                     byteAPult::readyToLoad),
              new WaitUntilCommand(byteAPult::cargoPresent).withTimeout(0.5),
              new ParallelCommandGroup(new InstantCommand(byteAPult::reverseGate, byteAPult),
                                      new InstantCommand(() -> intake.runShifter(0),intake)
                                      ),
              new WaitCommand(0.2),
              new InstantCommand(byteAPult::closeGate, byteAPult)
          )
    );
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
        .withSize(2,1)
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

    diagnosticsTab.addBoolean("ReadytoLaunch", () -> byteAPult.readyToLaunch(60.0)) 
        .withSize(2,1)
        .withPosition(2,1);

    diagnosticsTab.addBoolean("ReadytoLoad", byteAPult::readyToLoad)
        .withSize(2,1)
        .withPosition(4,1);

    diagnosticsTab.addNumber("Pressure", byteAPult::getPressure)
        .withSize(2,1)
        .withPosition(2,4);

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
        .withPosition(2,2);
    
    diagnosticsTab.addBoolean("Loaded1", byteAPult::returnLoaded1)
        .withSize(1,1)
        .withPosition(2,3);

    diagnosticsTab.addBoolean("Loaded2", byteAPult::returnLoaded2)
        .withSize(1,1)
        .withPosition(3,3);

    diagnosticsTab.addBoolean("ArmSensor", byteAPult::armRetracted)
        .withSize(2,1)
        .withPosition(4,2);
    
    diagnosticsTab.addBoolean("Arm1", byteAPult::returnArm1)
        .withSize(1,1)
        .withPosition(4,3);
    
    diagnosticsTab.addBoolean("Arm2", byteAPult::returnArm2)
        .withSize(1,1)
        .withPosition(5,3);

    diagnosticsTab.addNumber("LL Pipeline", limelight::getPipeline)
        .withSize(2, 1)
        .withPosition(4,4);

    diagnosticsTab.addBoolean("hangMode", hanger::getHangMode)
        .withSize(1,1)
        .withPosition(1,2);

    /**
     * Driver's operator interface
     */

    //Display the name and version number of the code.
    driverTab.add("Running Code Version:", BuildConfig.APP_NAME + " " + BuildConfig.APP_VERSION)
        .withSize(3,1)
        .withPosition(0,0);

    //Add commands to auto chooser, set default to null to avoid surprise operation
    m_AutoChooser.setDefaultOption("None", null);   
    m_AutoChooser.addOption("Demigod Mode", getPartGodMode());
    m_AutoChooser.addOption("Extended 3.5 Ball Auto", getExtendedAuto());
    m_AutoChooser.addOption("Extended 3.5 Ball Auto - Low Shot", getExtendedAutoLowShot());
    m_AutoChooser.addOption("Extended Auto - Testing Mode", getTestingExtendedAuto());
    m_AutoChooser.addOption("Shoot Move Shoot", getShootMoveShoot());
    m_AutoChooser.addOption("Shoot and Taxi", getShootAndTaxi());
    m_AutoChooser.addOption("Far Ball Run - might not be enough space in shop", getFarBall());
    m_AutoChooser.addOption("4 Ball God Mode Auto", getParallelGodMode());
    m_AutoChooser.addOption("Restructued God Mode", getRestructuredGodMode());
    //m_AutoChooser.addOption("Blue 3 Ball Auto", getBlueThreeBallAuto());
    //m_AutoChooser.addOption("Basic Forward", getBasicForwardAuto());
    //m_AutoChooser.addOption("Test", getTestAuto());

    //Put the auto chooser on the dashboard
    driverTab.add("Auto Mode",m_AutoChooser)
       .withSize(3,1)
       .withPosition(3,0);

    driverTab.addNumber("Pressure", byteAPult::getPressure)
      .withSize(1,1)
      .withPosition(6,0);

    driverTab.addBoolean("Cargo Loaded",byteAPult::cargoPresent)
      .withSize(1,1)
      .withPosition(7,0);

    driverTab.addBoolean("At Top", hanger::atTopLimit)
      .withSize(1,1)
      .withPosition(9,0);

    driverTab.addBoolean("At Bottom", hanger::atBottomLimit)
      .withSize(1,1)
      .withPosition(8,0);

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
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),//comment out these 4 lines to remove power hook firing if need be.
      //lineup for first shot
      new PathweaverCommand(pathSetup, drivetrain).configure(),
      //shoot cargo 1
      new InstantCommand(() -> byteAPult.launch(2,.25,65.0,false), byteAPult));
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
    //Max Velocity: 2.0 m/s
    //Max Acceleration: 2.0 m/s^2

    //Choose paths and combine multiple as necessary
    Trajectory pathSetup = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points0-5.wpilib.json")];
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points5-6.wpilib.json")];
    Trajectory pathB = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points6-5.wpilib.json")];
    Trajectory pathC = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points5-7.wpilib.json")];
    Trajectory pathD = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points7-5.wpilib.json")];
    Trajectory pathE = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points5-8.wpilib.json")];

    //Test routine to shoot the preloaded cargo and then run the autonomous path.
    return new SequentialCommandGroup(
      //lineup for first shot
      /**new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),**/
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

  private Command getExtendedAutoLowShot() {

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
      new InstantCommand(() -> byteAPult.launch(1,.25,80.0,false), byteAPult));
      //pick up cargo 2 and return
      /**new InstantCommand(intake::deploy, intake),
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
      new PathweaverCommand(pathE, drivetrain));**/
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
  
  private Command getGodMode(){
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points17-18.wpilib.json")];
    Trajectory pathB = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points18-19.wpilib.json")];
    Trajectory pathC = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points 19-21.wpilib.json")];
    Trajectory pathD = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points21-23.wpilib.json")];
    Trajectory pathE = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points23-25.wpilib.json")];

    return new SequentialCommandGroup(
      
     /* new InstantCommand(byteAPult::closeGate, byteAPult),
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),
      */
      //intake floor cargo
      new InstantCommand(intake::deploy, intake),
      //moving to first floor cargo
      new PathweaverCommand(pathA, drivetrain).configure(),
      //move back to shoot
      new InstantCommand(intake::stow, intake),
      new PathweaverCommand(pathB, drivetrain).configure(),
      //shooting 2 cargos
      new InstantCommand(byteAPult::reverseGate, byteAPult),
      new WaitCommand(0.15),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.85), 
      new InstantCommand(() -> intake.runShifter(IntakeConstants.kShifterSpeed), intake),
      new InstantCommand(byteAPult::openGate, byteAPult),
      new WaitCommand(0.2),
      new InstantCommand(() -> intake.runShifter(0), intake),
      new InstantCommand(byteAPult::closeGate, byteAPult),
      new WaitCommand(0.5),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.2),
      //moving to get floor cargo
      new InstantCommand(intake::deploy, intake),
      new InstantCommand(byteAPult::openGate, byteAPult),
      new PathweaverCommand(pathC, drivetrain).configure(),
      //moving to get far crago
      new InstantCommand(byteAPult::closeGate, byteAPult),
      new PathweaverCommand(pathD, drivetrain).configure(),
      new InstantCommand(intake::stow, intake),
      //intake cargo
      //moving back to shoot
      new PathweaverCommand(pathE, drivetrain).configure(),
      //shooting both cargos
      new WaitCommand(0.2),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.2),
      new InstantCommand(() -> intake.runShifter(IntakeConstants.kShifterSpeed), intake),
      new InstantCommand(byteAPult::openGate, byteAPult),
      new WaitCommand(0.5),
      new InstantCommand(() -> intake.runShifter(0), intake),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult)
      );
  }

  private Command getParallelGodMode(){
    //Max Velocity: 3.0 m/s
    //Max Acceleration: 2.5m/s^2

    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points17-18.wpilib.json")];
    Trajectory pathB = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points18-19.wpilib.json")];
    //Trajectory pathC = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points 19-21.wpilib.json")]
                        //.concatenate(Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points21-23.wpilib.json")]);
    Trajectory pathC = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points19-23.wpilib.json")];
    //Trajectory pathD = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points21-23.wpilib.json")];
    Trajectory pathE = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points23-24.wpilib.json")]; //change to Points23-25 if need to run near shot

    return new SequentialCommandGroup(
      
     /* new InstantCommand(byteAPult::closeGate, byteAPult),
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),
      */
      //intake floor cargo
      new ParallelCommandGroup(
        new InstantCommand(intake::deploy, intake),
        new PathweaverCommand(pathA, drivetrain).configure()
      ),
      //move back to shoot
      new ParallelCommandGroup(
        new InstantCommand(intake::stow, intake),
        new PathweaverCommand(pathB, drivetrain).configure()
      ),
      //shooting 2 cargos
      new InstantCommand(byteAPult::reverseGate, byteAPult),
      new WaitCommand(0.15),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      //new WaitUntilCommand(byteAPult::armRetracted),
      new WaitCommand(0.85),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.runShifter(IntakeConstants.kShifterSpeed), intake),
        new InstantCommand(byteAPult::openGate, byteAPult)
      ),
      new WaitCommand(0.2),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.runShifter(0), intake),
        new InstantCommand(byteAPult::closeGate, byteAPult)
      ),
      new WaitCommand(0.6),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.2),
      //moving to get floor cargo
      /**new ParallelCommandGroup(
        new InstantCommand(intake::deploy, intake),
        new InstantCommand(byteAPult::openGate, byteAPult),
        new PathweaverCommand(pathC, drivetrain).configure()
      ),
      //moving to get far crago
      new ParallelCommandGroup(
        new InstantCommand(byteAPult::closeGate, byteAPult),
        new InstantCommand(byteAPult::setFar),
        new PathweaverCommand(pathD, drivetrain).configure()
      ),*/
      new ParallelCommandGroup(
        new PathweaverCommand(pathC, drivetrain).configure(),
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new InstantCommand(intake::deploy, intake),
            new InstantCommand(byteAPult::openGate, byteAPult)
          ),
          new WaitCommand(4),
          new ParallelCommandGroup(
            new InstantCommand(byteAPult::setFar, byteAPult),
            new InstantCommand(byteAPult::closeGate),
            new InstantCommand(() -> intake.runShifter(IntakeConstants.kShifterSpeed/2))
          )
        )
      ),
      //Remove setting near and far to make only near shots
      new ParallelCommandGroup(
        new InstantCommand(intake::stow, intake),
        new PathweaverCommand(pathE, drivetrain).configure()
        //Below lines should only be here if we start having issues with cargo running into each other
        /**new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new WaitCommand(0.2),
            new InstantCommand(byteAPult::reverseGate),
            new InstantCommand(() -> intake.runShifter(-IntakeConstants.kShifterSpeed))
          ),
          new ParallelCommandGroup(
            new InstantCommand(byteAPult::closeGate),
            new InstantCommand(() -> intake.runShifter(0))
          )
        )*/
      ),
      //shooting both cargos
      new WaitCommand(0.2),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.2),
      new InstantCommand(byteAPult::setNear),
      //new WaitUntilCommand(byteAPult::armRetracted),
      new WaitCommand(0.3),
      new InstantCommand(() -> intake.runShifter(IntakeConstants.kShifterSpeed), intake),
      new InstantCommand(byteAPult::openGate, byteAPult),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        new InstantCommand(byteAPult::setFar),
        new InstantCommand(() -> intake.runShifter(0), intake)
      ),
      new WaitCommand(0.2),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.1),
      new InstantCommand(byteAPult::setNear)
      );  
  }

  private Command getPartGodMode(){
    //Max Velocity: 3.0 m/s
    //Max Acceleration: 2.5m/s^2
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points17-18.wpilib.json")];
    Trajectory pathB = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points18-19.wpilib.json")];
    //                    .concatenate(Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points19-19.5.wpilib.json")]);  //remove for edited path
    //Trajectory pathShoot2 = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points19.5-19.wpilib.json")]; //remove for edited path
    Trajectory pathC = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points 19-21.wpilib.json")];
    Trajectory pathD = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points21-26.wpilib.json")];

    return new SequentialCommandGroup(
      
     /* new InstantCommand(byteAPult::closeGate, byteAPult),
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),
      */
      //intake floor cargo
      new ParallelCommandGroup(
        new InstantCommand(intake::deploy, intake),
        new PathweaverCommand(pathA, drivetrain).configure()
      ),
      //move back to shoot
      new ParallelCommandGroup(
        new InstantCommand(intake::stow, intake),
        new PathweaverCommand(pathB, drivetrain).configure()
      ),
      //shooting 2 cargos
      new InstantCommand(byteAPult::reverseGate, byteAPult),
      new WaitCommand(0.15),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      //new WaitUntilCommand(byteAPult::armRetracted),
      new WaitCommand(0.85),
      new ParallelCommandGroup(
        //new PathweaverCommand(pathShoot2, drivetrain).configure(), //remove for edited path
        new InstantCommand(() -> intake.runShifter(IntakeConstants.kShifterSpeed), intake),
        new InstantCommand(byteAPult::openGate, byteAPult)
      ),
      new WaitCommand(0.2),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.runShifter(0), intake),
        new InstantCommand(byteAPult::closeGate, byteAPult)
      ),
      new WaitCommand(0.75),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.2),
      new ParallelCommandGroup(
        new PathweaverCommand(pathC, drivetrain).configure(),
        new InstantCommand(intake::deploy, intake),
        new InstantCommand(byteAPult::openGate, byteAPult)
      ),
      //shooting both cargos
      new ParallelCommandGroup(
        new PathweaverCommand(pathD, drivetrain).configure(),
        new SequentialCommandGroup(
          new WaitCommand(0.1),
          new InstantCommand(byteAPult::closeGate),
          new InstantCommand(() -> intake.runShifter(0))
        )
      ),
      new WaitCommand(0.2),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult)
    );
  }

  private Command getRestructuredGodMode(){
    Trajectory pathA = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points17-18.wpilib.json")];
    Trajectory pathB = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points18-19.wpilib.json")];
    Trajectory pathC = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points 19-21.wpilib.json")];
    Trajectory pathD = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points21-26.wpilib.json")];
    Trajectory pathE = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points26-23.wpilib.json")]; 
    Trajectory pathF = Robot.pathList[Arrays.asList(Robot.fileList).indexOf("Points23-25.wpilib.json")];
    return new SequentialCommandGroup(
      
     /* new InstantCommand(byteAPult::closeGate, byteAPult),
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::openPowerHook, hanger),
      new WaitCommand(0.1),
      new InstantCommand(hanger::closePowerHook, hanger),
      */
      //intake floor cargo
      new ParallelCommandGroup(
        new InstantCommand(intake::deploy, intake),
        new PathweaverCommand(pathA, drivetrain).configure()
      ),
      //move back to shoot
      new ParallelCommandGroup(
        new InstantCommand(intake::stow, intake),
        new PathweaverCommand(pathB, drivetrain).configure()
      ),
      //shooting 2 cargos
      new InstantCommand(byteAPult::reverseGate, byteAPult),
      new WaitCommand(0.15),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      //new WaitUntilCommand(byteAPult::armRetracted),
      new WaitCommand(0.85),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.runShifter(IntakeConstants.kShifterSpeed), intake),
        new InstantCommand(byteAPult::openGate, byteAPult)
      ),
      new WaitCommand(0.2),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.runShifter(0), intake),
        new InstantCommand(byteAPult::closeGate, byteAPult)
      ),
      new WaitCommand(0.5),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.2),
      //moving to get floor cargo
      new ParallelCommandGroup(
        new InstantCommand(intake::deploy, intake),
        new InstantCommand(byteAPult::openGate, byteAPult),
        new PathweaverCommand(pathC, drivetrain).configure()
      ),
      new PathweaverCommand(pathD, drivetrain).configure(),
      new WaitCommand(0.5),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.2),
      new PathweaverCommand(pathE, drivetrain).configure(),
      new ParallelCommandGroup(
        new InstantCommand(intake::stow, intake),
        new InstantCommand(byteAPult::closeGate, byteAPult),
        new PathweaverCommand(pathF, drivetrain)
      ),
      new InstantCommand(() -> byteAPult.launch(2,.25,80.0,false), byteAPult),
      new WaitCommand(0.2)
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
