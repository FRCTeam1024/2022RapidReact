// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.Arrays;
import java.io.IOException;
import java.io.File;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  //Create a list of trajectory JSONs and corresponding list of Trajectories
  private static File deployfile = Filesystem.getDeployDirectory();
  private static File pathfile = new File(deployfile, "paths/");
  static String fileList[] = pathfile.list();
  static Trajectory pathList[] = new Trajectory[fileList.length];

  private RobotContainer m_robotContainer;

  private final DigitalInput compBotJumper = new DigitalInput(9);
  private static boolean compBotState;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //Display and log the name and version of the code that is running
    System.out.println("Running "+BuildConfig.APP_NAME+" "+BuildConfig.APP_VERSION);

    // Check whether the current robot is the competition robot or the practice robot:
    if(compBotJumper.get() == false) {
      compBotState = false;
    } else {
      compBotState = true;
    }

    if(pathfile.isDirectory()){
      System.out.println("Is a directory");
      int index = Arrays.asList(Robot.fileList).indexOf("BlueFirstCargo.wpilib.json");
      System.out.println("index number: " + index);
    }


    // Load any path(s) needed for autonomous command from filesystem
    for(int i = 0; i < fileList.length; i++) {
      try {
        Path thePath = Filesystem.getDeployDirectory().toPath().resolve("paths/"+fileList[i]);
        pathList[i] = TrajectoryUtil.fromPathweaverJson(thePath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + fileList[i], ex.getStackTrace());
      }
    }


    // Instantiate our RobotContainer.  This will perform all our button bindings, and the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //Call this to diable any PIDSubsytems to avoid integral windup.
    m_robotContainer.disablePIDSubsystems();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  // Returns true if the current robot is the competition robot. Otherwise, false for practice bot.
  public static boolean isCompBot() {
    return compBotState;
  }
}
