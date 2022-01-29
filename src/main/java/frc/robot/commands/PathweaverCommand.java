package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class PathweaverCommand extends RamseteCommand {

    private Trajectory m_Path;
    private Drivetrain m_Drivetrain;

    public PathweaverCommand (Trajectory path, Drivetrain drivetrain) {

        super(
            path,
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

        m_Path = path;
        m_Drivetrain = drivetrain;


    }

    public SequentialCommandGroup configure() {
        return this.beforeStarting(() -> m_Drivetrain.resetOdometry(m_Path.getInitialPose()))
                    .andThen(() -> m_Drivetrain.tankDriveVolts(0, 0))
                    .andThen(new ParallelRaceGroup(
                                new TurnToHeading(m_Drivetrain, m_Path.sample(m_Path.getTotalTimeSeconds())
                                     .poseMeters.getRotation().getDegrees())),
                                new WaitCommand(4)); 
    }
    
}