
package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;

public class FollowTrajectoryCommand extends SequentialCommandGroup {

    public FollowTrajectoryCommand(DrivetrainSubsystem drivetrain, String pathName, HashMap<String, Command> eventMap,
            double maxVelocity, double maxAcceleration, boolean isFirstPath) {

        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, new PathConstraints(1.5, 1));
        Command swerveCommand = new PPSwerveControllerCommand(
                path,
                drivetrain::getPose, // Pose supplier
                drivetrain.getKinematics(), // SwerveDriveKinematics
                new PIDController(0.1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will
                                              // only use feedforwards.
                new PIDController(0.12, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(2, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0
                                            // will only use feedforwards.
                drivetrain::setModuleStates, // Module states consumer
                drivetrain // Requires the drive subsystem
        );
        FollowPathWithEvents command = new FollowPathWithEvents(
                swerveCommand,
                path.getMarkers(),
                eventMap);
                
        addCommands(
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true)),
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        drivetrain.getOdometry().resetPosition(drivetrain.getGyroscopeRotation(),
                                drivetrain.getSwerveModulePositions(), path.getInitialHolonomicPose());
                    }
                }),
                command,
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false))
        );
    }
}