package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.OutPathFileNameChooser;
import frc.robot.utils.PlacePieceCommandChooser;

public class AutoTwoCommand extends SequentialCommandGroup {

    private double m_startTime = 0;

    public AutoTwoCommand() {

        // TODO - do we want to do something cool at each stage like with LEDs?
        // We could create multiple eventMaps
        HashMap<String, Command> m_eventMap = new HashMap<>();

        // Select the "out path" file based on Shuffleboard configuration
        OutPathFileNameChooser m_outPathFileNameChooser = new OutPathFileNameChooser();
        String m_outPathFileName = m_outPathFileNameChooser.getOutPathFileName();

        // Get the appropriate command group to place the Preloaded Game Piece
        PlacePieceCommandChooser m_placePreloadedPieceCommandChooser = new PlacePieceCommandChooser(
                RobotContainer.getShuffleboardSubsystem()
                        .getPreloadedPieceLevel());
        SequentialCommandGroup m_placePreloadedPieceSequentialCommandGroup = m_placePreloadedPieceCommandChooser
                .getPlacePieceCommand();

        addCommands(new InstantCommand(() -> printStartCommand()),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().zeroGyroscope()),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setMotorsToBrake),
                m_placePreloadedPieceSequentialCommandGroup,
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), m_outPathFileName,
                        m_eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto2_back",
                        m_eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                        new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().reverseGyroscope()),
                new InstantCommand(() -> printEndCommand()));
    }

    private void printStartCommand() {
        m_startTime = Timer.getFPGATimestamp();
        System.out.println("Starting AutoTwoCommand");
    }

    private void printEndCommand() {
        System.out.println("AutoTwoCommand completed in " + (Timer.getFPGATimestamp() - m_startTime) + " seconds");
    }
}
