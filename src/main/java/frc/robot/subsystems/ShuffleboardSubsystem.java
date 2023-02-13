package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShuffleboardSubsystem extends SubsystemBase {

    private static ShuffleboardTab m_autoConfigTab;
    private static ShuffleboardTab m_teleopConfigTab;

    private static SendableChooser<Constants.AutoPosition> m_positionChooser = new SendableChooser<>();
    private static SendableChooser<Constants.PlacePosition> m_preloadedPieceLevelChooser = new SendableChooser<>();
    private static SendableChooser<Constants.PlacePosition> m_stagedPieceLevelChooser = new SendableChooser<>();
    private static SendableChooser<Constants.ConeOffsetPosition> m_preloadedPieceConeOffsetChooser = new SendableChooser<>();
    private static SendableChooser<Constants.ConeOffsetPosition> m_stagedPieceConeOffsetChooser = new SendableChooser<>();

    private static GenericEntry m_detectedTagIdEntry;
    private static GenericEntry m_targetTagIdEntry;
    private static GenericEntry m_idsMatchEntry;

    public ShuffleboardSubsystem(ShuffleboardTab autoConfigTab, ShuffleboardTab teleopConfigTab) {

        m_autoConfigTab = autoConfigTab;
        m_teleopConfigTab = teleopConfigTab;

        m_detectedTagIdEntry = m_teleopConfigTab.add("Detected ID", Constants.BAD_APRIL_TAG_ID).getEntry();
        m_targetTagIdEntry = m_teleopConfigTab.add("Target ID", Constants.BAD_APRIL_TAG_ID).getEntry();
        m_idsMatchEntry = m_teleopConfigTab.add("IDs Match?", false).getEntry();
    }

    public void configureShuffleboard() {

        m_positionChooser.setDefaultOption("Position 3", Constants.AutoPosition.Position3);
        m_positionChooser.addOption("Position 2", Constants.AutoPosition.Position2);
        m_positionChooser.addOption("Position 1", Constants.AutoPosition.Position1);
        m_autoConfigTab.add("Auto: Position", m_positionChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withSize(3, 1)
                .withPosition(0, 0);

        m_preloadedPieceLevelChooser.setDefaultOption("High Cone", Constants.PlacePosition.HighCone);
        m_preloadedPieceLevelChooser.addOption("Middle Cone", Constants.PlacePosition.MiddleCone);
        m_preloadedPieceLevelChooser.addOption("Low Cone", Constants.PlacePosition.LowCone);
        m_preloadedPieceLevelChooser.addOption("High Cube", Constants.PlacePosition.HighCube);
        m_preloadedPieceLevelChooser.addOption("Middle Cube", Constants.PlacePosition.MiddleCube);
        m_preloadedPieceLevelChooser.addOption("Low Cube", Constants.PlacePosition.LowCube);
        m_autoConfigTab.add("Preloaded Piece Level", m_preloadedPieceLevelChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withSize(5, 1)
                .withPosition(0, 1);

        m_preloadedPieceConeOffsetChooser.setDefaultOption("Left", Constants.ConeOffsetPosition.Left);
        m_preloadedPieceConeOffsetChooser.addOption("Right", Constants.ConeOffsetPosition.Right);
        m_autoConfigTab.add("Preloaded Cone Offset", m_preloadedPieceConeOffsetChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withSize(2, 1)
                .withPosition(5, 1);

        m_stagedPieceLevelChooser.setDefaultOption("High Cone", Constants.PlacePosition.HighCone);
        m_stagedPieceLevelChooser.addOption("Middle Cone", Constants.PlacePosition.MiddleCone);
        m_stagedPieceLevelChooser.addOption("Low Cone", Constants.PlacePosition.LowCone);
        m_stagedPieceLevelChooser.addOption("High Cube", Constants.PlacePosition.HighCube);
        m_stagedPieceLevelChooser.addOption("Middle Cube", Constants.PlacePosition.MiddleCube);
        m_stagedPieceLevelChooser.addOption("Low Cube", Constants.PlacePosition.LowCube);
        m_autoConfigTab.add("Staged Piece Level", m_stagedPieceLevelChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withSize(5, 1)
                .withPosition(0, 2);

        m_stagedPieceConeOffsetChooser.setDefaultOption("Left", Constants.ConeOffsetPosition.Left);
        m_stagedPieceConeOffsetChooser.addOption("Right", Constants.ConeOffsetPosition.Right);
        m_autoConfigTab.add("Staged Cone Offset", m_stagedPieceConeOffsetChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withSize(2, 1)
                .withPosition(5, 2);
    }

    public Constants.AutoPosition getAutoPosition() {
        return m_positionChooser.getSelected();
    }

    public Constants.PlacePosition getPreloadedPieceLevel() {
        return m_preloadedPieceLevelChooser.getSelected();
    }

    public Constants.ConeOffsetPosition getPreloadedConeOffsetPosition() {
        return m_preloadedPieceConeOffsetChooser.getSelected();
    }

    public Constants.PlacePosition getStagedPieceLevel() {
        return m_stagedPieceLevelChooser.getSelected();
    }

    public Constants.ConeOffsetPosition getStagedConeOffsetPosition() {
        return m_stagedPieceConeOffsetChooser.getSelected();
    }

    public GenericEntry getDetectedIDEntry() {
        return m_detectedTagIdEntry;
    }

    public GenericEntry getTargetIDEntry() {
        return m_targetTagIdEntry;
    }

    @Override
    public void periodic() {
        double targetAprilTagID = RobotContainer.getButtonBoardSubsystem().getAprilTagID();

        m_targetTagIdEntry.setDouble(targetAprilTagID);
        m_detectedTagIdEntry.setDouble(RobotContainer.getAprilTagManager().getTagID());

        m_idsMatchEntry.setBoolean(RobotContainer.getButtonBoardSubsystem().isAprilTagIDMatch());
    }
}