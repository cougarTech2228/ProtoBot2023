package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class OutPathFileNameChooser {

    private Constants.AutoPosition m_autoPosition;
    private Constants.PlacePosition m_preloadedPieceLevel;
    private Constants.ConeOffsetPosition m_coneOffsetPosition;
    private String m_outPathFileName;

    public OutPathFileNameChooser() {

        m_autoPosition =  RobotContainer.getShuffleboardSubsystem().getAutoPosition();
        m_preloadedPieceLevel = RobotContainer.getShuffleboardSubsystem().getPreloadedPieceLevel();
        m_coneOffsetPosition = RobotContainer.getShuffleboardSubsystem().getPreloadedConeOffsetPosition();

        m_outPathFileName = null;
    }

    public String getOutPathFileName() {

        if ((m_preloadedPieceLevel == Constants.PlacePosition.HighCone) ||
                (m_preloadedPieceLevel == Constants.PlacePosition.MiddleCone) ||
                (m_preloadedPieceLevel == Constants.PlacePosition.LowCone)) {

            if (m_coneOffsetPosition == Constants.ConeOffsetPosition.Left) {
                if (m_autoPosition == Constants.AutoPosition.Position1) {
                    m_outPathFileName = "auto1L_out";
                } else if (m_autoPosition == Constants.AutoPosition.Position2) {
                    m_outPathFileName = "auto2L_out";
                } else if (m_autoPosition == Constants.AutoPosition.Position3) {
                    m_outPathFileName = "auto3L_out";
                } else {
                    System.out.println("Uh Oh, somthing went wrong :( (Auto Location not found)");
                }

            } else { // Cone Right
                if (m_autoPosition == Constants.AutoPosition.Position1) {
                    m_outPathFileName = "auto1R_out";
                } else if (m_autoPosition == Constants.AutoPosition.Position2) {
                    m_outPathFileName = "auto2R_out";
                } else if (m_autoPosition == Constants.AutoPosition.Position3) {
                    m_outPathFileName = "auto3R_out";
                } else {
                    System.out.println("Uh Oh, somthing went wrong :( (Auto Location not found)");
                }
            }
        } else { // Cube
            if (m_autoPosition == Constants.AutoPosition.Position1) {
                m_outPathFileName = "autoM_out";
            } else if (m_autoPosition == Constants.AutoPosition.Position2) {
                m_outPathFileName = "auto2M_out";
            } else if (m_autoPosition == Constants.AutoPosition.Position3) {
                m_outPathFileName = "auto3M_out";
            } else {
                System.out.println("Uh Oh, somthing went wrong :( (Auto Location not found)");
            }
        }

        System.out.println("Choosing out path file: " + m_outPathFileName);

        return m_outPathFileName;
    }
}