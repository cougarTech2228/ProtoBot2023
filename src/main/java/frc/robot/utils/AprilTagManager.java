package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class AprilTagManager {

    static NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("AprilTag");

    static NetworkTableEntry m_pitchEntry = m_aprilTagTable.getEntry("Pitch");
    static NetworkTableEntry m_txEntry = m_aprilTagTable.getEntry("TX");
    static NetworkTableEntry m_tzEntry = m_aprilTagTable.getEntry("TZ");
    static NetworkTableEntry m_tagIdEntry = m_aprilTagTable.getEntry("Tag ID");

    public AprilTagManager() {
    }

    public double getPitch() {
        return m_pitchEntry.getDouble(Constants.BAD_APRIL_TAG_ID);
    }

    public double getTX() {
        return m_txEntry.getDouble(Constants.BAD_APRIL_TAG_ID);
    }

    public double getTZ() {
        return m_tzEntry.getDouble(Constants.BAD_APRIL_TAG_ID);
    }

    public double getTagID() {
        return m_tagIdEntry.getDouble(Constants.BAD_APRIL_TAG_ID);
    }
}
