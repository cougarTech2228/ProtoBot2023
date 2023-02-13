package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ButtonBoardSubsystem;
import frc.robot.utils.DockWithAprilTag;
import frc.robot.Constants;

public class DockWithAprilTagCommand extends CommandBase {
    private double m_aprilTagId;
    private boolean m_isFOV;
    private ButtonBoardSubsystem m_buttonBoardSubsystem;

    private Runnable m_dockWithAprilTagRunnable;
    private Thread m_dockWithAprilTagThread;

    public DockWithAprilTagCommand(
            boolean isFOV) {
        m_isFOV = isFOV;
    }

    public DockWithAprilTagCommand(
            boolean isFOV,
            ButtonBoardSubsystem buttonBoardSubsystem) {
        m_isFOV = isFOV;
        m_buttonBoardSubsystem = buttonBoardSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // If we're using this class from an Autonomous command, we will not
        // be sending in the ButtonBoardManager. If it's being used in Teleop,
        // then ButtonBoardManager is sent in and we can use that to get the
        // current target April Tag ID. This 'hack' is necessary to get the
        // button bindings in the ButtonBoardManager to work. Without this
        // approach, we were never getting the correct April Tag ID value
        // presumably due to how the button bindings are created at
        // initialization.
        if (m_buttonBoardSubsystem == null) {
            // Get the correct AprilTag ID based on Position and Alliance Color
            if (RobotContainer.getShuffleboardSubsystem().getAutoPosition() == Constants.AutoPosition.Position1) {
                if (DriverStation.getAlliance() == Alliance.Blue) {
                    m_aprilTagId = 6.0;
                } else {
                    m_aprilTagId = 1.0;
                }
            } else if (RobotContainer.getShuffleboardSubsystem().getAutoPosition() == Constants.AutoPosition.Position2) {
                if (DriverStation.getAlliance() == Alliance.Blue) {
                    m_aprilTagId = 7.0;
                } else {
                    m_aprilTagId = 2.0;
                }
            } else if (RobotContainer.getShuffleboardSubsystem().getAutoPosition() == Constants.AutoPosition.Position3) {
                if (DriverStation.getAlliance() == Alliance.Blue) {
                    m_aprilTagId = 8.0;
                } else {
                    m_aprilTagId = 3.0;
                }
            } else {
                System.out.println("Invalid position received from Shuffleboard");
            }
        } else {
            m_aprilTagId = m_buttonBoardSubsystem.getAprilTagID();
        }

        System.out.println("Running auto dock with AprilTag command for tag ID: " + m_aprilTagId);

        m_dockWithAprilTagRunnable = new DockWithAprilTag(
                m_isFOV,
                m_aprilTagId);

        m_dockWithAprilTagThread = new Thread(m_dockWithAprilTagRunnable, "DockWithAprilTagThread");
        m_dockWithAprilTagThread.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // do nothing ... code is running in a thread
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("ending auto dock with AprilTag command");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (!m_dockWithAprilTagThread.isAlive());
    }
}
