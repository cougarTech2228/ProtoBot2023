package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetArmReachCommand extends CommandBase {

    private double m_targetReachCm;

    private static final double MAXIMUM_REACH_CM = Constants.ARM_HIGH_CONE_REACH_CM;

    public SetArmReachCommand(double targetReachCm) {

        m_targetReachCm = targetReachCm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_targetReachCm > MAXIMUM_REACH_CM) {
            m_targetReachCm = MAXIMUM_REACH_CM;
        }

        System.out.println("Starting SetArmReachCommand. Target Reach(cm): " + m_targetReachCm);

        RobotContainer.getArmSubsystem().enableDistanceSensor(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.getArmSubsystem().enableDistanceSensor(false);
        RobotContainer.getArmSubsystem().stopMotor();
        System.out.println("Ending SetArmReachCommand");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return (RobotContainer.getArmSubsystem().atSetpoint());
        return true;
    }
}
