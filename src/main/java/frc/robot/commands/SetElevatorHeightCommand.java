package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetElevatorHeightCommand extends CommandBase {

    private double m_targetHeightCm;

    public SetElevatorHeightCommand(double targetHeightCm) {

        // TODO - probably should validate value of targetHeightCm
        m_targetHeightCm = targetHeightCm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting SetElevatorHeightCommand. Target Height(cm): " + m_targetHeightCm);
        RobotContainer.getElevatorSubsystem().updateSetpoint(m_targetHeightCm);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.getElevatorSubsystem().stopMotor();
        RobotContainer.getElevatorSubsystem().setMotorToBrake();
        System.out.println("Ending SetElevatorHeightCommand");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return (RobotContainer.getElevatorSubsystem().atSetpoint());
        return true;
    }
}
