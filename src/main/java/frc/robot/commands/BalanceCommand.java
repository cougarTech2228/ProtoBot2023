package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceCommand extends CommandBase{
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public BalanceCommand(DrivetrainSubsystem drivetrain){
        m_drivetrainSubsystem = drivetrain;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {

        System.out.println("BalanceCommand Command starting");
        m_drivetrainSubsystem.setDriveMotorStatusFramePeriod(10);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double roll = m_drivetrainSubsystem.getRoll();
        double speed = 0.05;
        System.out.println("Roll: " + roll);

        if(roll > 0){
            speed = -speed;
        }

        System.out.println("Drive: " + speed);
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speed * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                0.0,
                0.0,
                m_drivetrainSubsystem.getGyroscopeRotation()));

        m_drivetrainSubsystem.getEncoderCount();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopMotors();
        m_drivetrainSubsystem.setMotorsToBrake();

        // Return the status frame period back to its original value
        m_drivetrainSubsystem.setDriveMotorStatusFramePeriod(20);

        System.out.println("BalanceCommand finished");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(m_drivetrainSubsystem.getRoll()) < 2);
    }
}
