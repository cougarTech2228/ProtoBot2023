package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveFwdRevCommand extends CommandBase {
    private double m_distanceCM;
    private double m_speed;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    double m_currentEncoderCount;
    double m_startEncoderCount;
    double m_distanceInEncoderCounts;

    // Based on a 4" swerve wheel
    private final static double WHEEL_CIRCUMFERENCE_CM = 31.9278;

    // Falcon ticks per rotation is 2048 * SDS Mk4i Gear Ratio of 6.75:1
    // private final static double TICKS_PER_ROTATION = 2048.0 * 6.75; // 13824
    private final static double TICKS_PER_ROTATION = 12900.00;

    // We need to increase the frequency of the encoder status messages
    // on the drive motors to get a consistently accurate 'distance
    // covered' measurement.
    private final static int ORIGINAL_FRAME_STATUS_PERIOD = 20;
    private final static int FAST_STATUS_FRAME_PERIOD = 10;

    /**
     * 
     * @param distance Distance in CM to drive, should always be positive
     * @param speed    speed from [-1, 1]
     *                 For Field Oriented View (FOV) mode in Teleop, negative
     *                 speed goes right, positive speed goes left.
     *                 For Robot Oriented View mode in Autonomous mode,
     *                 negative speed goes left, positive speed goes right.
     * @throws Exception
     */
    public DriveFwdRevCommand(double distanceCM, double speed,
            DrivetrainSubsystem drivetrainSubsystem) {

        m_distanceCM = distanceCM;
        m_speed = speed;
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {

        System.out.println("DriveFwdRevCommand Command starting");

        if (m_speed < -1.0 || m_speed > 1.0) {
            System.out.println("ERROR: Speed value passed into DriveFwdRevCommand out of [-1, 1]");
            m_speed = 0.0;
        }

        // Increase the status frame period just for the life of this command
        // in order to try to get more accuracy in the encoder tick count
        m_drivetrainSubsystem.setDriveMotorStatusFramePeriod(FAST_STATUS_FRAME_PERIOD);

        m_distanceInEncoderCounts = ((m_distanceCM / WHEEL_CIRCUMFERENCE_CM) * TICKS_PER_ROTATION);

        m_currentEncoderCount = m_drivetrainSubsystem.getEncoderCount();
        m_startEncoderCount = m_currentEncoderCount;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(m_speed * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                0.0,
                0.0,
                m_drivetrainSubsystem.getGyroscopeRotation()));

        m_currentEncoderCount = m_drivetrainSubsystem.getEncoderCount();
    }

    @Override
    public boolean isFinished() {
        // Checks for both encoder count directions
        return ((m_currentEncoderCount <= (m_startEncoderCount - m_distanceInEncoderCounts)) ||
                (m_currentEncoderCount >= (m_startEncoderCount + m_distanceInEncoderCounts)));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopMotors();
        m_drivetrainSubsystem.setMotorsToBrake();

        // Return the status frame period back to its original value
        m_drivetrainSubsystem.setDriveMotorStatusFramePeriod(ORIGINAL_FRAME_STATUS_PERIOD);

        System.out.println("DriveFwdRevCommand finished");
    }
}