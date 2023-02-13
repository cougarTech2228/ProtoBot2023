package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class StrafeCommand extends CommandBase {
    private double m_distanceCM;
    private double m_speed;

    double m_currentEncoderCount;
    double m_startEncoderCount;
    double m_distanceInEncoderCounts;

    boolean m_accountForAprilTag;

    boolean m_hasStartedMoving;

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
    public StrafeCommand(double distanceCM, double speed, boolean accountForAprilTag) {

        m_distanceCM = distanceCM;
        m_speed = speed;
        m_accountForAprilTag = accountForAprilTag;
    }

    @Override
    public void initialize() {

        System.out.println("Strafe Command starting");

        if (m_speed < -1.0 || m_speed > 1.0) {
            System.out.println("ERROR: Speed value passed into StrafeCommand out of [-1, 1]");
            m_speed = 0.0;
        }

        // Increase the status frame period just for the life of this command
        // in order to try to get more accuracy in the encoder tick count
        RobotContainer.getDrivetrainSubsystem().setDriveMotorStatusFramePeriod(FAST_STATUS_FRAME_PERIOD);

        double correctedDistanceCM = m_distanceCM;
        double offsetInCm = 0.0;
        // If we're strafing away from an April Tag and it's not a manual strafe from
        // the Button Board's joystick, use the last recorded Tx value from the April
        // Tag detection to make a better estimate on how far we need to strafe in
        // order to line up with the Cone Nodes.
        if (m_accountForAprilTag) {
            offsetInCm = RobotContainer.getAprilTagManager().getTX() * 100.0;
            System.out.println("offsetInCm: " + offsetInCm);

            if (m_speed < 0.0) {

                // Auto is not FOV, need to invert control direction/speed
                if (DriverStation.isAutonomous()) {
                    System.out.println("Strafing Left");
                    correctedDistanceCM += offsetInCm;
                } else {
                    System.out.println("Strafing Right");
                    correctedDistanceCM -= offsetInCm;
                }
            } else {

                // Auto is not FOV, need to invert control direction/speed
                if (DriverStation.isAutonomous()) {
                    System.out.println("Strafing Right");
                    correctedDistanceCM -= offsetInCm;
                } else {
                    System.out.println("Strafing Left");
                    correctedDistanceCM += offsetInCm;
                }
            }
        }

        System.out.println("offsetInCm: " + offsetInCm + " correctedDistanceCM: " + correctedDistanceCM);

        m_distanceInEncoderCounts = ((correctedDistanceCM / WHEEL_CIRCUMFERENCE_CM) * TICKS_PER_ROTATION);

        m_currentEncoderCount = RobotContainer.getDrivetrainSubsystem().getEncoderCount();
        m_startEncoderCount = m_currentEncoderCount;

        m_hasStartedMoving = false;
    }

    @Override
    public void execute() {
        // if (RobotContainer.getDrivetrainSubsystem().getEncoderRateOfChange() > 0.0) {
        // m_hasStartedMoving = true;
        // }

        RobotContainer.getDrivetrainSubsystem().drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0,
                m_speed * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                0.0,
                RobotContainer.getDrivetrainSubsystem().getGyroscopeRotation()));

        m_currentEncoderCount = RobotContainer.getDrivetrainSubsystem().getEncoderCount();
    }

    @Override
    public boolean isFinished() {
        // TODO - Sometimes the robot stops without being obstructed, WTF?
        // Check to see if the robot has stopped moving prematurely
        // if (m_hasStartedMoving &&
        // (RobotContainer.getDrivetrainSubsystem().getEncoderRateOfChange() == 0.0)) {
        // System.out.println("Robot is obstructed, ending strafe command");
        // return true;
        // } else {
        // Checks for both encoder count directions
        return ((m_currentEncoderCount <= (m_startEncoderCount - m_distanceInEncoderCounts)) ||
                (m_currentEncoderCount >= (m_startEncoderCount + m_distanceInEncoderCounts)));
        // }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.getDrivetrainSubsystem().stopMotors();
        RobotContainer.getDrivetrainSubsystem().setMotorsToBrake();

        // Return the status frame period back to its original value
        RobotContainer.getDrivetrainSubsystem().setDriveMotorStatusFramePeriod(ORIGINAL_FRAME_STATUS_PERIOD);

        System.out.println("StrafeCommand finished");
    }
}