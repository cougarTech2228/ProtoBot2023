package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DockWithAprilTag implements Runnable {

    private double m_aprilTagId;
    private boolean m_isFOV;

    //private boolean m_hasStartedMoving;

    private static double kDt = 0.02;

    // Distance to Target Correction
    private static final double MAX_FORWARD_DOCKING_VELOCITY = 1.2;
    private static final double MAX_FORWARD_DOCKING_ACCELERATION = 0.3;

    private final TrapezoidProfile.Constraints m_forwardConstraints = new TrapezoidProfile.Constraints(
            MAX_FORWARD_DOCKING_VELOCITY, MAX_FORWARD_DOCKING_ACCELERATION);

    private final double FORWARD_P = 0.6;
    private final double FORWARD_D = 0.0;
    private final ProfiledPIDController m_forwardController = new ProfiledPIDController(FORWARD_P, 0.0, FORWARD_D,
            m_forwardConstraints, kDt);

    // Sideways Correction
    private static final double MAX_SIDEWAYS_DOCKING_VELOCITY = 1.0;
    private static final double MAX_SIDEWAYS_DOCKING_ACCELERATION = 0.3;

    private final TrapezoidProfile.Constraints m_sidewaysConstraints = new TrapezoidProfile.Constraints(
            MAX_SIDEWAYS_DOCKING_VELOCITY, MAX_SIDEWAYS_DOCKING_ACCELERATION);

    private final double SIDEWAYS_P = 0.6;
    private final double SIDEWAYS_D = 0.0;
    private final ProfiledPIDController m_sidewaysController = new ProfiledPIDController(SIDEWAYS_P, 0.0, SIDEWAYS_D,
            m_sidewaysConstraints, kDt);

    private final double TURN_P = 0.05;
    private final double TURN_D = 0.0;
    private final PIDController m_turnController = new PIDController(TURN_P, 0.0, TURN_D);

    // We'll make this a little larger to give the AprilTag detector some time to
    // process
    private static final double DOCKING_DISTANCE_GOAL_METERS = Units.inchesToMeters(26.0);

    private static final double MIN_FORWARD_VELOCITY = 0.2;
    private static final double MIN_SIDEWAYS_VELOCITY = 0.2;

    // If the AprilTag detection loss lasts this amount of time, then we give up
    // trying to reacquire the AprilTag
    private static final double MAX_DETECTION_LOST_TIME_SEC = 0.3;

    private static final double PITCH_CORRECTION_GYRO_ANGLE = 0.0;

    // private double m_startTime = 0;

    public DockWithAprilTag(
            boolean isFOV,
            double aprilTagId) {

        m_isFOV = isFOV;
        m_aprilTagId = aprilTagId;
    }

    public void run() {
        // m_hasStartedMoving = false;

        if (RobotContainer.getAprilTagManager().getTagID() == m_aprilTagId) {

            m_forwardController.setGoal(0.0);
            m_sidewaysController.setGoal(0.0);
            m_turnController.enableContinuousInput(180, -180);
            m_turnController.setTolerance(3, 5);

            double detectionLostTime = 0.0;

            while (true) {

                // If the AprilTag comes up as 2228 that means the detector can't see a
                // tag. We should wait a bit to see if the detection loss is just transitory
                // before giving up on it.
                if ((RobotContainer.getAprilTagManager().getTagID() == Constants.BAD_APRIL_TAG_ID) &&
                        (detectionLostTime == 0.0)) {
                    detectionLostTime = Timer.getFPGATimestamp();
                } else if (RobotContainer.getAprilTagManager().getTagID() == m_aprilTagId) {
                    detectionLostTime = 0.0;
                }

                if ((detectionLostTime != 0.0) &&
                        ((Timer.getFPGATimestamp() - detectionLostTime) > MAX_DETECTION_LOST_TIME_SEC)) {
                    System.out.println("Completely Lost April Tag Detection...");
                    // TODO - should we do this? CommandScheduler.getInstance().cancelAll();
                    break;
                }

                if (RobotContainer.getXboxController().getBButton()) {
                    System.out.println("Driver cancelled command...");
                    break;
                }

                // if (RobotContainer.getDrivetrainSubsystem().getEncoderRateOfChange() > 0) {
                //     m_hasStartedMoving = true;
                // }

                // If we've started moving but then stop moving due to some unforseen issue
                // like being blocked by another robot or field element, we need to kill the
                // thread.
                // TODO - This happens when the robot is not blocked, same as Strafe. Something
                // is going on in the DriveTrainSubystem with the RoC calc.
                // if (m_hasStartedMoving && (RobotContainer.getDrivetrainSubsystem().getEncoderRateOfChange() == 0)) {
                //     System.out.println("Robot is blocked and has stopped moving...");
                //     break;
                // }

                double distanceToTarget = RobotContainer.getAprilTagManager().getTZ();
                double offsetTargetDistance = RobotContainer.getAprilTagManager().getTX();

                double forwardSpeed = -m_forwardController.calculate(distanceToTarget);
                double sidewaysSpeed = m_sidewaysController.calculate(offsetTargetDistance);

                // For velocity PID control, getPositionError returns the velocity error and
                // getVelocity error returns the acceleration error ... kind of messed up
                // naming.
                // double forwardVelocityError = m_forwardController.getPositionError();
                // double forwardAccelerationError = m_forwardController.getVelocityError();
                // double sidewaysVelocityError = m_sidewaysController.getPositionError();
                // double sidewaysAccelerationError = m_sidewaysController.getVelocityError();

                // System.out.printf("FVE: %.2f FAE: %.2f SVE: %.2f SAE: %.2f\n",
                // forwardVelocityError,
                // forwardAccelerationError,
                // sidewaysVelocityError,
                // sidewaysAccelerationError);

                double forwardVelocity = forwardSpeed * MAX_FORWARD_DOCKING_VELOCITY;
                double sidewaysVelocity = sidewaysSpeed * MAX_SIDEWAYS_DOCKING_VELOCITY;

                // Need to ensure minimum velocities that are high enough to move the robot
                if (forwardVelocity < MIN_FORWARD_VELOCITY) {
                    forwardVelocity = MIN_FORWARD_VELOCITY;
                }

                if ((sidewaysVelocity > 0.0) && (sidewaysVelocity < MIN_SIDEWAYS_VELOCITY)) {
                    sidewaysVelocity = MIN_SIDEWAYS_VELOCITY;
                }

                if ((sidewaysVelocity < 0.0) && (sidewaysVelocity > -MIN_SIDEWAYS_VELOCITY)) {
                    sidewaysVelocity = -MIN_SIDEWAYS_VELOCITY;
                }

                ChassisSpeeds chassisSpeeds;

                // System.out.println("Current heading: " +
                // RobotContainer.getDrivetrainSubsystem().getGyroscopeRotation().getDegrees());

                if (m_isFOV) {
                    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-forwardVelocity,
                            -sidewaysVelocity,
                            m_turnController.calculate(
                                    ((RobotContainer.getDrivetrainSubsystem().getGyroscopeRotation().getDegrees()
                                            + 360.0) % 360.0) - 180.0,
                                    PITCH_CORRECTION_GYRO_ANGLE),
                            RobotContainer.getDrivetrainSubsystem().getGyroscopeRotation());
                } else {

                    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardVelocity,
                            sidewaysVelocity,
                            m_turnController.calculate(
                                    RobotContainer.getDrivetrainSubsystem().getGyroscopeRotation().getDegrees(),
                                    PITCH_CORRECTION_GYRO_ANGLE),
                            RobotContainer.getDrivetrainSubsystem().getGyroscopeRotation());
                }

                RobotContainer.getDrivetrainSubsystem().drive(chassisSpeeds);

                // Check to see if we're within docking distance
                if (distanceToTarget < DOCKING_DISTANCE_GOAL_METERS) {
                    System.out.println("Docked with target. Yipee!!!!!");
                    break;
                }

                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        } else {
            System.out.printf("April tag: %.0f not detected!\n", m_aprilTagId);
        }

        RobotContainer.getDrivetrainSubsystem().stopMotors();
    }
}
