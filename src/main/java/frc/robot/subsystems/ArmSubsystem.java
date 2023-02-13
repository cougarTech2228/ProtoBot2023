package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.utils.CT_DigitalInput;

public class ArmSubsystem extends PIDSubsystem {

    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0;

    private TalonFX m_winchMotor = new TalonFX(Constants.ARM_WINCH_MOTOR_ID);

    private Rev2mDistanceSensor m_distMxp;

    private CT_DigitalInput m_minimumArmReachLimitSwitch;

    private double m_currentArmReachCm;

    public ArmSubsystem() {
        super(new PIDController(kP, kI, kD));

        m_distMxp = new Rev2mDistanceSensor(Port.kMXP);

        m_minimumArmReachLimitSwitch = new CT_DigitalInput(Constants.MINIMUM_REACH_LIMIT_SWITCH_DIO, true);
    }

    @Override
    public double getMeasurement() {

        return m_currentArmReachCm;
    }

    @Override
    public void useOutput(double output, double setpoint) {

        // TODO - Got this from example doc, seems wrong to multiply times zero
        output += setpoint * kF;
        m_winchMotor.set(ControlMode.PercentOutput, output);
    }

    public void enableDistanceSensor(boolean isEnabled) {
        m_distMxp.setAutomaticMode(isEnabled);
    }

    public void updateSetpoint(double targetReachInCM) {
        m_controller.setSetpoint(targetReachInCM);
        m_controller.reset();
    }

    public void stopMotor() {
        m_winchMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void setMotorToBrake() {
        m_winchMotor.setNeutralMode(NeutralMode.Brake);
    }

    private boolean isMinimumReachLimitSwitchActive() {
        return m_minimumArmReachLimitSwitch.get();
    }

    public void setGripperOpen(boolean open) {
        if (open) {
            // TODO - Call PCH method to open the gripper
            System.out.println("Opening gripper");
        } else {
            // TODO - Call PCH method to close the gripper
            System.out.println("Closing gripper");
        }
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public double getCurrentArmReachCm() {
        // TODO - This is needed from the ButtonBoard to make incremental
        // reach changes with the Joystick
        return m_currentArmReachCm;
    }

    @Override
    public void periodic() {

        // We can use timestamp updates to see if the measurement's value is 'stale'
        if (m_distMxp.isEnabled() && m_distMxp.isRangeValid()) {
            // System.out.println(m_distMxp.getTimestamp() + ": Range: " +
            // m_distMxp.getRange());
            m_currentArmReachCm = m_distMxp.getRange(Unit.kMillimeters) / 10.0;
        }
    }
}
