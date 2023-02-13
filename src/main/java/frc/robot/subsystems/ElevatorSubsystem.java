package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.utils.CT_DigitalInput;

public class ElevatorSubsystem extends PIDSubsystem {

    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0;

    private TalonFX m_motor = new TalonFX(Constants.ELEVATOR_MOTOR_ID);

    private CT_DigitalInput m_upperElevatorLimitSwitch;
    private CT_DigitalInput m_lowerElevatorLimitSwitch;

    public ElevatorSubsystem() {
        super(new PIDController(kP, kI, kD));

        m_motor.setSelectedSensorPosition(0.0);

        m_upperElevatorLimitSwitch = new CT_DigitalInput(Constants.UPPER_ELEVATOR_LIMIT_SWITCH_DIO, true);
        m_lowerElevatorLimitSwitch = new CT_DigitalInput(Constants.LOWER_ELEVATOR_LIMIT_SWITCH_DIO, true);
    }

    @Override
    public double getMeasurement() {

        return m_motor.getSelectedSensorPosition();
    }

    @Override
    public void useOutput(double output, double setpoint) {

        // TODO - Got this from example doc, seems wrong to multiply times zero
        output += setpoint * kF;
        m_motor.set(ControlMode.PercentOutput, output);
    }

    private boolean isUpperElevatorLimitSwitchActive() {
        return m_upperElevatorLimitSwitch.get();
    }

    private boolean isLowerElevatorLimitSwithActive() {
        return m_lowerElevatorLimitSwitch.get();
    }

    public void updateSetpoint(double targetHeightInCM) {
        // TODO - need to convert target cm to encoder counts
        double kEncoderTicksPerCM = 1234.0;
        m_controller.setSetpoint(targetHeightInCM * kEncoderTicksPerCM);
        m_controller.reset();
    }

    public void stopMotor() {
        m_motor.set(ControlMode.PercentOutput, 0.0);
    }

    public void setMotorToBrake() {
        m_motor.setNeutralMode(NeutralMode.Brake);
    }

    public double getCurrentElevatorHeightCm() {
        // TODO - This is needed from the ButtonBoard to make incremental
        // height changes with the up/down buttons
        return 0.0;
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }
}
