package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristIDs;


/**
 * The elbow is the part connecting the elevator and the Diff. Wrist
 * Used for allowing more range up and down for the diff. wrist
 */
public class ElbowSubsystem extends SubsystemBase {
    
    //Instance of the Elbow System
    private static ElbowSubsystem Elbow;

    //PID
    PIDController pid = new PIDController(20, 0, 0);

    //Motor
    private TalonFX motor = new TalonFX(WristIDs.KElbowWristMotorID, Constants.CTRE_BUS);
    
    //Encoder
    private CANcoder encoder = new CANcoder(WristIDs.KElbowWristEncoderID, Constants.CTRE_BUS);

    public ElbowSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXConfigurator configurator = motor.getConfigurator();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configurator.apply(config);

        setSetpoint(0);
    }

    @Override
    public void periodic() {
        updatePID(encoder.getPosition().getValueAsDouble());
    }

    public double getEncoder() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public void updatePID(double pos) {
        double voltage = pid.calculate(pos);
        voltage = MathUtil.clamp(voltage, -1, 1);
        setVoltage(voltage);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setSetpoint(double setpoint) {
        setpoint /= -360;
        pid.setSetpoint(setpoint);
    }

    // TEMP: CHANGE
    public boolean atSetpoint() {
        return false;
    }

    public static ElbowSubsystem getInstance() {
        if (Elbow == null) {
            Elbow = new ElbowSubsystem();
        }
        return Elbow;
    }

}
