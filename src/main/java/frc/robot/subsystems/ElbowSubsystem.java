package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristIDs;
import frc.robot.Constants;


/**
 * The elbow is the part connecting the elevator and the Diff. Wrist
 * Used for allowing more range up and down for the diff. wrist
 */
public class ElbowSubsystem extends SubsystemBase {
    
    //Instance of the Elbow System
    private static ElbowSubsystem Elbow;

    //Motor
    private TalonFX motor = new TalonFX(WristIDs.KElbowWristMotorID, Constants.CTRE_BUS);
    
    //Encoder
    private CANcoder encoder = new CANcoder(WristIDs.KElbowWristEncoderID, Constants.CTRE_BUS);

    private double angle;
    private double setpoint;
    private double offsetSetpoint;

    private final double offsetToZeroDegrees = -110.3;

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private ElbowSubsystem() {

        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = Constants.robotConfig.getElbowOffset();
        encoder.getConfigurator().apply(cc_cfg);

        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.32; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 80; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = Constants.robotConfig.getElbowRatio();
        config.Feedback.RotorToSensorRatio = 30;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 6; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 5; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 256; // Target jerk of 1600 rps/s/s (0.1 seconds)

        TalonFXConfigurator configurator = motor.getConfigurator();
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configurator.apply(config);

        setSetpoint(0);
    }

    @Override
    public void periodic() {
        angle = motor.getPosition().getValueAsDouble();
        Logger.recordOutput("Reefscape/Elbow/Motor ENcoder", motor.getRotorPosition().getValueAsDouble());
        Logger.recordOutput("Reefscape/Elbow/Raw Setpoint", offsetSetpoint);
        Logger.recordOutput("Reefscape/Elbow/Raw Angle", angle);
        Logger.recordOutput("Reefscape/Elbow/Elbow Setpoint", getSetpoint());
        Logger.recordOutput("Reefscape/Elbow/Elbow Angle", getAngle());
        //SmartDashboard.putNumber("Elbow/encoder pos", getAngle());
    }

    public double getAngle() {
        return angle * -360 - offsetToZeroDegrees;
    }

    public void updatePID(double pos) {
        // double voltage = pid.calculate(pos);
        // voltage = MathUtil.clamp(voltage, -2, 2);
        // setVoltage(voltage);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        offsetSetpoint = (setpoint + offsetToZeroDegrees) / -360;
        motor.setControl(m_request.withPosition(offsetSetpoint));
    }

    public boolean atSetpoint() {
        double elbowAngle = getAngle();
        double setpoint = getSetpoint();
        if ((elbowAngle > setpoint - 7) && (elbowAngle < setpoint + 7)) {
            return true;
        }
        return false;
    }

    public static ElbowSubsystem getInstance() {
        if (Elbow == null) {
            Elbow = new ElbowSubsystem();
        }
        return Elbow;
    }

    /**
     * Used for getting current target of the elbow
     * @return angle of Elbow
     */
    public double getSetpoint() {
        return setpoint;
    }

}
