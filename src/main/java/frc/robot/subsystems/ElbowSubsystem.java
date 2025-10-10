package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristIDs;
import frc.robot.Log;

/**
 * Subsystem for controlling the robot's elbow joint,
 * which connects the elevator to the differential wrist.
 * Enables extended vertical range of motion for the wrist.
 */
public class ElbowSubsystem extends SubsystemBase {

    // Singleton instance
    private static ElbowSubsystem instance;

    // Motor and encoder
    private final TalonFX motor = new TalonFX(WristIDs.kElbowWristMotorID, Constants.RIO_BUS);
    private final CANcoder encoder = new CANcoder(WristIDs.kElbowWristEncoderID, Constants.RIO_BUS);

    // Measurement and control variables
    private double elbowAngle;
    private double setpoint;
    private final double offsetToZeroDegrees = 0;

    // Motion Magic parameters
    private double velocity = 18;
    private double acceleration = 11;
    private double jerk = 400;

    // Feedforward and PIDF constants
    private double currentLimit = 110;
    private double kS = 0;
    private double kV = 0.33;
    private double kA = 0.05;
    private double kP = 90;
    private double kI = 0.001;
    private double kD = 0.35;
    private double kG = 0.001;

    // Control request and logging
    private DynamicMotionMagicVoltage m_request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);
    private final Log logger;

    private ElbowSubsystem() {
        logger = new Log("elbow", motor, kS, kV, kA, kP, kI, kD, kG, velocity, acceleration, jerk, currentLimit);
        configureEncoder();
        setSetpoint(50); // default starting position
    }

    // Configure CANcoder settings
    private void configureEncoder() {
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = Constants.robotConfig.getElbowOffset();
        encoder.getConfigurator().apply(cc_cfg);
    }

    // Configure TalonFX motor settings
    public void motorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PIDF settings
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kA = kA;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kG = kG;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Feedback configuration
        config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = Constants.robotConfig.getElbowRatio();
        config.Feedback.RotorToSensorRatio = 30;

        // Motion Magic parameters
        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = velocity;
        mm.MotionMagicAcceleration = acceleration;
        mm.MotionMagicJerk = jerk;

        // Motor output settings
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        // Update readings
        elbowAngle = motor.getPosition().getValueAsDouble() * -360 - offsetToZeroDegrees;
        double motorStatorPull = motor.getStatorCurrent().getValueAsDouble();

        // Logging outputs
        Logger.recordOutput("Reefscape/Elbow/Motor Encoder", motor.getRotorPosition().getValueAsDouble());
        Logger.recordOutput("Reefscape/Elbow/Raw Motor Rotations", (elbowAngle + offsetToZeroDegrees) / -360);
        Logger.recordOutput("Reefscape/Elbow/Elbow Angle", elbowAngle);
        Logger.recordOutput("Reefscape/Elbow/Elbow Setpoint", getSetpoint());
        Logger.recordOutput("Diffy Tuning/Elbow Stator Pull", motorStatorPull);
        // Reconstruct control request if parameters changed
        if (m_request.Velocity != velocity || m_request.Acceleration != acceleration || m_request.Jerk != jerk) {
            m_request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);
            System.out.println("Updated motion profile: (" + velocity + ", " + acceleration + ", " + jerk + ")");
        }

        motor.setControl(m_request.withPosition((setpoint + offsetToZeroDegrees) / -360));
        logger.updateLogger(elbowAngle, setpoint, atSetpoint());
    }

    // Get current elbow angle
    public double getAngle() {
        return elbowAngle;
    }

    // Set target setpoint
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // Check if elbow has reached setpoint
    public boolean atSetpoint() {
        return Math.abs(elbowAngle - setpoint) < 7;
    }

    // Singleton instance access
    public static ElbowSubsystem getInstance() {
        if (instance == null) {
            instance = new ElbowSubsystem();
        }
        return instance;
    }

    // Get current setpoint
    public double getSetpoint() {
        return setpoint;
    }

    // Publish parameters and reapply motor config
    public void putParams() {
        logger.putParams();
        motorConfig();
    }

    // Dynamically update parameters from dashboard
    public void updateParams() {
        logger.updateParams();
        velocity = logger.dynamicUpdate("elbow_velocity", velocity);
        acceleration = logger.dynamicUpdate("elbow_acc", acceleration);
        jerk = logger.dynamicUpdate("elbow_jerk", jerk);

        currentLimit = logger.dynamicUpdate("elbow_currentlimit", currentLimit);
        kS = logger.dynamicUpdate("elbow_kS", kS);
        kV = logger.dynamicUpdate("elbow_kV", kV);
        kA = logger.dynamicUpdate("elbow_kA", kA);
        kP = logger.dynamicUpdate("elbow_kP", kP);
        kI = logger.dynamicUpdate("elbow_kI", kI);
        kD = logger.dynamicUpdate("elbow_kD", kD);
        kG = logger.dynamicUpdate("elbow_kG", kG);

        motorConfig();
    }
}