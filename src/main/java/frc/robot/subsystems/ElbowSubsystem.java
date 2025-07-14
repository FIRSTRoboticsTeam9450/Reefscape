package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristIDs;
import frc.robot.Log;
import frc.robot.Constants;


/**
 * The elbow is the part connecting the elevator and the Diff. Wrist
 * Used for allowing more range up and down for the diff. wrist
 */
public class ElbowSubsystem extends SubsystemBase {
    
    //Instance of the Elbow System
    private static ElbowSubsystem Elbow;

    //Motor
    private TalonFX motor = new TalonFX(WristIDs.kElbowWristMotorID, Constants.CTRE_BUS);
    
    //Encoder
    private CANcoder encoder = new CANcoder(WristIDs.kElbowWristEncoderID, Constants.CTRE_BUS);

    private double elbowAngle;
    private double setpoint;
    private double offsetSetpoint;

    private final double offsetToZeroDegrees = -110.3;
    // Not used currently
    double velocity = 18; //6 is closest to max velocity time: 0.82
    double acceleration = 11; // 5 is closest to max acceleration tim: 0.82, going lower makes it between 0.86-0.84
    double jerk = 400; //256 Make sure it's not 0 because the arm hit something

    double currentLimit = 110; // 100 is the max stator current pull
    double kS = 0; // Add 0.25 V output to overcome static friction .25 - Gives it a little boost in the very beginning
    double kV = 0.33; //0.32 A velocity target of 1 rps results in 0.12 V output .12
    double kA = 0.05; //0.01 An acceleration of 1 rps/s requires 0.01 V output .01 - Adds a little boost
    double kP = 90; //80 A position error of 2.5 rotations results in 12 V output 3.8 - Helps correct positional error
    double kI = 0.001; //0 no output for integrated error 0
    double kD = 0.35; //0.1 A velocity error of 1 rps results in 0.1 V output 0.1 - Can help correct kV and kA error
    double kG = 0.001;
    // Add kG with arm setting later
    
    DynamicMotionMagicVoltage m_request = new DynamicMotionMagicVoltage(0,velocity,acceleration,jerk);

    Log logger;
    private ElbowSubsystem() {
        logger = new Log("elbow", motor, kS, kV, kA, kP, kI, kD, kG, velocity, acceleration, jerk, currentLimit);
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = Constants.robotConfig.getElbowOffset();
        encoder.getConfigurator().apply(cc_cfg);

        setSetpoint(50);
    }

    public void motorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = kI; // no output for integrated error
        slot0Configs.kD = kD; // A velocity error of 1 rps results in 0.1 V output
        slot0Configs.kG = kG;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = Constants.robotConfig.getElbowRatio();
        config.Feedback.RotorToSensorRatio = 30;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        TalonFXConfigurator configurator = motor.getConfigurator();
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configurator.apply(config);
    }

    double motorStatorPull;

    @Override
    public void periodic() {
        motorStatorPull = motor.getStatorCurrent().getValueAsDouble();
        elbowAngle = motor.getPosition().getValueAsDouble() * -360 - offsetToZeroDegrees;
        Logger.recordOutput("Reefscape/Elbow/Motor ENcoder", motor.getRotorPosition().getValueAsDouble());
        Logger.recordOutput("Reefscape/Elbow/Raw Setpoint", offsetSetpoint);
        Logger.recordOutput("Reefscape/Elbow/Raw Motor Rotations", (elbowAngle + offsetToZeroDegrees)/-360);
        Logger.recordOutput("Reefscape/Elbow/Elbow Setpoint", getSetpoint());
        Logger.recordOutput("Reefscape/Elbow/Elbow Angle", elbowAngle);

        if (m_request.Velocity != velocity || m_request.Acceleration != acceleration || m_request.Jerk != jerk){
            m_request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);//.withEnableFOC(true); FOC slowed us down from 0.82 to 0.84
            System.out.println("new request("+velocity+", "+acceleration+", "+jerk+")");
        }

        motor.setControl(m_request.withPosition((setpoint + offsetToZeroDegrees) / -360));

        //SmartDashboard.putNumber("Elbow/encoder pos", getAngle());
        {
            logger.updateLogger(elbowAngle, setpoint, atSetpoint());
        }
        Logger.recordOutput("Diffy Tuning/Elbow Stator Pull", motorStatorPull);
    }

    public double getAngle() {
        return elbowAngle;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        // offsetSetpoint = (setpoint + offsetToZeroDegrees) / -360;
    }

    public boolean atSetpoint() {
        if (Math.abs(elbowAngle - setpoint) < 7) {
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
   
    public void putParams(){
        logger.putParams();
  
        motorConfig();
    }

    public void updateParams(){
        logger.updateParams();
        velocity = logger.dynamicUpdate("elbow_velocity",velocity);
        acceleration = logger.dynamicUpdate("elbow_acc", acceleration);
        jerk = logger.dynamicUpdate("elbow_jerk",jerk);

        currentLimit = logger.dynamicUpdate("elbow_currentlimit",currentLimit);
        kS = logger.dynamicUpdate("elbow_kS",kS);
        kV = logger.dynamicUpdate("elbow_kV",kV);
        kA = logger.dynamicUpdate("elbow_kA",kA);
        kP = logger.dynamicUpdate("elbow_kP",kP);
        kI = logger.dynamicUpdate("elbow_kI",kI);
        kD = logger.dynamicUpdate("elbow_kD",kD);
        kG = logger.dynamicUpdate("elbow_kG",kG);
        motorConfig();
    }
}