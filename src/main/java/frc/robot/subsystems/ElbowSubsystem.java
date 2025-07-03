package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
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

    private double elbowAngle;
    private double setpoint;
    private double offsetSetpoint;

    private double position;
    private double offset;
    private double moveTime;
    private double moveStartTime;

    private boolean atSetpoint;
    private boolean inMove;
    private final double offsetToZeroDegrees = -110.3;
    // Not used currently
    double velocity = 6; //77 is closest to max velocity time: 0.82
    double acceleration = 5; // 260 is closest to max acceleration tim: 0.82, going lower makes it between 0.86-0.84
    double jerk = 256; // Make sure it's not 0 because the arm hit something

    double currentLimit = 110; // 100 is the max stator current pull
    double kS = 0; // Add 0.25 V output to overcome static friction .25 - Gives it a little boost in the very beginning
    double kV = 0.32; // A velocity target of 1 rps results in 0.12 V output .12
    double kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output .01 - Adds a little boost
    double kP = 80; // A position error of 2.5 rotations results in 12 V output 3.8 - Helps correct positional error
    double kI = 0; // no output for integrated error 0
    double kD = 0.1; // A velocity error of 1 rps results in 0.1 V output 0.1 - Can help correct kV and kA error
    double kG = 0;
    // Add kG with arm setting later
    
    DynamicMotionMagicVoltage m_request = new DynamicMotionMagicVoltage(0,velocity,acceleration,jerk);

    // timestamp, delta time, position (rotation), speed, acceleration, jerk
    final int POSITION = 0;
    final int SPEED = 1;
    final int ACCEL = 2;
    final int JERK = 3;
    final int TIME = 4;
    final int DELTATIME = 5;
    final int OFFSET = 6;
    final int SETPOINT = 7;
    final int MOVETIME = 8;
    final int MOTIONSIZE = MOVETIME+1;

    // save the last 10 entries for use later
    // HISTORY can be reduced to 2 if desired
    final int HISTORY = 10;  
    int motionIndexBig = HISTORY; // start at 2 so prevIndex = motionIndex-2 is not a problem
    
    double[][] motion = new double[HISTORY][MOTIONSIZE];
    double[] motionAdj = new double[MOTIONSIZE];

    int adjustSize = 9;
    int[] indices = new int[adjustSize];
    private ElbowSubsystem() {

        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = Constants.robotConfig.getElbowOffset();
        encoder.getConfigurator().apply(cc_cfg);

        setSetpoint(0);
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
    @Override
    public void periodic() {
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

        atSetpoint = atSetpoint(); //Find out who uses elbow subsystem
        double currTime = Timer.getFPGATimestamp();
        if (!atSetpoint){
            if (!inMove){
                moveStartTime = currTime;
                inMove = true;
            }
            moveTime = currTime - moveStartTime;
        }
        else if (inMove){
            inMove = false;
        }

        //SmartDashboard.putNumber("Elbow/encoder pos", getAngle());
        {
            motionIndexBig++;
            int prevIndex = (motionIndexBig-1) % HISTORY;
            int motionIndex = motionIndexBig % HISTORY;
            
            motion[motionIndex][TIME]      = currTime;
            motion[motionIndex][DELTATIME] = motion[motionIndex][TIME] - motion[prevIndex][TIME];
            motion[motionIndex][POSITION]  = elbowAngle;
            motion[motionIndex][SPEED]     = (motion[motionIndex][POSITION] - motion[prevIndex][POSITION])/motion[motionIndex][DELTATIME];
            motion[motionIndex][ACCEL]     = (motion[motionIndex][SPEED]    - motion[prevIndex][SPEED])   /motion[motionIndex][DELTATIME];
            motion[motionIndex][JERK]      = (motion[motionIndex][ACCEL]    - motion[prevIndex][ACCEL])   /motion[motionIndex][DELTATIME];
            motion[motionIndex][OFFSET]    = offset;
            motion[motionIndex][SETPOINT]  = setpoint;
            motion[motionIndex][MOVETIME]  = moveTime;

            // motion[motionIndex][OFFSET] = (motion[motionIndex][ACCEL]+motion[prevIndex][ACCEL])/2.0;

            if (motionIndexBig>adjustSize){
                for (int i=0; i<adjustSize; i++){
                    indices[i] = (motionIndexBig-i) % HISTORY;
                }
                int middleIndex = indices[(adjustSize+1)/2];

                for (int j=0; j<MOTIONSIZE; j++){
                    motionAdj[j] = motion[middleIndex][j];
                }
                
                motionAdj[SPEED] = 0;
                motionAdj[ACCEL] = 0;
                motionAdj[JERK] = 0;

                for (int j=0; j<adjustSize; j++){
                    motionAdj[SPEED] += motion[indices[j]][SPEED];
                    motionAdj[ACCEL] += motion[indices[j]][ACCEL];
                    motionAdj[JERK] += motion[indices[j]][JERK];
                }
                motionAdj[SPEED] /= adjustSize;
                motionAdj[ACCEL] /= adjustSize;
                motionAdj[JERK] /= adjustSize;
                Logger.recordOutput("elbow/motionAdj", motionAdj);
                
            }
            Logger.recordOutput("elbow/motion", motion[motionIndex]);
            Logger.recordOutput("elbow/Stator", motor.getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("elbow/Supply", motor.getSupplyCurrent().getValueAsDouble());
        }
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
    private double dynamicUpdate(String desc, double current){
        double v = SmartDashboard.getNumber(desc,current);
        // if (v == current){
        //     SmartDashboard.putNumber(desc,current);
        // }
        return v;
    }

    private void dynamicPut(String desc, double current){
        SmartDashboard.putNumber(desc,current);
    }

    public void putParams(){
        dynamicPut("elbow_velocity",velocity);
        dynamicPut("elbow_acc", acceleration);
        dynamicPut("elbow_jerk",jerk);

        dynamicPut("elbow_currentlimit",currentLimit);

        dynamicPut("elbow_kS",kS);
        dynamicPut("elbow_kV",kV);
        dynamicPut("elbow_kA",kA);
        dynamicPut("elbow_kP",kP);
        dynamicPut("elbow_kI",kI);
        dynamicPut("elbow_kD",kD);
        dynamicPut("elbow_kG",kG);
  
        motorConfig();
    }

    public void updateParams(){
        velocity = dynamicUpdate("elbow_velocity",velocity);
        acceleration = dynamicUpdate("elbow_acc", acceleration);
        jerk = dynamicUpdate("elbow_jerk",jerk);

        currentLimit = dynamicUpdate("elbow_currentlimit",currentLimit);

        kS = dynamicUpdate("elbow_kS",kS);
        kV = dynamicUpdate("elbow_kV",kV);
        kA = dynamicUpdate("elbow_kA",kA);
        kP = dynamicUpdate("elbow_kP",kP);
        kI = dynamicUpdate("elbow_kI",kI);
        kD = dynamicUpdate("elbow_kD",kD);
        kG = dynamicUpdate("elbow_kG",kG);
  
        motorConfig();
    }
}
