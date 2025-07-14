package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorIDs;
import frc.robot.RobotContainer;

/**
 * ElevatorSubsystem manages a synchronized two-motor elevator using Motion Magic,
 * PIDF tuning, and live telemetry logging for performance monitoring.
 */
public class ElevatorSubsystem extends SubsystemBase {

    /* -------------------- Motor and Sensor Setup -------------------- */
    private static ElevatorSubsystem instance;

    private final TalonFX leftMotor = new TalonFX(ElevatorIDs.kLeftMotorID, "CantDrive");
    private final TalonFX rightMotor = new TalonFX(ElevatorIDs.kRightMotorID, "CantDrive");
    private final CANdi candi = new CANdi(ElevatorIDs.kCANdiID, "CantDrive");
    private final RadioSoftware radio = RadioSoftware.getInstance();

    /* -------------------- Control Parameters -------------------- */
    private double velocity = 90;
    private double acceleration = 270;
    private double jerk = 1000;
    private double currentLimit = 110;

    private double kS = 0.6, kV = 0.26, kA = 0.017;
    private double kP = 3, kI = 0, kD = 0.12, kG = 0.45;

    private DynamicMotionMagicVoltage m_request =
        new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);

    /* -------------------- Motion and State -------------------- */
    private double offset, position, setpoint;
    private double moveTime, moveStartTime;

    private boolean resetDone, atSetpoint, inMove;

    /* -------------------- Logging + Signal Caching -------------------- */
    private double cachedPosition, cachedVelocity, cachedAcceleration, cachedMotorVoltage;

    private static final int POSITION = 0, SPEED = 1, ACCEL = 2, JERK = 3;
    private static final int TIME = 4, DELTATIME = 5, OFFSET = 6;
    private static final int SETPOINT_IDX = 7, MOVETIME = 8, MOTIONSIZE = 9;

    private static final int HISTORY = 10;
    private final double[][] motion = new double[HISTORY][MOTIONSIZE];
    private final double[] motionAdj = new double[MOTIONSIZE];
    private final int adjustSize = 9;
    private final int[] indices = new int[adjustSize];
    private int motionIndexBig = HISTORY;

    private static final int ATLIMIT = 0, HIGHUP = 1, ATSETPOINT_IDX = 2, RESETDONE_IDX = 3;
    private final boolean[] state = new boolean[4];

    /* -------------------- Constructor and Initialization -------------------- */
    private ElevatorSubsystem() {
        configureLeftMotor();
        configureRightMotor();
        radio.addMotor(leftMotor);
        radio.addMotor(rightMotor);
    }

    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSubsystem();
        }
        return instance;
    }

    /* -------------------- Motor Configuration -------------------- */
    private void configureLeftMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = currentLimit;

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = kS; slot0.kV = kV; slot0.kA = kA;
        slot0.kP = kP; slot0.kI = kI; slot0.kD = kD;
        slot0.kG = kG;

        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = velocity;
        mm.MotionMagicAcceleration = acceleration;
        mm.MotionMagicJerk = jerk;

        leftMotor.getConfigurator().apply(config);
    }

    private void configureRightMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = currentLimit;

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }

    /* -------------------- Periodic Loop -------------------- */
    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            leftMotor.getPosition(),
            leftMotor.getAcceleration(),
            leftMotor.getVelocity(),
            leftMotor.getMotorVoltage()
        );

        double rawPosition = leftMotor.getPosition().getValueAsDouble();
        position = rawPosition - offset;

        boolean atLimit = candi.getS1State().getValue() == S1StateValue.Low;
        if (!resetDone && atLimit) {
            offset = rawPosition;
            resetDone = true;
        }

        if (profileChanged()) {
            m_request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);
            System.out.println("new request(" + velocity + ", " + acceleration + ", " + jerk + ")");
        }

        leftMotor.setControl(m_request.withPosition(setpoint + offset));
        atSetpoint = Math.abs(position - setpoint) < 0.3;

        trackMovementTiming();
        boolean highUp = position > 24;
        RobotContainer.setLiftUp(highUp);  // Could be abstracted for testability

        recordMotionData();
        recordTelemetry();

        cacheSignals();
    }

    private boolean profileChanged() {
        return m_request.Velocity != velocity ||
               m_request.Acceleration != acceleration ||
               m_request.Jerk != jerk;
    }

    private void trackMovementTiming() {
        double currTime = Timer.getFPGATimestamp();
        if (!atSetpoint) {
            if (!inMove) moveStartTime = currTime;
            inMove = true;
            moveTime = currTime - moveStartTime;
        } else if (inMove) {
            inMove = false;
        }
    }

    private void recordMotionData() {
        double currTime = Timer.getFPGATimestamp();
        motionIndexBig++;
    
        int prevIndex = (motionIndexBig - 1 + HISTORY) % HISTORY;
        int idx = motionIndexBig % HISTORY;
        double deltaTime = currTime - motion[prevIndex][TIME];
        if (deltaTime <= 0) deltaTime = 0.001; // Prevent divide-by-zero
    
        // Basic motion metrics
        motion[idx][TIME]       = currTime;
        motion[idx][DELTATIME]  = deltaTime;
        motion[idx][POSITION]   = position;
        motion[idx][SPEED]      = (position - motion[prevIndex][POSITION]) / deltaTime;
        motion[idx][ACCEL]      = (motion[idx][SPEED] - motion[prevIndex][SPEED]) / deltaTime;
        motion[idx][JERK]       = (motion[idx][ACCEL] - motion[prevIndex][ACCEL]) / deltaTime;
        motion[idx][OFFSET]     = offset;
        motion[idx][SETPOINT_IDX] = setpoint;
        motion[idx][MOVETIME]   = moveTime;
    
        // Smoothing average across last `adjustSize` samples
        if (motionIndexBig >= adjustSize) {
            for (int i = 0; i < adjustSize; i++) {
                indices[i] = (motionIndexBig - i + HISTORY) % HISTORY;
            }
    
            int middle = indices[(adjustSize + 1) / 2];
            System.arraycopy(motion[middle], 0, motionAdj, 0, MOTIONSIZE);
    
            motionAdj[SPEED] = motionAdj[ACCEL] = motionAdj[JERK] = 0;
            for (int i : indices) {
                motionAdj[SPEED] += motion[i][SPEED];
                motionAdj[ACCEL] += motion[i][ACCEL];
                motionAdj[JERK]  += motion[i][JERK];
            }
    
            motionAdj[SPEED] /= adjustSize;
            motionAdj[ACCEL] /= adjustSize;
            motionAdj[JERK]  /= adjustSize;
    
            Logger.recordOutput("elev/motionAdj", motionAdj);
        }
    
        Logger.recordOutput("elev/motion", motion[idx]);
        Logger.recordOutput("elev/Stator", leftMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("elev/Supply", leftMotor.getSupplyCurrent().getValueAsDouble());
    
        state[ATLIMIT] = candi.getS1State().getValue() == S1StateValue.Low;
        state[ATSETPOINT_IDX] = atSetpoint;
        state[HIGHUP] = position > 24;
        state[RESETDONE_IDX] = resetDone;
        Logger.recordOutput("elev/state", state);
    }

    private void recordTelemetry() {
        Logger.recordOutput("Status Signal Testing/ElevL Position", cachedPosition);
        Logger.recordOutput("Status Signal Testing/ElevL Velocity", cachedVelocity);
        Logger.recordOutput("Status Signal Testing/ElevL Acceleration", cachedAcceleration);
        Logger.recordOutput("Status Signal Testing/ElevL Motor Voltage", cachedMotorVoltage);
    }

    private void cacheSignals() {
        cachedPosition = leftMotor.getPosition().getValueAsDouble();
        cachedVelocity = leftMotor.getVelocity().getValueAsDouble();
        cachedAcceleration = leftMotor.getAcceleration().getValueAsDouble();
        cachedMotorVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
    }

    /* -------------------- Getters & Setters -------------------- */
    public double getPosition() {
        return position;
    }

    public void setSetpoint(double pSetpoint) {
        setpoint = pSetpoint;
    }

    public boolean atSetpoint() {
        return atSetpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void updateMotionMagic(double multiplier) {
        // Placeholder for dynamic velocity/acceleration scaling
        // velocity = 40 * multiplier;
        // acceleration = 125 * multiplier;
    }

    /* -------------------- SmartDashboard Param Sync -------------------- */
    private double dynamicUpdate(String desc, double current) {
        return SmartDashboard.getNumber(desc, current);
    }

    private void dynamicPut(String desc, double current) {
        SmartDashboard.putNumber(desc, current);
    }

    public void putParams() {
        dynamicPut("elev_velocity", velocity);
        dynamicPut("elev_acc", acceleration);
        dynamicPut("elev_jerk", jerk);
        dynamicPut("elev_currentlimit", currentLimit);
        dynamicPut("elev_kS", kS);
        dynamicPut("elev_kV", kV);
        dynamicPut("elev_kA", kA);
        dynamicPut("elev_kP", kP);
        dynamicPut("elev_kI", kI);
        dynamicPut("elev_kD", kD);
        dynamicPut("elev_kG", kG);
        configureLeftMotor();
    }

    public void updateParams() {
        velocity = dynamicUpdate("elev_velocity", velocity);
        acceleration = dynamicUpdate("elev_acc", acceleration);
        jerk = dynamicUpdate("elev_jerk", jerk);
        currentLimit = dynamicUpdate("elev_currentlimit", currentLimit);
        kS = dynamicUpdate("elev_kS", kS);
        kV = dynamicUpdate("elev_kV", kV);
        kA = dynamicUpdate("elev_kA", kA);
        kP = dynamicUpdate("elev_kP", kP);
        kI = dynamicUpdate("elev_kI", kI);
        kD = dynamicUpdate("elev_kD", kD);
        kG = dynamicUpdate("elev_kG", kG);
        configureLeftMotor();
    }
}

