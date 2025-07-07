package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WristIDs;

public class Log {
    private boolean inMove;
    private double offset;
    private double moveTime;
    private double moveStartTime;

    private TalonFX motor;
    String subsystem;
    double velocity;
    double acceleration;
    double jerk;
    double currentLimit;
    double kS;
    double kV;
    double kA;
    double kP;
    double kI;
    double kD;
    double kG;
    
    // Indices for logging array
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

    final int HISTORY = 10;  
    int motionIndexBig = HISTORY; // start at 2 so prevIndex = motionIndex-2 is not a problem
    
    double[][] motion = new double[HISTORY][MOTIONSIZE];
    double[] motionAdj = new double[MOTIONSIZE];

    int adjustSize = 9;
    int[] indices = new int[adjustSize];
    
    /**
     * Sets all the inital values
     * @param subsystem
     * @param motor
     * @param kS
     * @param kV
     * @param kA
     * @param kP
     * @param kI
     * @param kD
     * @param kG
     * @param velocity
     * @param acceleration
     * @param jerk
     * @param currentLimit
     */
    public Log(String subsystem, TalonFX motor, double kS, double kV, double kA, double kP, double kI, double kD, double kG, double velocity, double acceleration, double jerk, double currentLimit) {
        this.subsystem = subsystem;
        this.motor = motor;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kG = kG;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
        this.currentLimit = currentLimit;
    }

    /**
     * Records time, updates position, velocity, acceleration graphs
     * @param position
     * @param setpoint
     * @param atSetpoint
     */
    public void updateLogger(double position, double setpoint, boolean atSetpoint) {
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
  
        motionIndexBig++;
        int prevIndex = (motionIndexBig-1) % HISTORY;
        int motionIndex = motionIndexBig % HISTORY;
        
        motion[motionIndex][TIME]      = currTime;
        motion[motionIndex][DELTATIME] = motion[motionIndex][TIME] - motion[prevIndex][TIME];
        motion[motionIndex][POSITION]  = position;
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
                Logger.recordOutput(subsystem + "/motionAdj", motionAdj);
                
            }
            Logger.recordOutput(subsystem + "/motion", motion[motionIndex]);
            Logger.recordOutput(subsystem + "/Stator", motor.getStatorCurrent().getValueAsDouble());
            Logger.recordOutput(subsystem + "/Supply", motor.getSupplyCurrent().getValueAsDouble());
    }

    /**
     * Gets the number that was giving in smart dashboard
     * @param desc
     * @param value
     * @return
     */
    public double dynamicUpdate(String desc, double value) {
        return SmartDashboard.getNumber(desc,value);
    }

    private void dynamicPut(String desc, double current){
        SmartDashboard.putNumber(desc,current);
    }

    /**
     * Put in all of the values needed into smart dashboard
     */
    public void putParams(){
        dynamicPut(subsystem + "_velocity",velocity);
        dynamicPut(subsystem + "_acc", acceleration);
        dynamicPut(subsystem + "_jerk",jerk);

        dynamicPut(subsystem + "_currentlimit",currentLimit);

        dynamicPut(subsystem + "_kS",kS);
        dynamicPut(subsystem + "_kV",kV);
        dynamicPut(subsystem + "_kA",kA);
        dynamicPut(subsystem + "_kP",kP);
        dynamicPut(subsystem + "_kI",kI);
        dynamicPut(subsystem + "_kD",kD);
        dynamicPut(subsystem + "_kG",kG);
  
    }

    /**
     * Updates all of the values based on what's put in onlineViewer
     */
    public void updateParams(){
        velocity = dynamicUpdate(subsystem + "_velocity",velocity);
        acceleration = dynamicUpdate(subsystem + "_acc", acceleration);
        jerk = dynamicUpdate(subsystem + "_jerk",jerk);

        currentLimit = dynamicUpdate(subsystem + "_currentlimit",currentLimit);

        kS = dynamicUpdate(subsystem + "_kS",kS);
        kV = dynamicUpdate(subsystem + "_kV",kV);
        kA = dynamicUpdate(subsystem + "_kA",kA);
        kP = dynamicUpdate(subsystem + "_kP",kP);
        kI = dynamicUpdate(subsystem + "_kI",kI);
        kD = dynamicUpdate(subsystem + "_kD",kD);
        kG = dynamicUpdate(subsystem + "_kG",kG);
  
    }
}