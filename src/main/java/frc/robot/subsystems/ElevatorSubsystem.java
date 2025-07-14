package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.S1StateValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorIDs;

public class ElevatorSubsystem extends SubsystemBase{

    //Instance of Elevator Subsystem
    private static ElevatorSubsystem elev;

    //Motor instances
    private TalonFX leftMotor = new TalonFX(ElevatorIDs.kLeftMotorID, "CantDrive");
    private TalonFX rightMotor = new TalonFX(ElevatorIDs.kRightMotorID, "CantDrive");

    private double position;
    private double offset;
    private double setpoint;
    private double moveTime;
    private double moveStartTime;

    private boolean atSetpoint;
    private boolean resetDone;
    private boolean inMove;

    // 0.82 is the record going up and down
    double velocity = 90; //77 is closest to max velocity time: 0.82
    double acceleration = 270; // 260 is closest to max acceleration tim: 0.82, going lower makes it between 0.86-0.84
    double jerk = 1000; // Make sure it's not 0 because the arm hit something
    DynamicMotionMagicVoltage m_request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);//.withEnableFOC(true); FOC slowed us down from 0.82 to 0.84
    double currentLimit = 110; // 100 is the max stator current pull
    double kS = 0.6; // Add 0.25 V output to overcome static friction .25 - Gives it a little boost in the very beginning
    double kV = 0.26; // A velocity target of 1 rps results in 0.12 V output .12
    double kA = 0.017; // An acceleration of 1 rps/s requires 0.01 V output .01 - Adds a little boost
    double kP = 3; // A position error of 2.5 rotations results in 12 V output 3.8 - Helps correct positional error
    double kI = 0; // no output for integrated error 0
    double kD = 0.12; // A velocity error of 1 rps results in 0.1 V output 0.1 - Can help correct kV and kA error
    double kG = 0.45; // was originally left to default. this was added so it could be updated 0.55 - Perfect value is when it goes up when you push it up and doesn't go down when you push it down

    // kg is always applied, it counters gravity. 
    //     start low and increase until the elevator slowly creeps up, then backoff
    // ks is applied when starting to move (static resistance) both up and down
    //     when starting to move this will enable it get going then it is removed
    // kv is multiplied by desired velocity
    // ka is multi
    private CANdi candi = new CANdi(ElevatorIDs.kCANdiID, "CantDrive");

    private double cachedPosition = 0;
    private double cachedVelocity = 0;
    private double cachedAcceleration = 0;
    private double cachedMotorVoltage = 0;

    public static ElevatorSubsystem getInstance() {
        if (elev == null) {
            elev = new ElevatorSubsystem();
        }
        return elev;
    }
    RadioSoftware radio = RadioSoftware.getInstance();

    private ElevatorSubsystem() {
        leftMotorConfig();
        rightMotorConfig();
        radio.addMotor(rightMotor);
        radio.addMotor(leftMotor);
    }

    private void rightMotorConfig(){
        TalonFXConfiguration config2 = new TalonFXConfiguration();
        
        config2.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config2.CurrentLimits.StatorCurrentLimitEnable = true;
        config2.CurrentLimits.StatorCurrentLimit = currentLimit;

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }

    private void leftMotorConfig(){
        TalonFXConfiguration config1 = new TalonFXConfiguration();
        TalonFXConfigurator temp1 = leftMotor.getConfigurator();
        config1.MotorOutput.NeutralMode = Constants.defaultNeutral; //temp for when default neutral mode is coast
        config1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config1.CurrentLimits.StatorCurrentLimitEnable = true;
        config1.CurrentLimits.StatorCurrentLimit = currentLimit;
        
        Slot0Configs slot0Configs = config1.Slot0;
        slot0Configs.kS = kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = kI; // no output for integrated error
        slot0Configs.kD = kD; // A velocity error of 1 rps results in 0.1 V output
        slot0Configs.kG = kG; // for gravity

        var motionMagicConfigs = config1.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity;
        motionMagicConfigs.MotionMagicAcceleration = acceleration;
        motionMagicConfigs.MotionMagicJerk = jerk;

        temp1.apply(config1);

    }
    /* ----- Updaters ----- */

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

    final int ATLIMIT = 0;
    final int HIGHUP = 1;
    final int ATSETPOINT = 2;
    final int RESETDONE = 3;

    final int STATESIZE = RESETDONE+1;
    boolean[] state = new boolean[STATESIZE];
    boolean flag = true;
    boolean flag2 = true;

    @Override
    public void periodic() {

        BaseStatusSignal.refreshAll(leftMotor.getPosition(), leftMotor.getAcceleration(), leftMotor.getVelocity(), leftMotor.getMotorVoltage());

        double rawPosition = leftMotor.getPosition().getValueAsDouble();
        position = rawPosition - offset;
        
        boolean atLimit = candi.getS1State().getValue() == S1StateValue.Low;
        if (!resetDone && atLimit){
            offset =  rawPosition;
            resetDone = true;
        }

        if (m_request.Velocity != velocity || m_request.Acceleration != acceleration || m_request.Jerk != jerk){
            m_request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);//.withEnableFOC(true); FOC slowed us down from 0.82 to 0.84
            System.out.println("new request("+velocity+", "+acceleration+", "+jerk+")");
        }

        leftMotor.setControl(m_request.withPosition(setpoint + offset));
        
        atSetpoint = Math.abs(position - setpoint) < .3;
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

        boolean highUp = position > 24;

        // we really don't want our subsystem calling into the RobotContainer
        // but a simple solution is not apparent
        RobotContainer.setLiftUp(highUp); 
        
        {
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
                Logger.recordOutput("elev/motionAdj", motionAdj);
                
            }
            Logger.recordOutput("elev/motion", motion[motionIndex]);
            Logger.recordOutput("elev/Stator", leftMotor.getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("elev/Supply", leftMotor.getSupplyCurrent().getValueAsDouble());
            state[ATLIMIT] = atLimit;
            state[ATSETPOINT] = atSetpoint;
            state[HIGHUP] = highUp;
            state[RESETDONE] = resetDone;
            Logger.recordOutput("elev/state", state);
        }

        cachedPosition = leftMotor.getPosition().getValueAsDouble();
        cachedVelocity = leftMotor.getVelocity().getValueAsDouble();
        cachedAcceleration = leftMotor.getAcceleration().getValueAsDouble();
        cachedMotorVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
        
        Logger.recordOutput("Status Signal Testing/ElevL Position", cachedPosition);
        Logger.recordOutput("Status Signal Testing/ElevL Velocity", cachedVelocity);
        Logger.recordOutput("Status Signal Testing/ElevL Acceleration", cachedAcceleration);
        Logger.recordOutput("Status Signal Testing/ElevL Motor Voltage", cachedMotorVoltage);

    }

    /* ----- Getters & Setters ----- */

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
        // if(multiplier < 0) {
        //     velocity = -20;
        // }
        // else if(multiplier == 0) {
        //     velocity = 0;
        // }
        // else{
        //     velocity = 20;
        // }
        //velocity = 40 * multiplier;
        //acceleration = 125 * multiplier;
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
        dynamicPut("elev_velocity",velocity);
        dynamicPut("elev_acc", acceleration);
        dynamicPut("elev_jerk",jerk);

        dynamicPut("elev_currentlimit",currentLimit);

        dynamicPut("elev_kS",kS);
        dynamicPut("elev_kV",kV);
        dynamicPut("elev_kA",kA);
        dynamicPut("elev_kP",kP);
        dynamicPut("elev_kI",kI);
        dynamicPut("elev_kD",kD);
        dynamicPut("elev_kG",kG);
  
        leftMotorConfig();
    }

    public void updateParams(){
        velocity = dynamicUpdate("elev_velocity",velocity);
        acceleration = dynamicUpdate("elev_acc", acceleration);
        jerk = dynamicUpdate("elev_jerk",jerk);

        currentLimit = dynamicUpdate("elev_currentlimit",currentLimit);

        kS = dynamicUpdate("elev_kS",kS);
        kV = dynamicUpdate("elev_kV",kV);
        kA = dynamicUpdate("elev_kA",kA);
        kP = dynamicUpdate("elev_kP",kP);
        kI = dynamicUpdate("elev_kI",kI);
        kD = dynamicUpdate("elev_kD",kD);
        kG = dynamicUpdate("elev_kG",kG);
  
        leftMotorConfig();
    }
}
