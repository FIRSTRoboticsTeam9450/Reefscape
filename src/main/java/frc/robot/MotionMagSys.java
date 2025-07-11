package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

public class MotionMagSys {

    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator temp;
    MotionMagicConfigs mag = config.MotionMagic;
    DynamicMotionMagicVoltage m_request;
    TalonFX motor;
    public MotionMagSys(TalonFX Tmotor, boolean inverted, boolean CurrentLimitEnabled, double currentLimit, double[] vals){
        motor = Tmotor;
        temp = motor.getConfigurator();

        config.MotorOutput.NeutralMode = Constants.defaultNeutral; //temp for when default neutral mode is coast
        config.MotorOutput.Inverted = (inverted) ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = CurrentLimitEnabled;
        config.CurrentLimits.StatorCurrentLimit = currentLimit;

        config.Slot0.kG = vals[0];
        config.Slot0.kS = vals[1];
        config.Slot0.kV = vals[2];
        config.Slot0.kA = vals[3];
        config.Slot0.kP = vals[4];
        config.Slot0.kI = vals[5];
        config.Slot0.kD = vals[6];

        mag = config.MotionMagic;
        temp.apply(config);
    }



    //Setters
    public void setTargetParams(double pos, double velocity, double accel, double jerk){
        mag.MotionMagicCruiseVelocity = velocity; // Target cruise velocity of 80 rps
        mag.MotionMagicAcceleration = accel; // Target acceleration of 160 rps/s (0.5 seconds)
        mag.MotionMagicJerk = jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
        m_request = new DynamicMotionMagicVoltage(pos, velocity, accel, jerk);
        motor.setControl(m_request.withPosition(pos));
    }

    public void invert(boolean inverted){
        config.MotorOutput.Inverted = (inverted) ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    }

    public void setVals(double[] vals){
        config.Slot0.kG = vals[0];
        config.Slot0.kS = vals[1];
        config.Slot0.kV = vals[2];
        config.Slot0.kA = vals[3];
        config.Slot0.kP = vals[4];
        config.Slot0.kI = vals[5];
        config.Slot0.kD = vals[6];
        temp.apply(config);
    }

    //Getters
    public double getCurrentLim(){
        return config.CurrentLimits.StatorCurrentLimit;
    }
    
    
    public double[] getVals(){
        double[] v = {config.Slot0.kG,
            config.Slot0.kS,
            config.Slot0.kV,
            config.Slot0.kA,
            config.Slot0.kP,
            config.Slot0.kI,
            config.Slot0.kD};
        return v;
    }

    public void addFollower(TalonFX FollowMotor, boolean inverted){
        FollowMotor.setControl(new Follower(motor.getDeviceID(), inverted));
    }
}
