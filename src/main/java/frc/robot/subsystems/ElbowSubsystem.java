package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElbowSubsystem extends SubsystemBase{
    TalonFX elbowMotor = new TalonFX(1);
    CANcoder elbowEncoder = new CANcoder(1);
    PIDController elbowPID = new PIDController(1, 1, 1);
    static ElbowSubsystem master;

    public double getPerpendicularWristAngle(){
        double elbowAngle = elbowEncoder.getAbsolutePosition().getValueAsDouble();
        return (0.75 - elbowAngle);
    }

    /**
     * Sets voltage to the motor
     * @param voltage double between 0 and 1 to set speed to motor
     */
    public void setVoltage(double voltage){
        elbowMotor.set(voltage);
    }

    /**
     * Sets target point/angle for the elbow motor to move to
     * @param target double between 0 and 1 dictating where is the target angle
     */
    public void SetTarget(double target){
        elbowPID.setSetpoint(target);
    }

    /**
     * calculates voltage need to move to the target 
     * from current position using PID and ecoder values
     */
    public double getVoltToTarget(){
        double elbowAngle = elbowEncoder.getAbsolutePosition().getValueAsDouble();
        return elbowPID.calculate(elbowAngle);
    }

    /**
     * retuurns instance of the ElbowSubsystem class
     * @return ElbowSubsystem object
     */
    public static ElbowSubsystem getInstance(){
        if (master == null){
            master = new ElbowSubsystem();
        }
        return master;
    }

    @Override
    /**
     * constantly moves toward target angle
     */
    public void periodic() {
        setVoltage(getVoltToTarget());
    }


}
