package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.S1StateValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorIDs;

public class ElevatorSubsystem extends SubsystemBase{

    //Instance of Elevator Subsystem
    private static ElevatorSubsystem elev;

    //PID instance
    private PIDController pid = new PIDController(4, 0, 0);

    //Motor instances
    private TalonFX leftMotor = new TalonFX(ElevatorIDs.kLeftMotorID, Constants.CTRE_BUS); //TEMPORARY MOTOR ID
    private TalonFX rightMotor = new TalonFX(ElevatorIDs.kRightMotorID, Constants.CTRE_BUS); //TEMPORARY MOTOR ID

    double position;

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private CANdi candi = new CANdi(ElevatorIDs.kCANdiID);

    /* ----- Initialization ----- */

    public ElevatorSubsystem() {

        TalonFXConfiguration config1 = new TalonFXConfiguration();
        TalonFXConfiguration config2 = new TalonFXConfiguration();
        TalonFXConfigurator temp2 = rightMotor.getConfigurator();
        TalonFXConfigurator temp1 = leftMotor.getConfigurator();
        config1.MotorOutput.NeutralMode = Constants.defaultNeutral; //temp for when default neutral mode is coast
        config1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        Slot0Configs slot0Configs = config1.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        
        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigs = config1.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 50; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 90; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 270; // Target jerk of 1600 rps/s/s (0.1 seconds)

        temp1.apply(config1);

        config2.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        temp2.apply(config2);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

    }

    /* ----- Updaters ----- */

    @Override
    public void periodic() {
        position = leftMotor.getPosition().getValueAsDouble(); 
        RobotContainer.setLiftUp(position > 9);
        boolean atLimit = candi.getS1State().getValue() == S1StateValue.Low;
        //updatePID(position);
        SmartDashboard.putNumber("Reefscape/Elevator/Setpoint", pid.getSetpoint());
        SmartDashboard.putNumber("Reefscape/Elevator/pos", position);
        SmartDashboard.putBoolean("Elevator at limit", atLimit);

        if (atLimit) {
            //leftMotor.setPosition(0);
        }
    }

    public double getPosition() {
        return position;
    }

    public void updatePID(double pos) {
        double voltage = pid.calculate(pos);
        voltage = MathUtil.clamp(voltage, -1, 3);
        setVoltage(voltage);
    }

    /* ----- Getters & Setters ----- */

    public static ElevatorSubsystem getInstance() {
        if (elev == null) {
            elev = new ElevatorSubsystem();
        }
        return elev;
    }

    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
        leftMotor.setControl(m_request.withPosition(setpoint));
    }

    public boolean atSetpoint() {
        return false;
    }

    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    public double getSetpoint() {
        return pid.getSetpoint();
    }

    public boolean getAtLimit() {
        return candi.getS1State().getValue() == S1StateValue.Low;
    }

}
