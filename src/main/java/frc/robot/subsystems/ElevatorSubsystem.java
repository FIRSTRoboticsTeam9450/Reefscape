package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.S1StateValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorIDs;

public class ElevatorSubsystem extends SubsystemBase{

    //Instance of Elevator Subsystem
    private static ElevatorSubsystem elev;

    CommandXboxController controller;

    //PID instance
    private PIDController pid = new PIDController(4, 0, 0);

    //Motor instances
    private TalonFX leftMotor = new TalonFX(ElevatorIDs.kLeftMotorID, "CantDrive");
    private TalonFX rightMotor = new TalonFX(ElevatorIDs.kRightMotorID, "CantDrive");

    private double position;
    private double setpoint;

    private double offset;

    private boolean atLimit;

    private boolean highUp;

    private boolean resetting = true;

    final DynamicMotionMagicVoltage m_request = new DynamicMotionMagicVoltage(0, 75, 160, 1000);

    private CANdi candi = new CANdi(ElevatorIDs.kCANdiID, "CantDrive");

    /* ----- Initialization ----- */

    private ElevatorSubsystem() {

        TalonFXConfiguration config1 = new TalonFXConfiguration();
        TalonFXConfiguration config2 = new TalonFXConfiguration();
        TalonFXConfigurator temp2 = rightMotor.getConfigurator();
        TalonFXConfigurator temp1 = leftMotor.getConfigurator();
        config1.MotorOutput.NeutralMode = Constants.defaultNeutral; //temp for when default neutral mode is coast
        config1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config1.CurrentLimits.StatorCurrentLimitEnable = true;
        config1.CurrentLimits.StatorCurrentLimit = 80;
        
        Slot0Configs slot0Configs = config1.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 3.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    

        temp1.apply(config1);

        config2.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config2.CurrentLimits.StatorCurrentLimitEnable = true;
        config2.CurrentLimits.StatorCurrentLimit = 80;
        temp2.apply(config2);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

    }

    /* ----- Updaters ----- */

    @Override
    public void periodic() {
        position = leftMotor.getPosition().getValueAsDouble() - offset; 
        if (highUp && setpoint < 24 && position < 24) {
            highUp = false;
            // if (controller == null) {
            //     highUp = false;
            // } else if (Math.abs(controller.getLeftX()) < 0.1 && Math.abs(controller.getLeftY()) < 0.1 && Math.abs(controller.getRightX()) < 0.1) {
            //     highUp = false;
            // }
        }
        RobotContainer.setLiftUp(position > 24 || highUp); 
        atLimit = candi.getS1State().getValue() == S1StateValue.Low;
        Logger.recordOutput("Reefscape/Elevator/Setpoint", pid.getSetpoint());
        Logger.recordOutput("Reefscape/Elevator/pos", position);
        Logger.recordOutput("Elevator at limit", atLimit);
        Logger.recordOutput("Reefscape/Elevator/Offset", offset);

        if (resetting && DriverStation.isEnabled()) {
            setSetpoint(getSetpoint() - 0.05);
            if (atLimit) {
                setAtLimit();
                resetting = false;
                offset = getPosition();
            }
        }
        if (!resetting && atLimit) {
            offset = leftMotor.getPosition().getValueAsDouble();
        }
    }

    /* ----- Getters & Setters ----- */

    public double getPosition() {
        return position;
    }

    public static ElevatorSubsystem getInstance() {
        if (elev == null) {
            elev = new ElevatorSubsystem();
        }
        return elev;
    }

    public void setSetpoint(double setpoint) {
        if (setpoint > 15) {
            highUp = true;
        }
        // if (setpoint > 16) {
        //     System.out.println("Electrical gods are angry! Skip one L4/Net");
        //     return;
        // }
        pid.setSetpoint(setpoint);
        this.setpoint = setpoint;
        leftMotor.setControl(m_request.withPosition(setpoint + offset - 0.25));
    }

    public boolean atSetpoint() {
        return Math.abs(getPosition() - getSetpoint()) < 1;
    }

    public double getSetpoint() {
        return pid.getSetpoint();
    }

    public boolean getAtLimit() {
        return atLimit;
    }

    public void setAtLimit() {
        leftMotor.setPosition(0);
    }

    public void reset() {
        resetting = true;
    }

    public void setSlow() {
        m_request.Acceleration = 50;
    }

    public void setFast() {
        m_request.Acceleration = 160;
    }

    public void setController(CommandXboxController controller) {
        this.controller = controller;
    }

    /**
     * Checks if the number of revolutions on the left elevator motor are withing a certain amount of revolutions of the wanted setpoint
     * @return 
     */
    public boolean getAtSetpoint() {
        if ((getPosition() > getSetpoint() - 2.5) && (getPosition() < getSetpoint() + 2.5)) {
            //System.out.println("Elevator At Setpoint: Got to True");
            return true;
        }
        //System.out.println("Elevator At Setpoint: Got To False");
        return false;
    }

}
