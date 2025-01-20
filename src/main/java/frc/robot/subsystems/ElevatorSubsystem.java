package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{

    //Instance of Elevator Subsystem
    private static ElevatorSubsystem elev;

    //PID instance
    private PIDController pid = new PIDController(4, 0, 0);

    //Motor instances
    private TalonFX tempMotor1 = new TalonFX(60, Constants.CTRE_BUS); //TEMPORARY MOTOR ID
    private TalonFX tempMotor2 = new TalonFX(61, Constants.CTRE_BUS); //TEMPORARY MOTOR ID

    double position;

    /* ----- Initialization ----- */

    public ElevatorSubsystem() {

        /* ----- UNTESTED, MIGHT NOT WORK ----- */

        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXConfigurator temp2 = tempMotor2.getConfigurator();
        TalonFXConfigurator temp1 = tempMotor1.getConfigurator();
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        temp1.apply(config);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        temp2.apply(config);

        setSetpoint(0);

        /* ----- END OF TESTING AREA ----- */
    }

    /* ----- Updaters ----- */

    @Override
    public void periodic() {
        position = tempMotor1.getPosition().getValueAsDouble(); 
        updatePID(position);
        SmartDashboard.putNumber("Reefscape/Elevator/Setpoint", pid.getSetpoint());
        SmartDashboard.putNumber("Reefscape/Elevator/pos", position);
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
    }

    public boolean atSetpoint() {
        return false;
    }

    public void setVoltage(double voltage) {
        tempMotor1.setVoltage(voltage);
        tempMotor2.setVoltage(voltage);
    }

}
