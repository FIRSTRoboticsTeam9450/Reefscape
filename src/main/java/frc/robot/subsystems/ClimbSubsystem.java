package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ClimberIDs;
import frc.robot.Constants.debugging;


public class ClimbSubsystem extends SubsystemBase {

    private static ClimbSubsystem instance;

    private final TalonFX climbMotor = new TalonFX(Constants.ClimberIDs.kMotorID, Constants.CTRE_BUS);
    private final CANcoder climbEncoder = new CANcoder(Constants.ClimberIDs.kEncoderID, Constants.CTRE_BUS);

    private PIDController climberPID = new PIDController(55, 0, 0.5);

    private boolean runClimber = Constants.robotConfig.getRunClimber();

    private double maxVolts = 4;
    private double totalMotorAMPPull = 0;
    private Timer timer = new Timer();

    public ClimbSubsystem() {
        configuration();
        climberPID.setSetpoint(0.8);

        Logger.recordOutput("Reefscape/Climbers/Motor connected?", climbMotor.isConnected());
        Logger.recordOutput("Reefscape/Climbers/Motor alive?", climbMotor.isAlive());
        Logger.recordOutput("Reefscape/Climbers/Encoder connected?", climbEncoder.isConnected());

    }

    private void configuration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            timer.restart();
        }
        if (runClimber) {
            double voltage = updatePIDs(climbEncoder.getPosition().getValueAsDouble());
            setVoltage(voltage);
            Logger.recordOutput("Reefscape/Climbers/voltage", voltage);

            totalMotorAMPPull += climbMotor.getSupplyCurrent().getValueAsDouble();

            if (debugging.ClimberPos) {
                Logger.recordOutput("Reefscape/Climbers/Encoder Position", climbEncoder.getPosition(true).getValueAsDouble());
                Logger.recordOutput("Reefscape/Climbers/PID Setpoint", climberPID.getSetpoint());
                Logger.recordOutput("Reefscape/Climbers/Motor AMP Pull", climbMotor.getSupplyCurrent().getValueAsDouble());
                Logger.recordOutput("Reefscape/Climbers/Total Motor AMP Pull", totalMotorAMPPull / 20);
                Logger.recordOutput("Reefscape/Climbers/Avg Motor AMP Pull a sec", (totalMotorAMPPull / 20) / timer.get());

                // Logger.recordOutput("Reefscape/Climbers/Voltage", voltage);
            }
        }
    }

    public double updatePIDs(double pos) {
        double voltage = climberPID.calculate(pos);
        return MathUtil.clamp(voltage, -maxVolts, maxVolts);
    }

    /**
     * Sets the Voltage of the climb motor
     * @param voltage voltage to set climb motor to
     */
    public void setVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    }

    /**
     * Sets the setpoint of the pid
     * @param setpoint Absolute encoder value of where we wish climber to be
     */
    public void setSetpoint(double setpoint) {
        climberPID.setSetpoint(setpoint);
    }

    public void setMaxVolts(double maxVolts) {
        this.maxVolts = Math.abs(maxVolts);
    }

    /**
     * Makes sure there is only ever one instance of a subsystem
     * @return Instance of this subsystem
     */
    public static ClimbSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimbSubsystem();
        }
        return instance;
    }
}