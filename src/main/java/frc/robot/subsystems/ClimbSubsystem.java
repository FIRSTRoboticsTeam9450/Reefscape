package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ClimberIDs;
import frc.robot.Constants.debugging;

/**
 * Climbers, they go up and down... is fun :)
 * <p>
 * They do be lifting the hefty robot
 * 
 * <p> Grabbing Cage: 0.831 </p>
 * <p> Climbing: 0.26 </p>
 * <p> Store: 0.055 </p>
 */
public class ClimbSubsystem extends SubsystemBase {

    /* -------- Instance -------- */
    private static ClimbSubsystem CS;

    /* -------- Components -------- */
    private final SparkFlex climb = new SparkFlex(ClimberIDs.kMotorID, MotorType.kBrushless);
    private final SparkAbsoluteEncoder encoder = climb.getAbsoluteEncoder();
    private final PIDController pid = new PIDController(55, 0, 0.5);
    private final boolean runClimber = Constants.robotConfig.getRunClimber();

    private double maxVolts = 12;

    /* -------- Constructor -------- */
    private ClimbSubsystem() {
        pid.setSetpoint(0.035); // Store position
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* -------- Periodic Update -------- */
    @Override
    public void periodic() {
        if (runClimber) {
            double voltage = updatePIDs(encoder.getPosition());
            setVoltage(-voltage);

            if (debugging.ClimberPos) {
                Logger.recordOutput("Reefscape/Climbers/Motor Revolutions", encoder.getPosition());
                Logger.recordOutput("Reefscape/Climbers/PID Setpoint", pid.getSetpoint());
                // Logger.recordOutput("Reefscape/Climbers/Voltage", voltage);
            }
        }
    }

    /* -------- PID Helpers -------- */
    public double updatePIDs(double pos) {
        double voltage = pid.calculate(pos);
        return MathUtil.clamp(voltage, -maxVolts, maxVolts);
    }

    /* -------- Setters -------- */
    public void setVoltage(double voltage) {
        climb.setVoltage(voltage);
    }

    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
    }

    public void setMaxVolts(double maxVolts) {
        this.maxVolts = Math.abs(maxVolts);
    }

    /* -------- Singleton -------- */
    public static ClimbSubsystem getInstance() {
        if (CS == null) {
            CS = new ClimbSubsystem();
        }
        return CS;
    }
}