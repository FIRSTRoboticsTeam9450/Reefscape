package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberIDs;
import frc.robot.Constants.debugging;

/**
 * Climbers, they go up and down... is fun :)
 * <p>
 *  Grabbing Cage: 0.11
 * </p>
 * <p>
 *  Climbing: 0.6
 * </p>
 * <p>
 *  Store: 0.9
 * </p>
 */
public class ClimbSubsystem extends SubsystemBase {

    /* ----- Subsystem Instance ----- */
    private static ClimbSubsystem CS;

    /* ----- PID Controller ----- */
    private PIDController pid = new PIDController(55, 0, 0.5);
    
    /* ----- Motor ----- */
    private SparkFlex climb = new SparkFlex(ClimberIDs.kMotorID, MotorType.kBrushless);
    private SparkAbsoluteEncoder encoder = climb.getAbsoluteEncoder();

    private double maxVolts = 12;

    private boolean runClimber = Constants.robotConfig.getRunClimber();

    /* ----------- Initializaton ----------- */

    private ClimbSubsystem() {
        // 0.1
        pid.setSetpoint(0.109);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* ----------- Updaters ----------- */

    /*
     * POSITIONS
     * 
     * Grabbing: 0.98
     * Climbing: 0.649
     * Store: 0.211
     * 
     */

    @Override
    public void periodic() {
        if (runClimber) {
            double voltage = updatePIDs(encoder.getPosition());
            setVoltage(-voltage);
            if (debugging.ClimberPos) {
                Logger.recordOutput("Reefscape/Climbers/Motor Revolutions", encoder.getPosition());
                Logger.recordOutput("Reefscape/Climbers/PID Setpoint", pid.getSetpoint());
                Logger.recordOutput("Reefscape/Climbers/Voltage", voltage);
            }
        }
    }

    public double updatePIDs(double pos) {
        double voltage = pid.calculate(pos);
        voltage = MathUtil.clamp(voltage, -maxVolts, maxVolts);
        return voltage;
    }

    /* ----------- Setters & Getters ----------- */

    public void setVoltage(double voltage) {
        climb.setVoltage(voltage);
    }

    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
    }

    public void setMaxVolts(double maxVolts) {
        this.maxVolts = Math.abs(maxVolts);
    }

    public static ClimbSubsystem getInstance() {
        if (CS == null) {
            CS = new ClimbSubsystem();
        }
        return CS;
    }

}
