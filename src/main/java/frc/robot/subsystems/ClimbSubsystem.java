package frc.robot.subsystems;

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
import frc.robot.Constants.ClimberIDs;
import frc.robot.Constants.debugging;

/**
 * Climbers, they go up and down... is fun :)
 */
public class ClimbSubsystem extends SubsystemBase {

    /* ----- Subsystem Instance ----- */
    private static ClimbSubsystem CS;

    /* ----- PID Controller ----- */
    private PIDController pid = new PIDController(60, 0, 0);
    
    /* ----- Motor ----- */
    // private SparkFlex climb = new SparkFlex(ClimberIDs.kMotorID, MotorType.kBrushless);
    private SparkFlex climb = new SparkFlex(ClimberIDs.kMotorID, MotorType.kBrushless);
    private SparkAbsoluteEncoder encoder = climb.getAbsoluteEncoder();

    private double maxVolts = 12;
    private double minVolts = -12;

    /* ----------- Initializaton ----------- */

    private ClimbSubsystem() {
        pid.setSetpoint(0.58);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast);
        climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* ----------- Updaters ----------- */
    // .95store .05deploy.465climb
    @Override
    public void periodic() {
        double voltage = updatePIDs(encoder.getPosition());
        setVoltage(-voltage);
        if (debugging.ClimberPos) {
            SmartDashboard.putNumber("Reefscape/Debugging/Climbers/Motor Revolutions", encoder.getPosition());
            SmartDashboard.putNumber("Reefscape/Debugging/Climbers/PID Setpoint", pid.getSetpoint());
            SmartDashboard.putNumber("Reefscape/Debugging/Climbers/Voltage", voltage);
        }
    }

    public double updatePIDs(double pos) {
        double voltage = pid.calculate(pos);
        voltage = MathUtil.clamp(maxVolts, minVolts, voltage);
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
        if (maxVolts > 0) {
            this.maxVolts = maxVolts;
        } else if (maxVolts < 0) {
            minVolts = maxVolts;
        }
    }

    public static ClimbSubsystem getInstance() {
        if (CS == null) {
            CS = new ClimbSubsystem();
        }
        return CS;
    }

}
