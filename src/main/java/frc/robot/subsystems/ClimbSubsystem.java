package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
 *  <p> When Zeroing: Have climber all the way out to where it barely touches bumper, make sure climber is at a 15 degree angle away from robot </p>
 * 
 * <p> Grabbing Cage: 0.937 </p>
 * <p> Climbing: 0.35 </p>
 * <p> Store: 0.19 </p>
 */
// public class ClimbSubsystem extends SubsystemBase {

//     /* -------- Instance -------- */
//     private static ClimbSubsystem CS;

//     /* -------- Components -------- */
//     private final SparkFlex climb = new SparkFlex(ClimberIDs.kMotorID, MotorType.kBrushless);
//     private final SparkAbsoluteEncoder encoder = climb.getAbsoluteEncoder();
//     private final PIDController pid = new PIDController(55, 0, 0.5);
//     private final boolean runClimber = Constants.robotConfig.getRunClimber();

//     private double maxVolts = 12;

//     /* -------- Constructor -------- */
//     private ClimbSubsystem() {
//         pid.setSetpoint(0.132); // Store position
//         SparkFlexConfig config = new SparkFlexConfig();
//         config.idleMode(IdleMode.kBrake);
//         climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     /* -------- Periodic Update -------- */
//     @Override
//     public void periodic() {
//         if (runClimber) {
//             double voltage = updatePIDs(encoder.getPosition());
//             setVoltage(-voltage);

//             if (debugging.ClimberPos) {
//                 Logger.recordOutput("Reefscape/Climbers/Motor Revolutions", encoder.getPosition());
//                 Logger.recordOutput("Reefscape/Climbers/PID Setpoint", pid.getSetpoint());
//                 // Logger.recordOutput("Reefscape/Climbers/Voltage", voltage);
//             }
//         }
//     }

//     /* -------- PID Helpers -------- */
//     public double updatePIDs(double pos) {
//         double voltage = pid.calculate(pos);
//         return MathUtil.clamp(voltage, -maxVolts, maxVolts);
//     }

//     /* -------- Setters -------- */
//     public void setVoltage(double voltage) {
//         climb.setVoltage(voltage);
//     }

//     public void setSetpoint(double setpoint) {
//         pid.setSetpoint(setpoint);
//     }

//     public void setMaxVolts(double maxVolts) {
//         this.maxVolts = Math.abs(maxVolts);
//     }

//     /* -------- Singleton -------- */
//     public static ClimbSubsystem getInstance() {
//         if (CS == null) {
//             CS = new ClimbSubsystem();
//         }
//         return CS;
//     }
// }

public class ClimbSubsystem extends SubsystemBase {

    private static ClimbSubsystem instance;

    private final TalonFX climbMotor = new TalonFX(Constants.ClimberIDs.kMotorID, Constants.CTRE_BUS);
    private final CANcoder climbEncoder = new CANcoder(Constants.ClimberIDs.kEncoderID, Constants.CTRE_BUS);

    private PIDController climberPID = new PIDController(55, 0, 0.5);

    private boolean runClimber = Constants.robotConfig.getRunClimber();

    private double maxVolts = 4;

    public ClimbSubsystem() {
        configuration();
        climberPID.setSetpoint(0.68);

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
        if (runClimber) {
            double voltage = updatePIDs(climbEncoder.getPosition().getValueAsDouble());
            setVoltage(voltage);

            if (debugging.ClimberPos) {
                Logger.recordOutput("Reefscape/Climbers/Encoder Position", climbEncoder.getPosition(true).getValueAsDouble());
                Logger.recordOutput("Reefscape/Climbers/PID Setpoint", climberPID.getSetpoint());
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