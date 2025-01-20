package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristIDs;

public class DiffWristSubsystem extends SubsystemBase {
    
    private static DiffWristSubsystem DW;
    
    // PID
    private PIDController pitchPID = new PIDController(15, 0, 0);
    private PIDController rollPID = new PIDController(20, 0, 0);

    // // Motors
    // private SparkFlex leftMotor = new SparkFlex(WristIDs.kDiffWristLeftMotorID, MotorType.kBrushless);
    // private SparkFlex rightMotor = new SparkFlex(WristIDs.kDiffWristRightMotorID, MotorType.kBrushless);
    private TalonFX leftMotor = new TalonFX(WristIDs.kDiffWristLeftMotorID, Constants.CTRE_BUS);
    private TalonFX rightMotor = new TalonFX(WristIDs.kDiffWristRightMotorID, Constants.CTRE_BUS);

    //Encoders
    // private AbsoluteEncoder pitchEncoder = leftMotor.getAbsoluteEncoder(); //Max: 0.35, 0.8    positions to go to: Score: .75, hold: .5
    // private AbsoluteEncoder rollEncoder = rightMotor.getAbsoluteEncoder(); //Max: .75, .16   Positions to go to:  Grab: .7,  Score: .2   hold: .45
    private CANcoder pitchEncoder = new CANcoder(WristIDs.kDiffWristPitchCANCoderID, Constants.CTRE_BUS);
    private CANcoder rollEncoder = new CANcoder(WristIDs.kDiffWristRollCANCoderID, Constants.CTRE_BUS);

    // Variables
    public static boolean runPID = true;

    /* ----- Initialization ----- */

    public DiffWristSubsystem() {

        //Telemetry
        SmartDashboard.putBoolean("Reefscape/DiffWrist/RunPID?", runPID);

        //Motor Configuration
        TalonFXConfigurator leftConfigurator = leftMotor.getConfigurator();
        TalonFXConfigurator rightConfigurator = rightMotor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfigurator.apply(config);
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfigurator.apply(config);

        //Diff Wrist Start point
        if (runPID) {
            pitchPID.setSetpoint(0);
            rollPID.setSetpoint(0);
        }
    }

    /* ----- Updaters ----- */

    /**
     * Will update the volts to use calculated by the PID
     * @param pos current position
     */
    public void updatePID(double pitchPos, double rollPos) {
        double pitchVoltage = pitchPID.calculate(pitchPos);
        double rollVoltage = rollPID.calculate(rollPos);

        double lVolts = pitchVoltage - rollVoltage;
        double rVolts = pitchVoltage + rollVoltage;
        lVolts = MathUtil.clamp(lVolts, -2, 2);
        rVolts = MathUtil.clamp(rVolts, -2, 2);
        setVoltage(lVolts, rVolts);
    }

    @Override
    public void periodic() {
        runPID = SmartDashboard.getBoolean("Reefscape/DiffWrist/RunPID?", false);
        double pitchPos = pitchEncoder.getAbsolutePosition().getValueAsDouble();
        double rollPos = rollEncoder.getAbsolutePosition().getValueAsDouble();
        if (runPID) {
            updatePID(pitchPos, rollPos);
        }
        SmartDashboard.putNumber("Reefscape/DiffWrist/pitch Encoder Pos", pitchPos);
        SmartDashboard.putNumber("Reefscape/DiffWrist/roll Encoder Pos", rollPos);
        SmartDashboard.putNumber("Reefscape/DiffWrist/pitchPID Setpoint", pitchPID.getSetpoint());
        SmartDashboard.putNumber("Reefscape/DiffWrist/rollPID Setpoint", rollPID.getSetpoint());
    }

    /* ----- Setters & Getters ----- */

    public static DiffWristSubsystem getInstance() {
        if (DW == null) {
            DW = new DiffWristSubsystem();
        }
        return DW;
    }

    /**
     * Finds the current encoder value for the wrist's pitch
     * @return the current encoder value for pitch
     */
    public double getPitchEncoder() {
        return pitchEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Finds the current encoder value for the wrist's roll
     * @return the current encoder value for roll
     */
    public double getRollEncoder() {
        return rollEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Sets both of the motors is the Diff Wrist system to same voltage
     * Temporary way of usage, use till deemed safe to use a PID
     * @param leftVoltage voltage to set left motor to
     * @param rightVoltage voltage to set right motor to
     */
    public void setVoltage(double leftVoltage, double rightVoltage) {
        leftMotor.setVoltage(leftVoltage);
        rightMotor.setVoltage(rightVoltage);
    }

    /**
     * sets the target position of the pitch PID
     * @param setpoint
     */
    public void setPitchSetpoint(double setpoint) {
        setpoint /= 360;
        pitchPID.setSetpoint(setpoint);
    }

    public boolean atPitchSetpoint() {
        return false;
    }
    /**
     * sets the target position of the roll PID
     * @param setpoint
     */
    public void setRollSetpoint(double setpoint) {
        setpoint /= 360;
        rollPID.setSetpoint(setpoint);
    }

    public boolean atRollSetpoint() {
        return false;
    }

}
