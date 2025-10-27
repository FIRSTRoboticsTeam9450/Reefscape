package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Log;
import frc.robot.Constants.WristIDs;
import frc.robot.Constants.debugging;

public class DiffWristSubsystem extends SubsystemBase {
    
    private static DiffWristSubsystem DW;
    
    // PID
    private PIDController pitchPID = new PIDController(5, 0, 0);
    private PIDController rollPID = new PIDController(40, 0, 0);

    // // Motors
    private TalonFX leftMotor = new TalonFX(WristIDs.kDiffWristLeftMotorID, Constants.CTRE_BUS);
    private TalonFX rightMotor = new TalonFX(WristIDs.kDiffWristRightMotorID, Constants.CTRE_BUS);

    //Encoders
    private CANcoder pitchEncoder = new CANcoder(WristIDs.kDiffWristPitchCANCoderID, Constants.CTRE_BUS);
    private CANcoder rollEncoder = new CANcoder(WristIDs.kDiffWristRollCANCoderID, Constants.CTRE_BUS);

    private double pitchPos;
    private double rollPos;

    double pitchSetpoint;
    double rollSetpoint;
    
    double leftAccel;
    double rightAccel;

    double leftVeloc;
    double rightVeloc;

    double leftStatorPull;
    double rightStatorPull;

    // Variables
    private boolean runPID = true;

    /* ----- Initialization ----- */

    RadioSoftware radio = RadioSoftware.getInstance();
    private DiffWristSubsystem() {

        //Telemetry
        SmartDashboard.putBoolean("Reefscape/DiffWrist/RunPID?", runPID);

        //Motor Configuration
        TalonFXConfigurator leftConfigurator = leftMotor.getConfigurator();
        TalonFXConfigurator rightConfigurator = rightMotor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        leftConfigurator.apply(config);
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfigurator.apply(config);

        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = 0.184326171875;
        rollEncoder.getConfigurator().apply(cc_cfg);
        cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.2;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = -0.095703125;
        pitchEncoder.getConfigurator().apply(cc_cfg);

        //Diff Wrist Start point
        if (runPID) {
            pitchPID.setSetpoint(0);
            rollPID.setSetpoint(0);
        }
        radio.addMotor(leftMotor);
        radio.addMotor(rightMotor);
    }

    /* ----- Updaters ----- */

    /**
     * Will update the volts to use calculated by the PID
     * @param pos current position
     */
    public void updatePID(double pitchPos, double rollPos) {
        double pitchVoltage = pitchPID.calculate(pitchPos);
        double rollVoltage = rollPID.calculate(rollPos);
        Logger.recordOutput("Diffy Tuning/Pitch PID", pitchVoltage);
        Logger.recordOutput("Diffy Tuning/Roll PID", rollVoltage);
        
        //Pitch voltage is being multiplied by 3 due to the fact that its on a 3:1 gear ration (3 times slower than roll)
        pitchVoltage *= 3;

        double lVolts = pitchVoltage - rollVoltage;
        double rVolts = pitchVoltage + rollVoltage;
        lVolts = MathUtil.clamp(lVolts, -8, 8); // Used to be 8
        rVolts = MathUtil.clamp(rVolts, -8, 8); // Used to be 8
        

        setVoltage(lVolts, rVolts);
        Logger.recordOutput("Reefscape/DiffWrist/PID/Expected Left Motor Voltage", lVolts);
        Logger.recordOutput("Reefscape/DiffWrist/PID/Expected Right Motor Voltage", rVolts);
    }

    @Override
    public void periodic() {

        runPID = SmartDashboard.getBoolean("Reefscape/DiffWrist/RunPID?", false);

        pitchPos = pitchEncoder.getAbsolutePosition().getValueAsDouble();
        rollPos = rollEncoder.getAbsolutePosition().getValueAsDouble();

        pitchSetpoint = getPitchSetpoint();
        rollSetpoint = getRollSetpoint();

        leftAccel = leftMotor.getAcceleration().getValueAsDouble();
        rightAccel = rightMotor.getAcceleration().getValueAsDouble();

        leftVeloc = leftMotor.getVelocity().getValueAsDouble();
        rightVeloc = rightMotor.getVelocity().getValueAsDouble();

        leftStatorPull = leftMotor.getStatorCurrent().getValueAsDouble();
        rightStatorPull = rightMotor.getStatorCurrent().getValueAsDouble();
        
        if (runPID) {
            updatePID(pitchPos, rollPos);
            Logger.recordOutput("Reefscape/DiffWrist/PID/Acctual Left Motor Voltage", leftMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Reefscape/DiffWrist/PID/Acctual Right Motor Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
        }
        if (debugging.DiffyTuningValues) {
            Logger.recordOutput("Diffy Tuning/Pitch at Setpoint?", atPitchSetpoint());
            Logger.recordOutput("Diffy Tuning/Roll at Setpoint?", atRollSetpoint());
            Logger.recordOutput("Diffy Tuning/Pitch Setpoint", pitchSetpoint);
            Logger.recordOutput("Diffy Tuning/Roll Setpoint", rollSetpoint);
            Logger.recordOutput("Diffy Tuning/Pitch Pos", (pitchPos * 360));
            Logger.recordOutput("Diffy Tuning/Roll Pos", rollPos * 360 );
            Logger.recordOutput("Diffy Tuning/Left Motor Accel", leftAccel);
            Logger.recordOutput("Diffy Tuning/Right Motor Accel", rightAccel);
            Logger.recordOutput("Diffy Tuning/Left Motor Veloc", leftVeloc);
            Logger.recordOutput("Diffy Tuning/Right Motor Velco", rightVeloc);
            Logger.recordOutput("Diffy Tuning/Left Motor Stator Current", leftStatorPull);
            Logger.recordOutput("Diffy Tuning/Right Motor Stator Current", rightStatorPull);
        }

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
     * @return the current encoder value for pitch in degrees
     */
    public double getPitchAngle() {
        return pitchPos * 360;
    }

    /**
     * Finds the current encoder value for the wrist's roll
     * @return the current encoder value for roll in degrees
     */
    public double getRollAngle() {
        return rollPos * 360;// / 1.0957;
    }
    
    /**
     * Sets both of the motors is the Diff Wrist system to same voltage
     * Temporary way of usage, use till deemed safe to use a PID
     * @param leftVoltage voltage to set left motor to
     * @param rightVoltage voltage to set right motor to
     */
    public void setVoltage(double leftVoltage, double rightVoltage) {
        leftMotor.setControl(new VoltageOut(leftVoltage).withEnableFOC(true));
        rightMotor.setControl(new VoltageOut(rightVoltage).withEnableFOC(true));
        // leftMotor.setVoltage(leftVoltage);
        // rightMotor.setVoltage(rightVoltage);
    }

    /**
     * sets the target position of the pitch PID
     * @param setpoint
     */
    public void setPitchSetpoint(double setpoint) {
        setpoint /= 360;
        // setpoint /= 1.5;
        pitchPID.setSetpoint(setpoint);
    }

    public boolean atPitchSetpoint() {
        double pitchAngle = getPitchAngle();// * 1.5;
        double pitchSetpoint = getPitchSetpoint();
        if ((pitchAngle > pitchSetpoint - 15) && (pitchAngle < pitchSetpoint + 15)) {
            return true;
        }
        return false;
    }

    /**
     * sets the target position of the roll PID
     * @param setpoint
     */
    public void setRollSetpoint(double setpoint) {
        setpoint /= 360;
        setpoint /= 1.05;
        rollPID.setSetpoint(setpoint);
    }

    public boolean atRollSetpoint() {
        double rollAngle = getRollAngle();
        double rollSetpoint = getRollSetpoint();
        if ((rollAngle > rollSetpoint - 10) && (rollAngle < rollSetpoint + 10)) { // used to be 10
            return true;
        }
        return false;
    }

    /**
     * Used for getting the current target of the Pitch
     * @return angle of pitch
     */
    public double getPitchSetpoint() {
        return pitchPID.getSetpoint() * 360;// * 1.5;
    }

    /**
     * Used for getting the current target of the Roll
     * @return angle of roll
     */
    public double getRollSetpoint() {
        return rollPID.getSetpoint() * 360 * 1.05; // / 1.4;
    }

    public boolean getIfDoingPIDS() {
        return runPID;
    }

}
