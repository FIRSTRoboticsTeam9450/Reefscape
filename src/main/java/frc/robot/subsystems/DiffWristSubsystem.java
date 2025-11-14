package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class DiffWristSubsystem extends SubsystemBase {


    private final SendableChooser<NeutralModeValue> neutralModeChooser;


    
    private static DiffWristSubsystem DW;
    
    // PID
    private PIDController pitchPID = new PIDController(20, 0, 0); // Used to be 4, 0, 0.25
    private PIDController rollPID = new PIDController(20, 0, 0);

    // // Motors
    private TalonFX leftMotor = new TalonFX(RobotConstants.WristIDs.kDiffWristLeftMotorID, RobotConstants.RIO_BUS);
    private TalonFX rightMotor = new TalonFX(RobotConstants.WristIDs.kDiffWristRightMotorID, RobotConstants.RIO_BUS);

    //Encoders
    private CANcoder pitchEncoder = new CANcoder(RobotConstants.WristIDs.kDiffWristPitchCANCoderID, RobotConstants.RIO_BUS);
    private CANcoder rollEncoder = new CANcoder(RobotConstants.WristIDs.kDiffWristRollCANCoderID, RobotConstants.RIO_BUS);

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

    private NeutralModeValue currentNeutralModeValue = RobotConstants.defaultNeutral;

    // Variables
    private boolean runPID = true;

    /* ----- Initialization ----- */

    RadioSoftware radio = RadioSoftware.getInstance();
    private DiffWristSubsystem() {

        neutralModeChooser = new SendableChooser<>();
        neutralModeChooser.setDefaultOption("Default", RobotConstants.defaultNeutral);
        neutralModeChooser.addOption("Brake", NeutralModeValue.Brake);
        neutralModeChooser.addOption("Coast", NeutralModeValue.Coast);

        //Telemetry
        SmartDashboard.putBoolean("Reefscape/DiffWrist/RunPID?", runPID);

        //Diff Wrist Start point
        if (runPID) {
            pitchPID.setSetpoint(0);
            rollPID.setSetpoint(0);
        }
        radio.addMotor(leftMotor);
        radio.addMotor(rightMotor);
    }


    /**
     * Method used to confiure both CTRE Kraken x60 motors used in the differential wrist
     * @param neutralModeValue Neutral Mode Value (kBrake, kCoast), Default is kBrake
     */
    private void configureMotors(NeutralModeValue neutralModeValue) {
        currentNeutralModeValue = neutralModeValue;
        TalonFXConfigurator leftConfigurator = leftMotor.getConfigurator();
        TalonFXConfigurator rightConfigurator = rightMotor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = neutralModeValue;
        leftConfigurator.apply(config);
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfigurator.apply(config);
    }

    /* ----- Updaters ----- */

    /**
     * Will update the volts to use calculated by the PID
     * @param pos current position
     */
    public void updatePID(double pitchPos, double rollPos) {
        double pitchVoltage = pitchPID.calculate(pitchPos);
        double rollVoltage = rollPID.calculate(rollPos);

        //Pitch voltage is being multiplied by 3 due to the fact that its on a 3:1 gear ration (3 times slower than roll)
        // pitchVoltage *= 3;

        double lVolts = pitchVoltage + rollVoltage;
        double rVolts = pitchVoltage - rollVoltage;
        lVolts = MathUtil.clamp(lVolts, -8, 8);
        rVolts = MathUtil.clamp(rVolts, -8, 8);
        

        setVoltage(lVolts, rVolts);
    }

    @Override
    public void periodic() {

        runPID = SmartDashboard.getBoolean("Reefscape/DiffWrist/RunPID?", false);

        pitchPos = pitchEncoder.getAbsolutePosition().getValueAsDouble() - 0.5833333;  //the number it gets subtracted by is (Angle) / 360, to find angle, just multiple # by 360
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
        }
        if (RobotConstants.debugging.DiffyTuningValues) {
            Logger.recordOutput("Diffy Tuning/Pitch at Setpoint?", atPitchSetpoint());
            Logger.recordOutput("Diffy Tuning/Roll at Setpoint?", atRollSetpoint());
            Logger.recordOutput("Diffy Tuning/Pitch Setpoint", pitchSetpoint);
            Logger.recordOutput("Diffy Tuning/Roll Setpoint", rollSetpoint);
            Logger.recordOutput("Diffy Tuning/Pitch Pos", (pitchPos * 360));
            Logger.recordOutput("Diffy Tuning/Roll Pos", rollPos * 360);
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
        return rollPos * 360;
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
        double pitchAngle = getPitchAngle();
        double pitchSetpoint = getPitchSetpoint();
        if ((pitchAngle > pitchSetpoint - 18) && (pitchAngle < pitchSetpoint + 18)) {
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
        rollPID.setSetpoint(setpoint);
    }

    public boolean atRollSetpoint() {
        double rollAngle = getRollAngle();
        double rollSetpoint = getRollSetpoint();
        if ((rollAngle > rollSetpoint - 10) && (rollAngle < rollSetpoint + 10)) {
            return true;
        }
        return false;
    }

    /**
     * Used for getting the current target of the Pitch
     * @return angle of pitch
     */
    public double getPitchSetpoint() {
        return pitchPID.getSetpoint() * 360;
    }

    /**
     * Used for getting the current target of the Roll
     * @return angle of roll
     */
    public double getRollSetpoint() {
        return rollPID.getSetpoint() * 360;
    }

    public boolean getIfDoingPIDS() {
        return runPID;
    }

    public void putParams(NeutralModeValue neutralModeValue) {
        SmartDashboard.putData("Diffy Neutral Mode", neutralModeChooser);
        configureMotors(neutralModeValue);
    }

    public void updateParams() {
        NeutralModeValue neutralModeValue = neutralModeChooser.getSelected();
        if (currentNeutralModeValue != neutralModeValue) {
        configureMotors(neutralModeValue);
        }
    }

}
