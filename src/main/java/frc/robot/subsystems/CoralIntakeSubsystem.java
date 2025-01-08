package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeIDS;

public class CoralIntakeSubsystem extends SubsystemBase{
    
    //Instance of Subsystem object
    private static CoralIntakeSubsystem CI;

    // PID
    PIDController rollPID = new PIDController(0.01, 0, 0);
    PIDController yawPID = new PIDController(0.01, 0, 0);

    // Motors
    private SparkFlex leftMotor = new SparkFlex(IntakeIDS.kCoralIntakeLeftMotorID, MotorType.kBrushless);
    private SparkFlex rightMotor = new SparkFlex(IntakeIDS.kCoralIntakeRightMotorID, MotorType.kBrushless);

    //Encoders
    private AbsoluteEncoder rollEncoder = leftMotor.getAbsoluteEncoder(); //Max: 0.35, 0.8    positions to go to: Score: .75, hold: .5
    private AbsoluteEncoder yawEncoder = rightMotor.getAbsoluteEncoder(); //Max: .75, .16   Positions to go to:  Grab: .7,  Score: .2   hold: .45

    // Variables
    public static boolean runPID = true;
    public static double voltage;

    /* ----- Initialization ----- */

    public CoralIntakeSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(false);
        config.smartCurrentLimit(40);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putBoolean("Reefscape/CoralIntake/RunPID?", runPID);
        SmartDashboard.putNumber("Reefscape/CoralIntake/Voltage", voltage);
        if (runPID) {
            rollPID.setSetpoint(.5);
            yawPID.setSetpoint(.45);
        }
    }

    /* ----- Updaters ----- */

    /**
     * Will update the volts to use calculated by the PID
     * @param pos current position
     */
    public void updateRollPID(double pos) {
        double volts = rollPID.calculate(pos);
        // double volts = MathUtil.clamp(PID.calculate(pos), -0.125, 0.125);
        SmartDashboard.putNumber("Reefscape/CoralIntake/rollPIDVolts", volts);
    }

    public void updateYawPID(double pos) {
        double volts = rollPID.calculate(pos);
        // double volts = MathUtil.clamp(PID.calculate(pos), -0.125, 0.125);
        SmartDashboard.putNumber("Reefscape/CoralIntake/yawPIDVolts", volts);
    }

    @Override
    public void periodic() {
        runPID = SmartDashboard.getBoolean("Reefscape/CoralIntake/RunPID?", false);
        if (runPID) {
            updateRollPID(rollEncoder.getPosition());
            updateYawPID(yawEncoder.getPosition());
        }
        voltage = MathUtil.clamp(SmartDashboard.getNumber("Reefscape/CoralIntake/Motor Voltages", 0.0), -0.185, 0.185);
        SmartDashboard.putNumber("Reefscape/CoralIntake/Roll Encoder Pos",rollEncoder.getPosition());
        SmartDashboard.putNumber("Reefscape/CoralIntake/Yaw Encoder Pos", yawEncoder.getPosition());
        SmartDashboard.putNumber("Reefscape/CoralIntake/rollPID Setpoint", rollPID.getSetpoint());
        SmartDashboard.putNumber("Reefscape/CoralIntake/yawPID Setpoint", yawPID.getSetpoint());
    }

    /* ----- Setters & Getters ----- */

    /**
     * returns the instance of this subsystem
     * @return istance of the subsystem object
     */
    public static CoralIntakeSubsystem getInstance() {
        if (CI == null) {
            CI = new CoralIntakeSubsystem();
        }
        return CI;
    }

    /**
     * Sets both of the motors is the coral intake system to same voltage
     * Temporary way of usage, use till deemed safe to use a PID
     * @param leftVoltage voltage to set left motor to
     * @param rightVoltage voltage to set right motor to
     */
    public void setVoltage(double leftVoltage, double rightVoltage) {
        // leftMotor.setVoltage(leftVoltage);
        // rightMotor.setVoltage(rightVoltage);
    }

    /**
     * sets the target position of the roll PID
     * @param setpoint
     */
    public void setRollSetpoint(double setpoint) {
        rollPID.setSetpoint(setpoint);
    }

    /**
     * sets the target position of the yaw PID
     * @param setpoint
     */
    public void setYawSetpoint(double setpoint) {
        yawPID.setSetpoint(setpoint);
    }


}