package frc.robot.subsystems;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.debugging;
import frc.robot.Constants.testingPos;

public class CoordTestingSubsystem extends SubsystemBase{

    /* ----- Subsystem Instances ----- */
    private static CoordTestingSubsystem CT;
    private DiffWristSubsystem DW = DiffWristSubsystem.getInstance();
    private ElevatorSubsystem Elev = ElevatorSubsystem.getInstance();
    private ElbowSubsystem Elbow = ElbowSubsystem.getInstance();

    /* ----- Encoders ----- */
    private double pitchEncoder;
    private double rollEncoder;
    private double elbowEncoder;
    private double elevEncoder;

    /* ----- Current / Targeting position ----- */
    private testingPos pos = testingPos.START;
    
    public Map<testingPos, Set<testingPos>> allowedPaths = new HashMap<>();
    private Set<testingPos> Start_Set = new HashSet<>();
    private Set<testingPos> Coral_Store_Set = new HashSet<>();
    private Set<testingPos> Coral_Intake_Set = new HashSet<>();
    private Set<testingPos> Source_Intake_Set = new HashSet<>();
    private Set<testingPos> Algae_Intake_Set = new HashSet<>();
    private Set<testingPos> Algae_Store_Set = new HashSet<>();

    /**
     * gets the starting angle / position of the encoders
     */
    public CoordTestingSubsystem() {

        pos = testingPos.START;

        pitchEncoder = DW.getPitchAngle();
        rollEncoder = DW.getRollAngle();
        elbowEncoder = Elbow.getAngle();
        elevEncoder = Elev.getPosition();

        Start_Set.add(testingPos.CORAL_STORE);

        Coral_Store_Set.add(testingPos.INTAKE_CORAL);
        Coral_Store_Set.add(testingPos.INTAKE_ALGAE);
        Coral_Store_Set.add(testingPos.INTAKE_SOURCE);

        Coral_Intake_Set.add(testingPos.CORAL_STORE);
        Coral_Intake_Set.add(testingPos.INTAKE_SOURCE);

        Source_Intake_Set.add(testingPos.CORAL_STORE);
        Source_Intake_Set.add(testingPos.INTAKE_CORAL);

        Algae_Intake_Set.add(testingPos.CORAL_STORE);
        Algae_Intake_Set.add(testingPos.ALGAE_STORE);

        Algae_Store_Set.add(testingPos.CORAL_STORE);

        allowedPaths.put(testingPos.START, Start_Set);

        allowedPaths.put(testingPos.CORAL_STORE, Coral_Store_Set);

        allowedPaths.put(testingPos.INTAKE_CORAL, Coral_Intake_Set);

        allowedPaths.put(testingPos.INTAKE_SOURCE, Source_Intake_Set);

        allowedPaths.put(testingPos.INTAKE_ALGAE, Algae_Intake_Set);

        allowedPaths.put(testingPos.ALGAE_STORE, Algae_Store_Set);

    }

    @Override
    public void periodic() {
        pitchEncoder = DW.getPitchAngle();
        rollEncoder = DW.getRollAngle();
        elbowEncoder = Elbow.getAngle();
        elevEncoder = Elev.getPosition();

        updatePosition();

        if (debugging.CoordPositionDebugging) {
            debugger();
        }
    }

    public void updatePosition() {

        if (pos == testingPos.CORAL_STORE) {
            goToCoralStore();
        } else if (pos == testingPos.ALGAE_STORE) {
            goToAlgaeStore();
        } else if (pos == testingPos.INTAKE_CORAL) {
            goToCoralIntake();
        } else if (pos == testingPos.INTAKE_SOURCE) {
            goToSourceIntake();
        } else if (pos == testingPos.INTAKE_ALGAE) {
            goToAlgaeIntake();
        } else if (pos == testingPos.START) {
            goToStart();
        }

    }

    public void goToStart() {
        if (rollEncoder < 0) {
            DW.setRollSetpoint(-90);
        } else if (rollEncoder > 0) {
            DW.setRollSetpoint(90);
        }
        DW.setPitchSetpoint(-90);
        Elbow.setSetpoint(90);
        if (
            DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            ) {
                Elev.setSetpoint(0);
        }
    }

    public void goToCoralStore() {
        if (rollEncoder < 0) {
            DW.setRollSetpoint(-90);
        } else if (rollEncoder > 0) {
            DW.setRollSetpoint(90);
        }
        DW.setPitchSetpoint(-75);
        Elbow.setSetpoint(60);
        if (
            DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            ) {
                Elev.setSetpoint(0);
        }
    }

    public void goToSourceIntake() {
        Elev.setSetpoint(8);
        if (elevEncoder > 6.5) {
            DW.setPitchSetpoint(-38);
            DW.setRollSetpoint(0);
            Elbow.setSetpoint(8.5);
        }
    }

    public void goToAlgaeStore() {
        Elev.setSetpoint(8);
        DW.setPitchSetpoint(-82);
        DW.setRollSetpoint(-175);
        if (
            DW.atPitchSetpoint()
            && DW.atRollSetpoint()
            ) {
            Elbow.setSetpoint(80);
        }
    }

    public void goToCoralIntake() {
        DW.setPitchSetpoint(-118);
        DW.setRollSetpoint(0);
        Elbow.setSetpoint(-18);
        Elev.setSetpoint(1);
    }

    public void goToAlgaeIntake() {
        Elev.setSetpoint(6);
        if (elevEncoder > 5.25) {
            DW.setRollSetpoint(0);
            DW.setPitchSetpoint(-130);
            Elbow.setSetpoint(-48);
        }
    }

    public void setPosition(testingPos pos) {
        this.pos = pos;
    }

    public static CoordTestingSubsystem getInstance() {
        if (CT == null) {
            CT = new CoordTestingSubsystem();
        }
        return CT;
    }

    public testingPos getPos() {
        return pos;
    }

    public void setPos(testingPos pos) {
        this.pos = pos;
    }

    /* ----------- DEBUGGING ----------- */

    /**
     * Debugger
     */
    public void debugger() {

        SmartDashboard.putNumber("Reefscape/Debugging/Elbow Angle", elbowEncoder);
        SmartDashboard.putNumber("Reefscape/Debugging/Pitch Angle", pitchEncoder);
        SmartDashboard.putNumber("Reefscape/Debugging/Roll Angle", rollEncoder);
        SmartDashboard.putNumber("Reefscape/Debugging/Elevator Revolutions", elevEncoder);

        SmartDashboard.putNumber("Reefscape/Debugging/Elbow Setpoint", Elbow.getSetpoint());
        SmartDashboard.putNumber("Reefscape/Debugging/Pitch Setpoint", DW.getPitchSetpoint());
        SmartDashboard.putNumber("Reefscape/Debugging/Roll Setpoint", DW.getRollSetpoint());
        SmartDashboard.putNumber("Reefscape/Debugging/Elevator Setpoint", Elev.getSetpoint());

        SmartDashboard.putBoolean("Reefscape/Debugging/Elbow atSetpoint?", Elbow.atSetpoint());
        SmartDashboard.putBoolean("Reefscape/Debugging/Pitch atSetpoint?", DW.atPitchSetpoint());
        SmartDashboard.putBoolean("Reefscape/Debugging/Roll atSetpoint?", DW.atRollSetpoint());
        SmartDashboard.putBoolean("Reefscape/Debugging/Elevator atSetpoint?", Elev.atSetpoint());

        SmartDashboard.putString("Reefscape/Debugging/Start Paths", allowedPaths.get(testingPos.START).toString());
        SmartDashboard.putString("Reefscape/Debugging/Coral Store Paths", allowedPaths.get(testingPos.CORAL_STORE).toString());
        SmartDashboard.putString("Reefscape/Debugging/Coral Intake Paths", allowedPaths.get(testingPos.INTAKE_CORAL).toString());

    }
}
