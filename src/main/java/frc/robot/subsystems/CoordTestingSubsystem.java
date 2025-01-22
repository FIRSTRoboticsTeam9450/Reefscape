package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    
    public Map<testingPos, testingPos> allowedPaths = new HashMap<>();

    /**
     * gets the starting angle / position of the encoders
     */
    public CoordTestingSubsystem() {

        pitchEncoder = DW.getPitchAngle();
        rollEncoder = DW.getRollAngle();
        elbowEncoder = Elbow.getAngle();
        elevEncoder = Elev.getPosition();

        allowedPaths.put(testingPos.START, testingPos.CORAL_STORE);

        allowedPaths.put(testingPos.CORAL_STORE, testingPos.INTAKE_ALGAE);
        allowedPaths.put(testingPos.CORAL_STORE,testingPos.INTAKE_ALGAE);
        allowedPaths.put(testingPos.CORAL_STORE, testingPos.INTAKE_SOURCE);

        allowedPaths.put(testingPos.INTAKE_CORAL, testingPos.CORAL_STORE);
        allowedPaths.put(testingPos.INTAKE_CORAL, testingPos.INTAKE_SOURCE);

        allowedPaths.put(testingPos.INTAKE_SOURCE, testingPos.CORAL_STORE);
        allowedPaths.put(testingPos.INTAKE_SOURCE, testingPos.INTAKE_CORAL);

        allowedPaths.put(testingPos.INTAKE_ALGAE, testingPos.CORAL_STORE);
        allowedPaths.put(testingPos.INTAKE_ALGAE, testingPos.ALGAE_STORE);

    }

    @Override
    public void periodic() {
        pitchEncoder = DW.getPitchAngle();
        rollEncoder = DW.getRollAngle();
        elbowEncoder = Elbow.getAngle();
        elevEncoder = Elev.getPosition();

        updatePosition();        
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
        Elev.setSetpoint(0);
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

}
