package frc.robot.subsystems;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.debugging;
import frc.robot.Constants.ScoringPos;

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
    private ScoringPos pos = ScoringPos.START;
    
    public Map<ScoringPos, Set<ScoringPos>> allowedPaths = new HashMap<>();
    private Set<ScoringPos> Start_Set = new HashSet<>();
    private Set<ScoringPos> Coral_Store_Set = new HashSet<>();
    private Set<ScoringPos> Coral_Intake_Set = new HashSet<>();
    private Set<ScoringPos> Source_Intake_Set = new HashSet<>();
    private Set<ScoringPos> Algae_Intake_Set = new HashSet<>();
    private Set<ScoringPos> Algae_Store_Set = new HashSet<>();
    private Set<ScoringPos> Algae_Net_Score_Set = new HashSet<>();
    private Set<ScoringPos> Algae_Processor_Score_Set = new HashSet<>();
    private Set<ScoringPos> Coral_ScoreL1_Set = new HashSet<>();
    private Set<ScoringPos> Coral_ScoreL2_Set = new HashSet<>();
    private Set<ScoringPos> Coral_ScoreL3_Set = new HashSet<>();
    private Set<ScoringPos> Coral_ScoreL4_Set = new HashSet<>();

    

    /**
     * gets the starting angle / position of the encoders
     */
    public CoordTestingSubsystem() {

        pos = ScoringPos.START;

        pitchEncoder = DW.getPitchAngle();
        rollEncoder = DW.getRollAngle();
        elbowEncoder = Elbow.getAngle();
        elevEncoder = Elev.getPosition();

        Start_Set.add(ScoringPos.CORAL_STORE);

        Coral_Store_Set.add(ScoringPos.CORAL_SCOREL1);
        Coral_Store_Set.add(ScoringPos.CORAL_SCOREL2);
        Coral_Store_Set.add(ScoringPos.CORAL_SCOREL3);
        Coral_Store_Set.add(ScoringPos.CORAL_SCOREL4);

        Coral_Store_Set.add(ScoringPos.INTAKE_CORAL);
        Coral_Store_Set.add(ScoringPos.INTAKE_ALGAE);
        Coral_Store_Set.add(ScoringPos.INTAKE_SOURCE);

        Coral_Intake_Set.add(ScoringPos.CORAL_STORE);
        Coral_Intake_Set.add(ScoringPos.INTAKE_SOURCE);

        Source_Intake_Set.add(ScoringPos.CORAL_STORE);
        Source_Intake_Set.add(ScoringPos.INTAKE_CORAL);

        Algae_Intake_Set.add(ScoringPos.CORAL_STORE);
        Algae_Intake_Set.add(ScoringPos.ALGAE_STORE);

        Algae_Store_Set.add(ScoringPos.SCORE_NET);
        Algae_Store_Set.add(ScoringPos.SCORE_PROCESSOR);
        Algae_Store_Set.add(ScoringPos.INTAKE_ALGAE); //temp... maybe
        Algae_Store_Set.add(ScoringPos.CORAL_STORE); //temp

        Coral_ScoreL1_Set.add(ScoringPos.CORAL_STORE);
        Coral_ScoreL2_Set.add(ScoringPos.CORAL_STORE);
        Coral_ScoreL3_Set.add(ScoringPos.CORAL_STORE);
        Coral_ScoreL4_Set.add(ScoringPos.CORAL_STORE);

        Algae_Net_Score_Set.add(ScoringPos.CORAL_STORE);

        Algae_Processor_Score_Set.add(ScoringPos.CORAL_STORE);

        allowedPaths.put(ScoringPos.START, Start_Set);

        allowedPaths.put(ScoringPos.CORAL_STORE, Coral_Store_Set);

        allowedPaths.put(ScoringPos.INTAKE_CORAL, Coral_Intake_Set);

        allowedPaths.put(ScoringPos.INTAKE_SOURCE, Source_Intake_Set);

        allowedPaths.put(ScoringPos.INTAKE_ALGAE, Algae_Intake_Set);

        allowedPaths.put(ScoringPos.ALGAE_STORE, Algae_Store_Set);

        allowedPaths.put(ScoringPos.SCORE_NET, Algae_Net_Score_Set);

        allowedPaths.put(ScoringPos.SCORE_PROCESSOR, Algae_Processor_Score_Set);

        allowedPaths.put(ScoringPos.CORAL_SCOREL1, Coral_ScoreL1_Set);
        allowedPaths.put(ScoringPos.CORAL_SCOREL2, Coral_ScoreL2_Set);
        allowedPaths.put(ScoringPos.CORAL_SCOREL3, Coral_ScoreL3_Set);
        allowedPaths.put(ScoringPos.CORAL_SCOREL4, Coral_ScoreL4_Set);

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

        if (pos == ScoringPos.CORAL_STORE) {
            goToCoralStore();
        } else if (pos == ScoringPos.ALGAE_STORE) {
            goToAlgaeStore();
        } else if (pos == ScoringPos.INTAKE_CORAL) {
            goToCoralIntake();
        } else if (pos == ScoringPos.INTAKE_SOURCE) {
            goToSourceIntake();
        } else if (pos == ScoringPos.INTAKE_ALGAE) {
            goToAlgaeIntake();
        } else if (pos == ScoringPos.SCORE_NET) {
            goToScoreNet();
        } else if(pos == ScoringPos.CORAL_SCOREL1) {
            goToL1();;
        } else if(pos == ScoringPos.CORAL_SCOREL2) {
            goToL2();
        } else if(pos == ScoringPos.CORAL_SCOREL3) {
            goToL3();
        } else if(pos == ScoringPos.CORAL_SCOREL4) {
            goToL4();
        } else if (pos == ScoringPos.SCORE_PROCESSOR) {

        } else if (pos == ScoringPos.START) {
            goToStart();
        }

    }

    public void goToStart() {
        if (elbowEncoder > 20 && (rollEncoder > -45 && rollEncoder < 45)) {
            Elbow.setSetpoint(19); 
        } else {
            rollToClosestSide();
            DW.setPitchSetpoint(-100);
            Elbow.setSetpoint(98);
        } 
        if (
            DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            ) {
                Elev.setSetpoint(0);
        }
    }

    public void goToCoralStore() {
        rollToClosestSide();
        DW.setPitchSetpoint(-130);
        Elbow.setSetpoint(70);
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
        Elev.setSetpoint(12);
        // DW.setPitchSetpoint(0);
        // DW.setRollSetpoint(0);
        // if (
        //     DW.atPitchSetpoint()
        //     && DW.atRollSetpoint()
        //     ) {
        //     Elbow.setSetpoint(0);
        // }
    }
 
    public void goToCoralIntake() {
        DW.setPitchSetpoint(-128);
        DW.setRollSetpoint(0);
        Elbow.setSetpoint(0);
        Elev.setSetpoint(0);
    }

    public void goToAlgaeIntake() {
        Elev.setSetpoint(6.5);
        if (elevEncoder > 5.25) {
            DW.setRollSetpoint(0);
            DW.setPitchSetpoint(-159.78);
            Elbow.setSetpoint(-33.13);
        }
    }

    public void goToScoreNet() {
        Elev.setSetpoint(38);
        if (elevEncoder > 8) {
            DW.setPitchSetpoint(-70);
            DW.setRollSetpoint(-175);
            Elbow.setSetpoint(0);
        }
    }

    public void goToScoreProcessor() {
        DW.setPitchSetpoint(-102.5);
        DW.setRollSetpoint(0);
        Elbow.setSetpoint(-17);
        if (DW.atPitchSetpoint() && DW.atRollSetpoint() && Elbow.atSetpoint()) {
            Elev.setSetpoint(0);
        }
    }

    public void goToL1() {
        DW.setPitchSetpoint(-128);
        //DW.setRollSetpoint(0);
        Elbow.setSetpoint(65);
        if (DW.atPitchSetpoint() && DW.atRollSetpoint() && Elbow.atSetpoint()) {
            Elev.setSetpoint(0);
        }
    }
    
    public void goToL2() {
        Elev.setSetpoint(7.2);
        if (elevEncoder > 6.8) {
            DW.setPitchSetpoint(-110);
            //DW.setRollSetpoint(90);
            rollToClosestSide();
            Elbow.setSetpoint(63);
        }
    }
    
    public void goToL3() {
        Elev.setSetpoint(15.75);
        if (elevEncoder > 8) {
            DW.setPitchSetpoint(-110);
            //DW.setRollSetpoint(90);
            rollToClosestSide();
            Elbow.setSetpoint(63);
        }
    }
    public void goToL4() {
        Elev.setSetpoint(30.5);
        if (elevEncoder > 8) {
            //DW.setPitchSetpoint(-139);
            DW.setPitchSetpoint(-110);
            rollToClosestSide();
            //DW.setRollSetpoint(90);
            //Elbow.setSetpoint(71.5);
            Elbow.setSetpoint(63);
            
        }
    }
    public void setPosition(ScoringPos pos) {
        this.pos = pos;
    }

    public void rollToClosestSide() {
        if (rollEncoder < 0) {
            DW.setRollSetpoint(-90);
        } else if (rollEncoder > 0) {
            DW.setRollSetpoint(90);
        }
    }

    public static CoordTestingSubsystem getInstance() {
        if (CT == null) {
            CT = new CoordTestingSubsystem();
        }
        return CT;
    }

    public ScoringPos getPos() {
        return pos;
    }

    public void setPos(ScoringPos pos) {
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

        SmartDashboard.putString("Reefscape/Debugging/Start Paths", allowedPaths.get(ScoringPos.START).toString());
        SmartDashboard.putString("Reefscape/Debugging/Coral Store Paths", allowedPaths.get(ScoringPos.CORAL_STORE).toString());
        SmartDashboard.putString("Reefscape/Debugging/Coral Intake Paths", allowedPaths.get(ScoringPos.INTAKE_CORAL).toString());

    }
}
