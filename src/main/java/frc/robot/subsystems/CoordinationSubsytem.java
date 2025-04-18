package frc.robot.subsystems;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.debugging;
import frc.robot.Constants.ScoringPos;
import frc.robot.Constants;

public class CoordinationSubsytem extends SubsystemBase{

    public static boolean autoGround = false;

    /* ----- Subsystem Instances ----- */
    private static CoordinationSubsytem CT;
    private DiffWristSubsystem DW = DiffWristSubsystem.getInstance();
    private ElevatorSubsystem Elev = ElevatorSubsystem.getInstance();
    private ElbowSubsystem Elbow = ElbowSubsystem.getInstance();
    //private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

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
    private Set<ScoringPos> Coral_Score_Set = new HashSet<>();
    private Set<ScoringPos> Score_L4_Set = new HashSet<>();
    private Set<ScoringPos> Algae_L1_Set = new HashSet<>();
    private Set<ScoringPos> Algae_L2_Set = new HashSet<>();
    private Set<ScoringPos> Algae_Grabbed_Set = new HashSet<>();
    private Set<ScoringPos> Coral_Score_Go_Set = new HashSet<>();
    private Set<ScoringPos> Coral_Intake_Vertical_Set = new HashSet<>();
    private Set<ScoringPos> Pre_L4_Set = new HashSet<>();
    private Set<ScoringPos> L1_Store_Set = new HashSet<>();
    private Set<ScoringPos> Lolipop_Intake_Set = new HashSet<>();
    
    boolean algae;

    private int level = 3;
    private int desiredLevel = 3;

    private boolean algaeNet = true;
    private boolean desiredAlgaeNet = true;

    private boolean justHitScore = true;

    private boolean allAtSetpoints = false;
    private boolean justFinished = false;
    private boolean justChanged = false;

    private double elevOriginalSetpoint;
    private double elbowOriginalSetpoint;
    private double pitchOriginalSetpoint;
    private double rollOriginalSetpoint;

    private double elevAllowedDifference = 1.5;
    private double elbowAllowedDifference = 15;
    private double pitchAllowedDifference = 6;

    private boolean coralSideLeft;
    private boolean l4Extend;

    private double coralScoreElbow = 0;
    private double coralScoreElev = 0;
    private double coralScorePitch = 0;

    ScoringPos lastPos = ScoringPos.START;
    /**
     * gets the starting angle / position of the encoders
     */
    private CoordinationSubsytem() {
        pos = ScoringPos.START;

        pitchEncoder = DW.getPitchAngle();
        rollEncoder = DW.getRollAngle();
        elbowEncoder = Elbow.getAngle();
        elevEncoder = Elev.getPosition();

        Start_Set.add(ScoringPos.CORAL_STORE);
        Start_Set.add(ScoringPos.GO_SCORE_CORAL);

        Coral_Store_Set.add(ScoringPos.CORAL_SCOREL1);
        Coral_Store_Set.add(ScoringPos.CORAL_SCOREL2);
        Coral_Store_Set.add(ScoringPos.CORAL_SCOREL3);
        Coral_Store_Set.add(ScoringPos.CORAL_SCOREL4);
        Coral_Store_Set.add(ScoringPos.PRE_L4);

        Coral_Store_Set.add(ScoringPos.START);
        Coral_Store_Set.add(ScoringPos.INTAKE_CORAL);
        Coral_Store_Set.add(ScoringPos.INTAKE_ALGAE);
        Coral_Store_Set.add(ScoringPos.INTAKE_SOURCE);
        Coral_Store_Set.add(ScoringPos.INTAKE_VERTICAL_CORAL);
        Coral_Store_Set.add(ScoringPos.LOLIPOP_INTAKE_ALGAE);

        Coral_Store_Set.add(ScoringPos.ALGAEL1);
        Coral_Store_Set.add(ScoringPos.ALGAEL2);
        Coral_Store_Set.add(ScoringPos.GO_SCORE_CORAL);

        Coral_Store_Set.add(ScoringPos.CORAL_STORE);

        Coral_Intake_Set.add(ScoringPos.CORAL_STORE);
        Coral_Intake_Set.add(ScoringPos.INTAKE_SOURCE);
        Coral_Intake_Set.add(ScoringPos.ALGAEL1);
        Coral_Intake_Set.add(ScoringPos.ALGAEL2);
        Coral_Intake_Set.add(ScoringPos.INTAKE_ALGAE);
        Coral_Intake_Set.add(ScoringPos.LOLIPOP_INTAKE_ALGAE);


        Source_Intake_Set.add(ScoringPos.CORAL_STORE);
        Source_Intake_Set.add(ScoringPos.INTAKE_CORAL);

        Algae_Intake_Set.add(ScoringPos.CORAL_STORE);
        Algae_Intake_Set.add(ScoringPos.ALGAE_STORE);
        Algae_Intake_Set.add(ScoringPos.ALGAEL1);
        Algae_Intake_Set.add(ScoringPos.ALGAEL2);
        Algae_Intake_Set.add(ScoringPos.INTAKE_CORAL);
        Algae_Intake_Set.add(ScoringPos.LOLIPOP_INTAKE_ALGAE);

        Algae_Store_Set.add(ScoringPos.SCORE_NET);
        Algae_Store_Set.add(ScoringPos.SCORE_PROCESSOR);
        Algae_Store_Set.add(ScoringPos.INTAKE_ALGAE); //temp... maybe
        Algae_Store_Set.add(ScoringPos.CORAL_STORE); //temp
        Algae_Store_Set.add(ScoringPos.ALGAEL1);
        Algae_Store_Set.add(ScoringPos.ALGAEL2);
        Algae_Store_Set.add(ScoringPos.GRABBED_ALGAE);
        Algae_Store_Set.add(ScoringPos.GO_SCORE_CORAL);
        Algae_Store_Set.add(ScoringPos.LOLIPOP_INTAKE_ALGAE);
        
        Coral_ScoreL1_Set.add(ScoringPos.CORAL_STORE);
        Coral_ScoreL2_Set.add(ScoringPos.CORAL_STORE);
        Coral_ScoreL3_Set.add(ScoringPos.CORAL_STORE);
        Coral_ScoreL4_Set.add(ScoringPos.CORAL_STORE);

        Coral_ScoreL1_Set.add(ScoringPos.SCORE_CORAL);
        Coral_ScoreL2_Set.add(ScoringPos.SCORE_CORAL);
        Coral_ScoreL3_Set.add(ScoringPos.SCORE_CORAL);
        Coral_ScoreL4_Set.add(ScoringPos.ScoreL4);

        Coral_ScoreL1_Set.add(ScoringPos.GO_SCORE_CORAL);
        Coral_ScoreL2_Set.add(ScoringPos.GO_SCORE_CORAL);
        Coral_ScoreL3_Set.add(ScoringPos.GO_SCORE_CORAL);
        Coral_ScoreL4_Set.add(ScoringPos.GO_SCORE_CORAL);

        Coral_Score_Set.add(ScoringPos.CORAL_STORE);
        Coral_Score_Set.add(ScoringPos.GO_SCORE_CORAL);
        Coral_Score_Set.add(ScoringPos.INTAKE_CORAL);

        Algae_Net_Score_Set.add(ScoringPos.CORAL_STORE);

        Algae_Processor_Score_Set.add(ScoringPos.CORAL_STORE);

        Score_L4_Set.add(ScoringPos.CORAL_STORE);
        Score_L4_Set.add(ScoringPos.GO_SCORE_CORAL);
        Score_L4_Set.add(ScoringPos.INTAKE_CORAL);

        Algae_L1_Set.add(ScoringPos.ALGAE_STORE);
        Algae_L1_Set.add(ScoringPos.SCORE_PROCESSOR);
        Algae_L1_Set.add(ScoringPos.GRABBED_ALGAE);
        Algae_L1_Set.add(ScoringPos.CORAL_STORE);
        Algae_L1_Set.add(ScoringPos.INTAKE_ALGAE);
        Algae_L1_Set.add(ScoringPos.ALGAEL2);
        Algae_L1_Set.add(ScoringPos.GO_SCORE_CORAL);
        Algae_L1_Set.add(ScoringPos.INTAKE_CORAL);
        Algae_L1_Set.add(ScoringPos.LOLIPOP_INTAKE_ALGAE);

        Algae_L2_Set.add(ScoringPos.ALGAE_STORE);
        Algae_L2_Set.add(ScoringPos.SCORE_PROCESSOR);
        Algae_L2_Set.add(ScoringPos.GRABBED_ALGAE);
        Algae_L2_Set.add(ScoringPos.CORAL_STORE);
        Algae_L2_Set.add(ScoringPos.INTAKE_ALGAE);
        Algae_L2_Set.add(ScoringPos.ALGAEL1);
        Algae_L2_Set.add(ScoringPos.GO_SCORE_CORAL);
        Algae_L2_Set.add(ScoringPos.INTAKE_CORAL);
        Algae_L2_Set.add(ScoringPos.LOLIPOP_INTAKE_ALGAE);

        Coral_Score_Go_Set.add(ScoringPos.CORAL_STORE);
        Coral_Score_Go_Set.add(ScoringPos.SCORE_CORAL);
        Coral_Score_Go_Set.add(ScoringPos.GO_SCORE_CORAL);
        Coral_Score_Go_Set.add(ScoringPos.ScoreL4);

        Algae_Grabbed_Set.add(ScoringPos.ALGAE_STORE);
        Algae_Grabbed_Set.add(ScoringPos.CORAL_STORE);
        Algae_Grabbed_Set.add(ScoringPos.SCORE_NET);
        Algae_Grabbed_Set.add(ScoringPos.ALGAEL1);
        Algae_Grabbed_Set.add(ScoringPos.ALGAEL2);
        Algae_Grabbed_Set.add(ScoringPos.INTAKE_ALGAE);
        Algae_Grabbed_Set.add(ScoringPos.GO_SCORE_CORAL);
        Algae_Grabbed_Set.add(ScoringPos.LOLIPOP_INTAKE_ALGAE);

        Coral_Intake_Vertical_Set.add(ScoringPos.CORAL_STORE);
        Coral_Intake_Vertical_Set.add(ScoringPos.START);

        Pre_L4_Set.add(ScoringPos.CORAL_STORE);
        Pre_L4_Set.add(ScoringPos.GO_SCORE_CORAL);
        Pre_L4_Set.add(ScoringPos.INTAKE_CORAL);

        L1_Store_Set.add(ScoringPos.CORAL_STORE);
        L1_Store_Set.add(ScoringPos.GO_SCORE_CORAL);
        L1_Store_Set.add(ScoringPos.INTAKE_CORAL);

        Lolipop_Intake_Set.add(ScoringPos.CORAL_STORE);
        Lolipop_Intake_Set.add(ScoringPos.INTAKE_CORAL);
        Lolipop_Intake_Set.add(ScoringPos.INTAKE_ALGAE);
        Lolipop_Intake_Set.add(ScoringPos.ALGAEL1);
        Lolipop_Intake_Set.add(ScoringPos.ALGAEL2);
        Lolipop_Intake_Set.add(ScoringPos.ALGAE_STORE);

        allowedPaths.put(ScoringPos.START, Start_Set);

        allowedPaths.put(ScoringPos.CORAL_STORE, Coral_Store_Set);

        allowedPaths.put(ScoringPos.INTAKE_CORAL, Coral_Intake_Set);

        allowedPaths.put(ScoringPos.INTAKE_SOURCE, Source_Intake_Set);

        allowedPaths.put(ScoringPos.INTAKE_ALGAE, Algae_Intake_Set);
        allowedPaths.put(ScoringPos.LOLIPOP_INTAKE_ALGAE, Lolipop_Intake_Set);

        allowedPaths.put(ScoringPos.ALGAE_STORE, Algae_Store_Set);

        allowedPaths.put(ScoringPos.SCORE_NET, Algae_Net_Score_Set);

        allowedPaths.put(ScoringPos.SCORE_PROCESSOR, Algae_Processor_Score_Set);

        allowedPaths.put(ScoringPos.CORAL_SCOREL1, Coral_ScoreL1_Set);
        allowedPaths.put(ScoringPos.CORAL_SCOREL2, Coral_ScoreL2_Set);
        allowedPaths.put(ScoringPos.CORAL_SCOREL3, Coral_ScoreL3_Set);
        allowedPaths.put(ScoringPos.CORAL_SCOREL4, Coral_ScoreL4_Set);

        allowedPaths.put(ScoringPos.ScoreL4, Score_L4_Set);
        allowedPaths.put(ScoringPos.SCORE_CORAL, Coral_Score_Set);

        allowedPaths.put(ScoringPos.ALGAEL1, Algae_L1_Set);
        allowedPaths.put(ScoringPos.ALGAEL2, Algae_L2_Set);

        allowedPaths.put(ScoringPos.GRABBED_ALGAE, Algae_Grabbed_Set);
        allowedPaths.put(ScoringPos.GO_SCORE_CORAL, Coral_Score_Go_Set);

        allowedPaths.put(ScoringPos.INTAKE_VERTICAL_CORAL, Coral_Intake_Vertical_Set);
        allowedPaths.put(ScoringPos.PRE_L4, Pre_L4_Set);

        allowedPaths.put(ScoringPos.L1_STORE, L1_Store_Set);
    }

    @Override
    public void periodic() {
        pitchEncoder = DW.getPitchAngle();
        rollEncoder = DW.getRollAngle();
        elbowEncoder = Elbow.getAngle();
        elevEncoder = Elev.getPosition();
        
        if (!allAtSetpoints || justChanged) {
            justChanged = false;
            updatePosition();
            recordSetpoints();
        } else if (allAtSetpoints && justFinished) {
            justFinished = false;
        }

        if (debugging.CoordPositionDebugging || debugging.CoordAllAtSetpoint || debugging.currentPos) {
            debugger();
        }
        Logger.recordOutput("Reefscape/Scoring/AutoIntakeMode", autoGround);
    }



    public void updatePosition() {

        if (pos != ScoringPos.GO_SCORE_CORAL && pos != ScoringPos.SCORE_CORAL) {
            justHitScore = true;
        }

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
        } else if(pos == ScoringPos.GO_SCORE_CORAL) {
            if (justHitScore) {
                justHitScore = false;
                level = desiredLevel;
                algaeNet = desiredAlgaeNet;
            }
            goScoreLevel();
        }else if(pos == ScoringPos.SCORE_CORAL) {
            goToScoreCoral();
        } else if(pos == ScoringPos.ScoreL4) {
            goScoreL4();
        } else if(pos == ScoringPos.ALGAEL1) {
            goL1Algae();
        } else if(pos == ScoringPos.ALGAEL2) {
            goL2Algae();
        } else if (pos == ScoringPos.SCORE_PROCESSOR) {

        } else if (pos == ScoringPos.START) {
            goToStart();
        } else if (pos == ScoringPos.GRABBED_ALGAE) {
            goToGrabbed();
        } else if (pos == ScoringPos.INTAKE_VERTICAL_CORAL) {
            goToIntakeVertical();
        } else if (pos == ScoringPos.PRE_L4) {
            goToPreL4();
        } else if (pos == ScoringPos.LOLIPOP_INTAKE_ALGAE) {
            goToLolipopAlgae();
        }

    }

    public void pitchManualMovement(double change) {

        double changeTemp = Math.abs(change);
        double setpoint = DW.getPitchSetpoint();

        //DW.setPitchSetpoint(setpoint + change);

        if (!(
            (setpoint + change  > pitchOriginalSetpoint + pitchAllowedDifference)
            || (setpoint + change < pitchOriginalSetpoint - pitchAllowedDifference)
            )) 
        {
            DW.setPitchSetpoint(setpoint + change);
        }
    }

    public void elevManualMovement(double change) {
        double changeTemp = Math.abs(change);
        double setpoint = Elev.getSetpoint();
        if (!(
            (setpoint + changeTemp > elevOriginalSetpoint + pitchAllowedDifference)
            || (setpoint - changeTemp < elevOriginalSetpoint - elevAllowedDifference)
            )) {
                Elev.setSetpoint(setpoint + change);
            }
    }

    public void recordSetpoints() {
        elevOriginalSetpoint = Elev.getSetpoint();
        elbowOriginalSetpoint = Elbow.getSetpoint();
        pitchOriginalSetpoint = DW.getPitchSetpoint();
        rollOriginalSetpoint = DW.getRollSetpoint();
    }

    public void goToStart() {
        // if (elbowEncoder > 20 && (rollEncoder > -45 && rollEncoder < 45)) {
            // Elbow.setSetpoint(19); 
        // } else {
        rollToClosestSide();
        DW.setPitchSetpoint(-120);
        Elbow.setSetpoint(90);
        // } 
        if (
            DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            ) {
                Elev.setSetpoint(0);
        }
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goToPreL4() {
        DW.setRollSetpoint(0);
        DW.setPitchSetpoint(-70);
        Elev.setSetpoint(4.5);
        Elbow.setSetpoint(67);

        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goToL1Store() {
        DW.setRollSetpoint(0);
        DW.setPitchSetpoint(-70);
        Elev.setSetpoint(0);
        Elbow.setSetpoint(67);

        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goToCoralStore() {
        setL4RollSide(false);
        algae = false;
        l4Extend = false;
        if ((desiredLevel == 4 && DualIntakeSubsystem.getInstance().hasCoral()) || lastPos == ScoringPos.INTAKE_SOURCE || DriverStation.isAutonomous()) {
            goToPreL4();
        } else if (desiredLevel == 1) {
            goToL1Store();
        } else {
            if (lastPos == ScoringPos.INTAKE_CORAL) {
                if (elbowEncoder > 15) {
                    rollToClosestSide();
                }
            } else {
                rollToClosestSide();
            }
            //DW.setRollSetpoint(0);
            Elbow.setSetpoint(90);
            if (lastPos == ScoringPos.INTAKE_VERTICAL_CORAL) {
                if (elbowEncoder > 30)
                DW.setPitchSetpoint(-150);

            } else {
                DW.setPitchSetpoint(-150);
            }
            Elev.setSetpoint(0);
        }

        
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goToSourceIntake() {
        Elev.setSetpoint(5.9);
        DW.setPitchSetpoint(-135);
        Elbow.setSetpoint(100);
        DW.setRollSetpoint(0);
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goToAlgaeStore() {
        // DW.setPitchSetpoint(-152);
        // DW.setRollSetpoint(0);
        // Elbow.setSetpoint(75);

        algae = true;
        DW.setPitchSetpoint(-70);
        Elbow.setSetpoint(56);
        Elev.setSetpoint(3);
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }
 
    public void goToCoralIntake() {
        DW.setPitchSetpoint(Constants.robotConfig.getPitchGroundPos()); // OLD: -129
        DW.setRollSetpoint(0); 
        Elbow.setSetpoint(Constants.robotConfig.getElbowGroundPos()); // Old: 2
        Elev.setSetpoint(0);
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    //elbow 28, pitch -80, roll 180
    public void goToAlgaeIntake() {
        algae = true;
        DW.setPitchSetpoint(-100);
        Elbow.setSetpoint(-5);
        Elev.setSetpoint(0);
        DW.setRollSetpoint(0);


        // DW.setPitchSetpoint(-129);
        // DW.setRollSetpoint(0);
        // Elbow.setSetpoint(0);
        // Elev.setSetpoint(0);
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goToLolipopAlgae() {
        algae = true;
        DW.setPitchSetpoint(-100);
        Elbow.setSetpoint(20);
        Elev.setSetpoint(0);
        DW.setRollSetpoint(0);


        // DW.setPitchSetpoint(-129);
        // DW.setRollSetpoint(0);
        // Elbow.setSetpoint(0);
        // Elev.setSetpoint(0);
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    /**
     * [Insert good comments here]
     */
    public void goToScoreNet() {
        Elev.setSetpoint(38);
        if (elevEncoder > 8) {
            DW.setPitchSetpoint(-172);
            DW.setRollSetpoint(0);
            Elbow.setSetpoint(76);
        }
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goToScoreProcessor() {
        DW.setPitchSetpoint(-125.5);
        DW.setRollSetpoint(0);
        Elbow.setSetpoint(-17);
        if (DW.atPitchSetpoint() && DW.atRollSetpoint() && Elbow.atSetpoint()) {
            Elev.setSetpoint(0);
        }
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goToIntakeVertical() {
        DW.setPitchSetpoint(-65);
        rollToClosestSide();
        Elbow.setSetpoint(-28);
        Elev.setSetpoint(0);
        
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goScoreLevel() {
        if (algae) {
            //Elev.setSlow();
            if (algaeNet) { // net
                coralScorePitch = -130; //-145
                coralScoreElbow = 76;
                coralScoreElev = Constants.robotConfig.getElevatorNetPos();
                DW.setRollSetpoint(0);
            } else { // processor
                coralScorePitch = -90;
                coralScoreElbow = 15;
                coralScoreElev = 0;
                DW.setRollSetpoint(0);
            }
        } else {
            switch (level) {
                case 1:
                    coralScorePitch = -175;
                    coralScoreElbow = 62;
                    coralScoreElev = 3;
                    DW.setRollSetpoint(0);
                    break;
                case 2:
                    coralScorePitch = -112;
                    coralScoreElbow = 78;
                    if (l4Extend) {
                        coralScoreElev = 5.5;
                    } else {
                        coralScoreElev = 4.5; //Comp: 3.75
                    }
                    //rollToClosestSide();
                    break;
                case 3:
                    coralScorePitch = -112;
                    coralScoreElbow = 78;
                    if (l4Extend) {
                        coralScoreElev = 14.25;
                    } else {
                        coralScoreElev = 13.25;
                    }
                    //rollToClosestSide();
                    break;
                case 4:
                    if (l4Extend) {
                        coralScorePitch = -132;
                        coralScoreElbow = 37;
                        coralScoreElev = 36;
                    } else {
                        coralScorePitch = Constants.robotConfig.getL4Pitch();
                        coralScoreElbow = Constants.robotConfig.getL4Elbow();
                        coralScoreElev = Constants.robotConfig.getL4Elevator();
                    }
                    break;
            }
        }

        // elev 0
        // pitch -71
        // elbow -22

        if(algae) {
            if (algaeNet) {
                Elbow.setSetpoint(coralScoreElbow);
                Elev.setSetpoint(coralScoreElev);
                DW.setPitchSetpoint(-110); //old pitch pos used while going up
                if (Elev.getPosition() > 35) {
                    DW.setPitchSetpoint(coralScorePitch); //pitch pos wanted when scoring
                }
            } else {
                DW.setPitchSetpoint(coralScorePitch);
                Elbow.setSetpoint(coralScoreElbow);
                
                if(DW.atPitchSetpoint()) {
                    Elev.setSetpoint(coralScoreElev);
                }
            }
        } else if (level == 1) {
            DW.setPitchSetpoint(coralScorePitch);
            Elbow.setSetpoint(coralScoreElbow);

            if (DW.atPitchSetpoint() && Elbow.atSetpoint()) {
                Elev.setSetpoint(coralScoreElev);
            }
        } else if (level == 4) {
            Elev.setSetpoint(coralScoreElev);
            if (elevEncoder > 25) {
                DW.setPitchSetpoint(coralScorePitch);
                rollToL4();
                Elbow.setSetpoint(coralScoreElbow);
            }
        } else if (level == 3 || level == 2) {
            DW.setPitchSetpoint(coralScorePitch);
            Elbow.setSetpoint(coralScoreElbow);
            Elev.setSetpoint(coralScoreElev);
            if (DW.atPitchSetpoint() && Elbow.atSetpoint()) {
                rollToClosestSide();
            }
        } else {
            DW.setPitchSetpoint(coralScorePitch);
            Elbow.setSetpoint(coralScoreElbow);
            Elev.setSetpoint(coralScoreElev);
        }
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            && Elev.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void setCoralInFront(boolean coral) {
        l4Extend = coral;
        if (pos == ScoringPos.GO_SCORE_CORAL) {
            justChanged = true;
        }
    }

    public void toggleCoralInFront() {
        setCoralInFront(!l4Extend);
    }


    public void goScoreL4() {
        Elev.setSetpoint(21);
    }

    public void goL1Algae() {
        algae = true;
        Elev.setSetpoint(11);
        DW.setPitchSetpoint(-110);
        Elbow.setSetpoint(37.09);
        DW.setRollSetpoint(0);
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goL2Algae() {
        algae = true;
        Elev.setSetpoint(20);
        DW.setPitchSetpoint(-110);
        Elbow.setSetpoint(37.09);
        DW.setRollSetpoint(0);
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goToScoreCoral() {
        DW.setPitchSetpoint(-107.19);
        Elbow.setSetpoint(32.91);
        if (level == 3 && Elbow.atSetpoint()) {
            Elev.setSetpoint(11.5);
        }
        if (level == 2 && Elbow.atSetpoint()) {

        }
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            && Elev.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    public void goToGrabbed() {
        Elev.setSetpoint(elbowOriginalSetpoint);
        if (DW.atRollSetpoint()
            && DW.atPitchSetpoint()
            && Elbow.atSetpoint()
            )
        {
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    /* ----- Setters and Getters ----- */

    public void setPosition(ScoringPos pos) {
        lastPos = this.pos;
        this.pos = pos;
        justHitScore = true;
        justChanged = true;
        allAtSetpoints = false;
    }

    private void rollToL4() {
        if (l4RollLeft) {
            DW.setRollSetpoint(90);
            coralSideLeft = true;
        } else {
            DW.setRollSetpoint(-94);
            coralSideLeft = false;
        }
    }

    boolean l4RollLeft;

    public void setL4RollSide(boolean left) {
        l4RollLeft = left;
    }

    public void rollToClosestSide() {
        if (rollEncoder <= -5) {
            DW.setRollSetpoint(-94);
            coralSideLeft = false;
        } else if (rollEncoder > 5) {
            DW.setRollSetpoint(90);
            coralSideLeft = true;
        } else {
            DW.setRollSetpoint(-94);
            coralSideLeft = false;
        }
    }

    public void rollToOtherSide() {
        if (coralSideLeft) {
            DW.setRollSetpoint(-94);
            coralSideLeft = false;
        } else if (!coralSideLeft) {
            DW.setRollSetpoint(90);
            coralSideLeft = true;
        }
    }

    public static CoordinationSubsytem getInstance() {
        if (CT == null) {
            CT = new CoordinationSubsytem();
        }
        return CT;
    }

    public ScoringPos getPos() {
        return pos;
    }

    public boolean getAllAtSetpoints() {
        return allAtSetpoints;
    }

    public int getScoringLevel() {
        return level;
    }

    public int getDesiredLevel() {
        return desiredLevel;
    }

    public void setScoringLevel(int level) {
        desiredLevel = level;
        if (pos == ScoringPos.CORAL_STORE) {
            justChanged = true;
        }
    }

    public void setAlgaeNet(boolean net) {
        desiredAlgaeNet = net;
    }

    public boolean getAlgaeNet() {
        return algaeNet;
    }

    public boolean getAlgae() {
        return algae;
    }

    /* ----------- DEBUGGING ----------- */

    /**
     * Debugger
     */
    public void debugger() {

        if (debugging.CoordPositionDebugging) {
            Logger.recordOutput("Reefscape/Debugging/AtSetpoint?/Elbow atSetpoint?", Elbow.atSetpoint());
            Logger.recordOutput("Reefscape/Debugging/AtSetpoint?/Pitch atSetpoint?", DW.atPitchSetpoint());
            Logger.recordOutput("Reefscape/Debugging/AtSetpoint?/Roll atSetpoint?", DW.atRollSetpoint());
            Logger.recordOutput("Reefscape/Debugging/AtSetpoint?/Elevator atSetpoint?", Elev.atSetpoint());
        }
        if (debugging.CoordAllAtSetpoint) {
            Logger.recordOutput("Reefscape/Debugging/All At Setpoints", allAtSetpoints);
        }

        if (debugging.currentPos) {
            Logger.recordOutput("Reefscape/Debugging/Current Position", getPos());
        }

    }
}
