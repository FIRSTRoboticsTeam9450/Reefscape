package frc.robot.subsystems;

import java.lang.Thread.State;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class CoordinationSubsytem extends SubsystemBase{

    public static boolean autoGround = false;

    /* ----- Subsystem Instances ----- */
    private static CoordinationSubsytem CT;
    private DiffWristSubsystem DW = DiffWristSubsystem.getInstance();
    private ElevatorSubsystem Elev = ElevatorSubsystem.getInstance();
    private ElbowSubsystem Elbow = ElbowSubsystem.getInstance();
    //private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    /* ----- Encoders ----- */
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
    private Set<ScoringPos> Algae_L3_Set = new HashSet<>();
    private Set<ScoringPos> ALGAE_INTAKE_REEF_DYNAMIC_Set = new HashSet<>();
    private Set<ScoringPos> Algae_Grabbed_Set = new HashSet<>();
    private Set<ScoringPos> Coral_Score_Go_Set = new HashSet<>();
    private Set<ScoringPos> Coral_Intake_Vertical_Set = new HashSet<>();
    private Set<ScoringPos> CORAL_PRE_L4_Set = new HashSet<>();

    public Map<ScoringPos, Pair<double[], boolean[]>> standardPosMap = new HashMap<>();
    
    boolean algae;

    private int level = 1;
    private int desiredLevel = 1;

    private boolean algaeNet = true;
    private boolean desiredAlgaeNet = true;

    private boolean justHitScore = true;

    private boolean allAtSetpoints = false;
    private boolean justFinished = false;
    private boolean justChanged = false;

    private double justACoupleMore = 0;

    private double elevOriginalSetpoint;
    private double elbowOriginalSetpoint;
    private double pitchOriginalSetpoint;

    private double elevAllowedDifference = 1.5;
    private double pitchAllowedDifference = 12;

    private boolean coralSideLeft;
    private boolean l4Extend;

    private double coralScoreElbow = 0;
    private double coralScoreElev = 0;
    private double coralScorePitch = 0;

    ScoringPos lastPos = ScoringPos.START;

    private boolean onlyOnce;
    private boolean combinedAlgae;

    private int tid;

    /**
     * gets the starting angle / position of the encoders
     */
    private CoordinationSubsytem() {
        onlyOnce = false;
        combinedAlgae = false;
        autoGround = false;
        pos = ScoringPos.START;

        rollEncoder = DW.getRollAngle();
        elbowEncoder = Elbow.getAngle();
        elevEncoder = Elev.getPosition();

        Start_Set.add(ScoringPos.CORAL_STORE);
        Start_Set.add(ScoringPos.GO_TO_SCORE);
        
        Coral_Store_Set.add(ScoringPos.CORAL_PRE_L4);

        Coral_Store_Set.add(ScoringPos.START);
        Coral_Store_Set.add(ScoringPos.CORAL_INTAKE_GROUND);
        Coral_Store_Set.add(ScoringPos.ALGAE_INTAKE_GROUND);
        Coral_Store_Set.add(ScoringPos.CORAL_INTAKE_VERTICAL);
        Coral_Store_Set.add(ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC);
        Coral_Store_Set.add(ScoringPos.ALGAE_INTAKE_REEF_LOW);
        Coral_Store_Set.add(ScoringPos.ALGAE_INTAKE_REEF_HIGH);
        Coral_Store_Set.add(ScoringPos.ALGAE_INTAKE_PROC);
        Coral_Store_Set.add(ScoringPos.GO_TO_SCORE);

        Coral_Store_Set.add(ScoringPos.CORAL_STORE);

        Coral_Intake_Set.add(ScoringPos.START);
        Coral_Intake_Set.add(ScoringPos.CORAL_STORE);
        Coral_Intake_Set.add(ScoringPos.ALGAE_INTAKE_REEF_LOW);
        Coral_Intake_Set.add(ScoringPos.ALGAE_INTAKE_REEF_HIGH);
        Coral_Intake_Set.add(ScoringPos.ALGAE_INTAKE_PROC);
        Coral_Intake_Set.add(ScoringPos.ALGAE_INTAKE_GROUND);


        Source_Intake_Set.add(ScoringPos.CORAL_STORE);
        Source_Intake_Set.add(ScoringPos.CORAL_INTAKE_GROUND);

        Algae_Intake_Set.add(ScoringPos.CORAL_STORE);
        Algae_Intake_Set.add(ScoringPos.ALGAE_STORE);
        Algae_Intake_Set.add(ScoringPos.ALGAE_INTAKE_REEF_LOW);
        Algae_Intake_Set.add(ScoringPos.ALGAE_INTAKE_REEF_HIGH);
        Algae_Intake_Set.add(ScoringPos.ALGAE_INTAKE_PROC);
        Algae_Intake_Set.add(ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC);
        Algae_Intake_Set.add(ScoringPos.CORAL_INTAKE_GROUND);
        Algae_Intake_Set.add(ScoringPos.GO_TO_SCORE);

        Algae_Store_Set.add(ScoringPos.ALGAE_INTAKE_GROUND); //temp... maybe
        Algae_Store_Set.add(ScoringPos.CORAL_STORE); //temp
        Algae_Store_Set.add(ScoringPos.ALGAE_INTAKE_REEF_LOW);
        Algae_Store_Set.add(ScoringPos.ALGAE_INTAKE_REEF_HIGH);
        Algae_Store_Set.add(ScoringPos.ALGAE_INTAKE_PROC);
        Algae_Store_Set.add(ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC);
        Algae_Store_Set.add(ScoringPos.GO_TO_SCORE);
        
        Coral_ScoreL1_Set.add(ScoringPos.CORAL_STORE);
        Coral_ScoreL2_Set.add(ScoringPos.CORAL_STORE);
        Coral_ScoreL3_Set.add(ScoringPos.CORAL_STORE);
        Coral_ScoreL4_Set.add(ScoringPos.CORAL_STORE);

        Coral_ScoreL1_Set.add(ScoringPos.CORAL_SCORE);
        Coral_ScoreL2_Set.add(ScoringPos.CORAL_SCORE);
        Coral_ScoreL3_Set.add(ScoringPos.CORAL_SCORE);
        Coral_ScoreL4_Set.add(ScoringPos.CORAL_SCORE_L4);

        Coral_ScoreL1_Set.add(ScoringPos.GO_TO_SCORE);
        Coral_ScoreL2_Set.add(ScoringPos.GO_TO_SCORE);
        Coral_ScoreL3_Set.add(ScoringPos.GO_TO_SCORE);
        Coral_ScoreL4_Set.add(ScoringPos.GO_TO_SCORE);

        Coral_Score_Set.add(ScoringPos.CORAL_STORE);
        Coral_Score_Set.add(ScoringPos.GO_TO_SCORE);
        Coral_Score_Set.add(ScoringPos.CORAL_INTAKE_GROUND);

        Algae_Net_Score_Set.add(ScoringPos.CORAL_STORE);

        Algae_Processor_Score_Set.add(ScoringPos.CORAL_STORE);

        Score_L4_Set.add(ScoringPos.CORAL_STORE);
        Score_L4_Set.add(ScoringPos.GO_TO_SCORE);
        Score_L4_Set.add(ScoringPos.CORAL_INTAKE_GROUND);

        Algae_L1_Set.add(ScoringPos.ALGAE_STORE);
        Algae_L1_Set.add(ScoringPos.CORAL_STORE);
        Algae_L1_Set.add(ScoringPos.ALGAE_INTAKE_GROUND);
        Algae_L1_Set.add(ScoringPos.ALGAE_INTAKE_REEF_HIGH);
        Algae_L1_Set.add(ScoringPos.ALGAE_INTAKE_PROC);
        Algae_L1_Set.add(ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC);
        Algae_L1_Set.add(ScoringPos.GO_TO_SCORE);
        Algae_L1_Set.add(ScoringPos.CORAL_INTAKE_GROUND);

        Algae_L2_Set.add(ScoringPos.ALGAE_STORE);
        Algae_L2_Set.add(ScoringPos.CORAL_STORE);
        Algae_L2_Set.add(ScoringPos.ALGAE_INTAKE_GROUND);
        Algae_L2_Set.add(ScoringPos.ALGAE_INTAKE_REEF_LOW);
        Algae_L2_Set.add(ScoringPos.ALGAE_INTAKE_PROC);
        Algae_L2_Set.add(ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC);
        Algae_L2_Set.add(ScoringPos.GO_TO_SCORE);
        Algae_L2_Set.add(ScoringPos.CORAL_INTAKE_GROUND);

        Algae_L3_Set.add(ScoringPos.ALGAE_STORE);
        Algae_L3_Set.add(ScoringPos.CORAL_STORE);
        Algae_L3_Set.add(ScoringPos.ALGAE_INTAKE_GROUND);
        Algae_L3_Set.add(ScoringPos.ALGAE_INTAKE_REEF_LOW);
        Algae_L3_Set.add(ScoringPos.ALGAE_INTAKE_REEF_HIGH);
        Algae_L3_Set.add(ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC);
        Algae_L3_Set.add(ScoringPos.GO_TO_SCORE);
        Algae_L3_Set.add(ScoringPos.CORAL_INTAKE_GROUND);

        ALGAE_INTAKE_REEF_DYNAMIC_Set.add(ScoringPos.ALGAE_STORE);
        ALGAE_INTAKE_REEF_DYNAMIC_Set.add(ScoringPos.CORAL_STORE);
        ALGAE_INTAKE_REEF_DYNAMIC_Set.add(ScoringPos.ALGAE_INTAKE_GROUND);
        ALGAE_INTAKE_REEF_DYNAMIC_Set.add(ScoringPos.ALGAE_INTAKE_REEF_LOW);
        ALGAE_INTAKE_REEF_DYNAMIC_Set.add(ScoringPos.ALGAE_INTAKE_REEF_HIGH);
        ALGAE_INTAKE_REEF_DYNAMIC_Set.add(ScoringPos.ALGAE_INTAKE_PROC);
        ALGAE_INTAKE_REEF_DYNAMIC_Set.add(ScoringPos.GO_TO_SCORE);
        ALGAE_INTAKE_REEF_DYNAMIC_Set.add(ScoringPos.CORAL_INTAKE_GROUND);

        Coral_Score_Go_Set.add(ScoringPos.CORAL_STORE);
        Coral_Score_Go_Set.add(ScoringPos.CORAL_SCORE);
        Coral_Score_Go_Set.add(ScoringPos.GO_TO_SCORE);
        Coral_Score_Go_Set.add(ScoringPos.CORAL_SCORE_L4);
        Coral_Score_Go_Set.add(ScoringPos.ALGAE_STORE);

        Algae_Grabbed_Set.add(ScoringPos.ALGAE_STORE);
        Algae_Grabbed_Set.add(ScoringPos.CORAL_STORE);
        Algae_Grabbed_Set.add(ScoringPos.ALGAE_INTAKE_REEF_LOW);
        Algae_Grabbed_Set.add(ScoringPos.ALGAE_INTAKE_REEF_HIGH);
        Algae_Grabbed_Set.add(ScoringPos.ALGAE_INTAKE_PROC);
        Algae_Grabbed_Set.add(ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC);
        Algae_Grabbed_Set.add(ScoringPos.ALGAE_INTAKE_GROUND);
        Algae_Grabbed_Set.add(ScoringPos.GO_TO_SCORE);

        Coral_Intake_Vertical_Set.add(ScoringPos.CORAL_STORE);
        Coral_Intake_Vertical_Set.add(ScoringPos.START);

        CORAL_PRE_L4_Set.add(ScoringPos.CORAL_STORE);
        CORAL_PRE_L4_Set.add(ScoringPos.GO_TO_SCORE);
        CORAL_PRE_L4_Set.add(ScoringPos.CORAL_INTAKE_GROUND);

        allowedPaths.put(ScoringPos.START, Start_Set);

        allowedPaths.put(ScoringPos.CORAL_STORE, Coral_Store_Set);

        allowedPaths.put(ScoringPos.CORAL_INTAKE_GROUND, Coral_Intake_Set);

        allowedPaths.put(ScoringPos.ALGAE_INTAKE_GROUND, Algae_Intake_Set);

        allowedPaths.put(ScoringPos.ALGAE_STORE, Algae_Store_Set);


        allowedPaths.put(ScoringPos.CORAL_SCORE_L4, Score_L4_Set);
        allowedPaths.put(ScoringPos.CORAL_SCORE, Coral_Score_Set);

        allowedPaths.put(ScoringPos.ALGAE_INTAKE_REEF_LOW, Algae_L1_Set);
        allowedPaths.put(ScoringPos.ALGAE_INTAKE_REEF_HIGH, Algae_L2_Set);
        allowedPaths.put(ScoringPos.ALGAE_INTAKE_PROC, Algae_L3_Set);
        allowedPaths.put(ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC, ALGAE_INTAKE_REEF_DYNAMIC_Set);

        allowedPaths.put(ScoringPos.GO_TO_SCORE, Coral_Score_Go_Set);

        allowedPaths.put(ScoringPos.CORAL_INTAKE_VERTICAL, Coral_Intake_Vertical_Set);
        allowedPaths.put(ScoringPos.CORAL_PRE_L4, CORAL_PRE_L4_Set);

        standardPosMap.put(ScoringPos.CORAL_INTAKE_GROUND, StatePositions.CORAL_INTAKE_GROUND_PAIR);
        standardPosMap.put(ScoringPos.CORAL_INTAKE_VERTICAL, StatePositions.CORAL_INTAKE_VERTICAL_PAIR);
        standardPosMap.put(ScoringPos.ALGAE_STORE, StatePositions.ALGAE_STORE_PAIR);
        standardPosMap.put(ScoringPos.ALGAE_INTAKE_GROUND, StatePositions.ALGAE_INTAKE_GROUND_PAIR);
        standardPosMap.put(ScoringPos.ALGAE_INTAKE_REEF_LOW, StatePositions.ALGAE_INTAKE_REEF_LOW_PAIR);
        standardPosMap.put(ScoringPos.ALGAE_INTAKE_REEF_HIGH, StatePositions.ALGAE_INTAKE_REEF_HIGH_PAIR);
        standardPosMap.put(ScoringPos.ALGAE_INTAKE_PROC, StatePositions.ALGAE_INTAKE_PROC_PAIR);

    }

    @SuppressWarnings("unused")
    @Override
    public void periodic() {

        if (DriverStation.isAutonomous()) {
            autoGround = false;
            onlyOnce = false;
        } else if(DriverStation.isTeleop() && !onlyOnce) {
            autoGround = true;
            onlyOnce = true;
            desiredLevel = 1;
            level = 1;
            setScoringLevel(1);
        }

        combinedAlgae = (pos == ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC);

        tid = (int)LimelightHelpers.getFiducialID("limelight-coral");

        rollEncoder = DW.getRollAngle();
        elbowEncoder = Elbow.getAngle();
        elevEncoder = Elev.getPosition();
        
        if (!allAtSetpoints || justChanged || combinedAlgae || justACoupleMore < 5) {
            justChanged = false;

            if (allAtSetpoints && !justChanged) {
                justACoupleMore++;
            }
            
            updatePositionState();
            recordSetpoints();
        } else if (allAtSetpoints && justFinished) {
            justFinished = false;
        }

        if (debugging.CoordPositionDebugging || debugging.CoordAllAtSetpoint) {
            debugger();
        }
        Logger.recordOutput("Reefscape/Scoring/AutoIntakeMode", autoGround);
        Logger.recordOutput("Reefscape/Scoring/State", pos);
    }
    

    /**
     * Uses a switch-case to determine which method to call
     */
    private void updatePositionState() {
        
        justHitScore = (pos != ScoringPos.GO_TO_SCORE && pos != ScoringPos.CORAL_SCORE);

        switch (pos) {

            case START:
                startPos();
                break;
            
            case GO_TO_SCORE:
                goToScorePos();
                break;

            case CORAL_STORE:
                coralStorePos();
                break;

            case CORAL_INTAKE_GROUND:
                coralIntakeGroundPos();
                break;

            case CORAL_INTAKE_VERTICAL:
                coralIntakeVerticalPos();
                break;

            case CORAL_SCORE:
                coralScorePos();
                break;

            case CORAL_SCORE_L4:
                coralScoreL4Pos();
                break;

            case CORAL_PRE_L4:
                coralPreL4Pos();
                break;

            case ALGAE_STORE:
                algaeStorePos();
                break;

            case ALGAE_INTAKE_GROUND:
                algaeIntakeGroundPos();
                break;

            case ALGAE_INTAKE_REEF_LOW:
                algaeIntakeReefLowPos();
                break;

            case ALGAE_INTAKE_REEF_HIGH:
                algaeIntakeReefHighPos();
                break;

            case ALGAE_INTAKE_REEF_DYNAMIC:
                algaeIntakeReefDynamicPos();
                break;

            case ALGAE_INTAKE_PROC:
                algaeIntakeProcPos();
                break;

            default:
                coralStorePos();
                break;

        }
    }

    private void standardUpdatePositionState() {
         
        justHitScore = (pos != ScoringPos.GO_TO_SCORE && pos != ScoringPos.CORAL_SCORE);

        switch (pos) {

            case START:
                startPos();
                break;
            
            case GO_TO_SCORE:
                goToScorePos();
                break;

            case CORAL_STORE:
                coralStorePos();
                break;

            case CORAL_SCORE:
                coralScorePos();
                break;

            case CORAL_SCORE_L4:
                coralScoreL4Pos();
                break;

            case CORAL_PRE_L4:
                coralPreL4Pos();
                break;

            case ALGAE_INTAKE_REEF_DYNAMIC:
                algaeIntakeReefDynamicPos();
                break;

            default:
                standardPos();
                break;

        }
    }

    public void pitchManualMovement(double change) {

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
    }

    /**
     * State the robot starts in upon match start, code deploy, or robot boot up
     */
    private void startPos() {

        rollToClosestSide();
        DW.setPitchSetpoint(-120);
        Elbow.setSetpoint(90);

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

    /**
     * Dynamic Scoring state based of desired scoring area
     */
    private void goToScorePos() {

        if (algae) {
            if (algaeNet) {
                coralScorePitch = -130;
                coralScoreElbow = 76;
                coralScoreElev = Constants.RobotConstants.robotConfig.getElevatorNetPos();
                DW.setRollSetpoint(0);
            } else {
                coralScorePitch = -90;
                coralScoreElbow = 15;
                coralScoreElev = 0;
                DW.setRollSetpoint(0);
            }
        } else {
            switch (desiredLevel) {
                case 0:
                    coralScoreElev = 3.75;
                    coralScorePitch = -130;
                    coralScoreElbow = 35;
                    DW.setRollSetpoint(0);
                    break;
                case 1:
                    coralScorePitch = -184;
                    coralScoreElbow = 60;
                    coralScoreElev = 3;
                    DW.setRollSetpoint(0);
                    break;
                case 2:
                    coralScorePitch = -115;
                    coralScoreElbow = 78;
                    if (l4Extend) {
                        coralScoreElev = 5.5;
                    } else {
                        coralScoreElev = 4;
                    }
                    break;
                case 3:
                    coralScorePitch = -112;
                    coralScoreElbow = 78;
                    if (l4Extend) {
                        coralScoreElev = 13.5;
                    } else {
                        coralScoreElev = 12.5;
                    }
                    break;
                case 4:
                    if (l4Extend) {
                        coralScorePitch = -132;
                        coralScoreElbow = 37;
                        coralScoreElev = 36;
                    } else {
                        coralScorePitch = Constants.RobotConstants.robotConfig.getL4Pitch();
                        coralScoreElbow = Constants.RobotConstants.robotConfig.getL4Elbow();
                        coralScoreElev = Constants.RobotConstants.robotConfig.getL4Elevator();
                    }
                    break;
            }
        }



        if(algae) {
            if (algaeNet) {
                Elbow.setSetpoint(coralScoreElbow);
                Elev.setSetpoint(coralScoreElev);
                DW.setPitchSetpoint(-110); 
                if (Elev.getPosition() > 35) {
                    DW.setPitchSetpoint(coralScorePitch);
                }
            } else {
                DW.setPitchSetpoint(coralScorePitch);
                Elbow.setSetpoint(coralScoreElbow);
                
                if(DW.atPitchSetpoint()) {
                    Elev.setSetpoint(coralScoreElev);
                }
            }
        } else {
            switch (desiredLevel) {
                case 0:
                    DW.setPitchSetpoint(coralScorePitch); 
                    Elbow.setSetpoint(coralScoreElbow);
                    if (DW.atPitchSetpoint() && Elbow.atSetpoint()) Elev.setSetpoint(coralScoreElev);
                    break;

                case 1:
                    DW.setPitchSetpoint(coralScorePitch); 
                    Elbow.setSetpoint(coralScoreElbow);
                    if (DW.atPitchSetpoint() && Elbow.atSetpoint()) Elev.setSetpoint(coralScoreElev);
                    break;

                case 2:
                    DW.setPitchSetpoint(coralScorePitch);
                    Elbow.setSetpoint(coralScoreElbow);
                    Elev.setSetpoint(coralScoreElev);
                    if (DW.atPitchSetpoint() && Elbow.atSetpoint()) rollToClosestSide();
                    break;

                case 3:
                    DW.setPitchSetpoint(coralScorePitch);
                    Elbow.setSetpoint(coralScoreElbow);
                    Elev.setSetpoint(coralScoreElev);
                    if (DW.atPitchSetpoint() && Elbow.atSetpoint()) rollToClosestSide();
                    break;

                case 4:
                    Elev.setSetpoint(coralScoreElev);
                    if (elevEncoder > 25) {
                        DW.setPitchSetpoint(coralScorePitch);
                        rollToL4();
                        Elbow.setSetpoint(coralScoreElbow);
                    }
                    break;

                default:
                    DW.setPitchSetpoint(coralScorePitch);
                    Elbow.setSetpoint(coralScoreElbow);
                    Elev.setSetpoint(coralScoreElev);
            }
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

    /**
     * Default/idle state
     */
    private void coralStorePos() {
        
        setL4RollSide(false);
        algae = false;
        l4Extend = false;

        if (desiredLevel == 4 && DualIntakeSubsystem.getInstance().hasCoral) {
            coralPreL4Pos();
        } else if (desiredLevel == 0 || desiredLevel == 1) {
            if (lastPos == ScoringPos.CORAL_INTAKE_GROUND) {
                goToScorePos();
            } else {
                DW.setRollSetpoint(0);
                DW.setPitchSetpoint(-70);
                Elev.setSetpoint(0);
                Elbow.setSetpoint(67);
            }
        } else {
            if (lastPos == ScoringPos.CORAL_INTAKE_GROUND) {
                if (elbowEncoder > 25) {
                    if (DriverStation.isAutonomous()) {
                        rollToOtherSide();
                    } else {
                        rollToClosestSide();
                    }
                }
            } else {
                rollToClosestSide();
            }
            Elbow.setSetpoint(90);
            if (lastPos == ScoringPos.CORAL_INTAKE_VERTICAL) {
                if (DriverStation.isAutonomous()) {
                    // rollToOtherSide();
                }
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

    /**
     * State used for intake coral off the ground
     * Note for self: see if tieing things like intake into a state works
     *      - Could use a dynamic system like experimental keybinds
     */
    private void coralIntakeGroundPos() {

        DW.setPitchSetpoint(Constants.RobotConstants.robotConfig.getPitchGroundPos()); // OLD: -129
        DW.setRollSetpoint(0); 
        Elbow.setSetpoint(Constants.RobotConstants.robotConfig.getElbowGroundPos()); // Old: 2
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

    private void coralIntakeVerticalPos() {

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

    private void coralScorePos() {

        if (desiredLevel == 2) {
            DW.setPitchSetpoint(-128);
        } else {
            DW.setPitchSetpoint(-107.19);
        }

        Elbow.setSetpoint(32.91);

        if (desiredLevel == 3 && Elbow.atSetpoint()) {
            Elev.setSetpoint(11.15);
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

    private void coralScoreL4Pos() {

        Elev.setSetpoint(21);

        if (Elev.atSetpoint()){
            allAtSetpoints = true;
            justFinished = true;
        }
    }

    private void coralPreL4Pos() {

        DW.setRollSetpoint(0);
        DW.setPitchSetpoint(-70);

        if (!DriverStation.isAutonomous()) {
            Elev.setSetpoint(4.5);
        } else {
            Elev.setSetpoint(0);
        }

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

    private void algaeStorePos() {

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

    private void algaeIntakeGroundPos() {

        algae = true;

        DW.setPitchSetpoint(-100.7);
        Elbow.setSetpoint(-11.68);
        Elev.setSetpoint(0);
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

    private void algaeIntakeReefLowPos() {

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

    private void algaeIntakeReefHighPos() {
        
        algae = true;

        Elev.setSetpoint(20);
        DW.setPitchSetpoint(-90);
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

    private void algaeIntakeReefDynamicPos() {

        algae = true;

        Elbow.setSetpoint(37.09);
        DW.setPitchSetpoint(-110);
        DW.setRollSetpoint(0);

        if(Elbow.atSetpoint()) {
            if (tid == 7 || tid == 9 || tid == 11 || tid == 18 || tid == 20 || tid == 22) {
                Elev.setSetpoint(20);
            } else if (tid == 6 || tid == 8 || tid == 10 || tid == 17 || tid == 19 || tid == 21) {
                Elev.setSetpoint(11);
            }
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

    private void algaeIntakeProcPos() {

        algae = true;

        Elev.setSetpoint(29);
        DW.setPitchSetpoint(-90);
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

    private void standardPos() {

        Pair<double[], boolean[]> tmpPair = standardPosMap.get(getPos());

        double[] subsystemSetpoints = tmpPair.getFirst();
        boolean[] positionBooleans = tmpPair.getSecond();

        algae = positionBooleans[0];

        Elev.setSetpoint(subsystemSetpoints[0]);
        Elbow.setSetpoint(subsystemSetpoints[1]);
        DW.setPitchSetpoint(subsystemSetpoints[2]);

        if (positionBooleans[1]) {
            rollToClosestSide();
        } else {
            DW.setRollSetpoint(subsystemSetpoints[3]);
        }

        if (Elbow.atSetpoint()
            && DW.atPitchSetpoint()
            && DW.atRollSetpoint()
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
        justACoupleMore = 0;
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

    public void checkAllAtSetpoints() {
        if (Elev.atSetpoint()
            && Elbow.atSetpoint()
            && DW.atPitchSetpoint()
            && DW.atRollSetpoint()
            ) {
                allAtSetpoints = true;
            } else {
                allAtSetpoints = false;
            }
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

    public void toggleCoralInFront() {
        setCoralInFront(!l4Extend);
    }

    public void setCoralInFront(boolean coral) {
        l4Extend = coral;
        if (pos == ScoringPos.GO_TO_SCORE) {
            justChanged = true;
        }
    }

    public void setAlgaeNet(boolean net) {
        desiredAlgaeNet = net;
    }

    public boolean getDesiredAlgaeNet() {
        return desiredAlgaeNet;
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
            Logger.recordOutput("Reefscape/Debugging/Position/Desired Level", getDesiredLevel());
        }
        if (debugging.CoordAllAtSetpoint) {
            Logger.recordOutput("Reefscape/Debugging/All At Setpoints", allAtSetpoints);
        }

        if (debugging.CurrentPos) {
            Logger.recordOutput("Reefscape/Debugging/Current Position", getPos());
        }

    }
}