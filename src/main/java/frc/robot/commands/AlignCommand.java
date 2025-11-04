package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.RobotConstants.*;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;

/**
 * Uses April Tags to understand where it is and to align with primary april tag with certain offsets depending on which reef pole is choosen.
 */
public class AlignCommand extends Command {

    /* ----- April Tag ID - Positions ----- */
    private HashMap<Integer, double[]> aprilTagLocationMap = new HashMap<>();

    /* ----- PIDs ----- */
    private PIDController FpidX = new PIDController(4.5, 0, 0);
    private PIDController FpidY = new PIDController(4.5, 0, 0);
    private PIDController FpidRotate = new PIDController(5, 0, 0.5);

    private PIDController BpidX = new PIDController(4, 0, 0);
    private PIDController BpidY = new PIDController(3.25, 0, 0.6);
    private PIDController BpidRotate = new PIDController(7, 0, 0);

    /* ----- Subsystem Instances ----- */
    private CommandSwerveDrivetrain drive;
    private CoordinationSubsytem score = CoordinationSubsytem.getInstance();
    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    private SequentialCommandGroup L1ScoreAndWait = new SequentialCommandGroup(new WaitCommand(0.1).andThen(new ScoringCommand()));
    private SequentialCommandGroup L3UpAndWait = new SequentialCommandGroup(new WaitCommand(0.15)).andThen(new CoordinationCommand(ScoringPos.GO_TO_SCORE));
    private SequentialCommandGroup L3ScoreAndWait = new SequentialCommandGroup(new WaitCommand(0.25).andThen(new ScoringCommand()));

    /* ----- Variables ----- */
    private boolean hasTarget;
    private AlignPos position;
    private boolean redAlliance;
    private int tid;
    private boolean up = false;
    private Pose2d currentPose;

    private boolean hasCoral = intake.hasCoral();
    private boolean algae = intake.hasAlgae();

    private boolean wentup = false;
    private boolean intaking = false;
    int stuckCounter = 0;

    private Timer timer = new Timer();
    private boolean hasScored = false;

    private double robotRotation;
    private int[] possibleTags = new int[2];
    private boolean careAboutRotation;
    private boolean usedBackLL;
    private double currentX;
    
    private boolean onRedSide;

    private String debuggingCenterAlignIssue = "Null";
    private double debuggingTagForwardOffset = 0;
    private double debuggingTagLeftOffset = 0;
    private double debuggingXError = 0;
    private double debuggingYError = 0;
    private double debuggingPowMag = 0;
    private double debuggingVelMag = 0;

    private boolean runFrontLL;
    private boolean runBackLL;
    
    // Controller rumbles when at setpoint
    CommandXboxController controller;

    /* ----- Swerve Drive ----- */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric() // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* ----------- Initialzation ----------- */

    public AlignCommand(CommandSwerveDrivetrain drive, AlignPos position, CommandXboxController controller) {
        this.position = position;
        this.controller = controller;

        /* --------------- AprilTag location map --------------- */
        //                X,         Y,        Rotation
        double[] tag3 = {11.560833, 8.055626, 3 * Math.PI / 2};
        double[] tag6 =  {13.474446, 3.306318, 5 * Math.PI / 3.0};
        double[] tag7 =  {13.890498, 4.0259,   0};
        double[] tag8 =  {13.474446, 4.745482, Math.PI / 3.0};
        double[] tag9 =  {12.643358, 4.745482, 2 * Math.PI / 3.0};
        double[] tag10 = {12.227306, 4.0259,   Math.PI};
        double[] tag11 = {12.643358, 3.306318, 4 * Math.PI / 3.0};
        double[] tag16 = {5.987553, -0.003810, Math.PI / 2};
        double[] tag17 = {4.0739,    3.3063,   4 * Math.PI / 3.0};
        double[] tag18 = {3.6576,    4.0259,   Math.PI};
        double[] tag19 = {4.0739,    4.7455,   2 * Math.PI / 3.0};
        double[] tag20 = {4.9047,    4.7455,   Math.PI / 3.0};
        double[] tag21 = {5.3210,    4.0259,   0};
        double[] tag22 = {4.9047,    3.3063,   5 * Math.PI / 3.0};
        aprilTagLocationMap.put(3, tag3);
        aprilTagLocationMap.put(6, tag6);
        aprilTagLocationMap.put(7, tag7);
        aprilTagLocationMap.put(8, tag8);
        aprilTagLocationMap.put(9, tag9);
        aprilTagLocationMap.put(10, tag10);
        aprilTagLocationMap.put(11, tag11);
        aprilTagLocationMap.put(16, tag16);
        aprilTagLocationMap.put(17, tag17);
        aprilTagLocationMap.put(18, tag18);
        aprilTagLocationMap.put(19, tag19);
        aprilTagLocationMap.put(20, tag20);
        aprilTagLocationMap.put(21, tag21);
        aprilTagLocationMap.put(22, tag22);
        /* ----------------------------------------------------- */

        FpidRotate.enableContinuousInput(-Math.PI, Math.PI);
        BpidRotate.enableContinuousInput(-Math.PI, Math.PI);

        this.drive = drive;
    }

    @Override
    public void initialize() {
        hasScored = false;
        timer.restart();
        wentup = false;
        intaking = false;
        drive.runVision = true;
        stuckCounter = 0;
        redAlliance = DriverStation.getAlliance().get() == Alliance.Red;
        currentPose = drive.getState().Pose;
        usedBackLL = false;
        currentX = currentPose.getX();

        runFrontLL = RobotConstants.AlignConstants.runFrontLL;
        runBackLL = RobotConstants.AlignConstants.runBackLL;

        if (currentX > 8.775) {
            onRedSide = true;
        } else {
            onRedSide = false;
        }

        hasCoral = intake.hasCoral();
        algae = intake.hasAlgae();

        // grab target tag from limelight to align to
        int seenTid = -1;
        if (runFrontLL) {
            seenTid = (int)LimelightHelpers.getFiducialID("limelight-coral");
        }
        if (!aprilTagLocationMap.containsKey(seenTid) && runBackLL) {
            seenTid = (int)LimelightHelpers.getFiducialID("limelight-back");
            usedBackLL = true;
        }

        this.tid = seenTid;
        

        // if tag not a reef tag ignore it
        if (aprilTagLocationMap.containsKey(tid) || (algae)) {
            hasTarget = true;
            double offset = RobotConstants.AlignOffsets.firstCoralBack;
            // Initial position is back ~1 coral width
            if (algae) {
                if (onRedSide) {
                    tid = 3;
                } else {
                    tid = 16;
                }
                offset = RobotConstants.AlignOffsets.procOut;
                usedBackLL = false;
            }
            if (!algae && score.getDesiredLevel() == 0) {
                offset = RobotConstants.AlignOffsets.tripleL1CoralBack;
            }
            double[] pose = getAlignPos(aprilTagLocationMap.get(tid), offset);
            debuggingCenterAlignIssue = "In Initialization";
            FpidX.setSetpoint(pose[0]);
            FpidY.setSetpoint(pose[1]);
            FpidRotate.setSetpoint(pose[2]);
            BpidX.setSetpoint(pose[0]);
            BpidY.setSetpoint(pose[1]);
            BpidRotate.setSetpoint(pose[2]);
        } else {
            hasTarget = false;
        }
        up = false;

        possibleTags[0] = -1;
        possibleTags[1] = -1;

        robotRotation = 0;

        careAboutRotation = LimelightHelpers.getTargetCount("limelight-coral") > 1;
    }

    /* ----------- Updaters ----------- */

    /**
     * Calculates the aligned position based on the given target position.
     *
     * @param targetPos An array containing the target position with [x, y, rotation].
     * @return An array containing the aligned position with [x, y, rotation].
     */
    private double[] getAlignPos(double[] targetPos, double tagForwardOffset) {
        double tagLeftOffset;
        if (hasCoral && !algae && score.getPos() != ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC) {
            tagLeftOffset = (score.getDesiredLevel() == 1  || score.getDesiredLevel() == 0) ?RobotConstants.AlignOffsets.leftReefL1 :RobotConstants.AlignOffsets.leftReef;
            if (position == AlignPos.RIGHT) {
                tagLeftOffset = (score.getDesiredLevel() == 1  || score.getDesiredLevel() == 0) ?RobotConstants.AlignOffsets.rightReefL1 :RobotConstants.AlignOffsets.rightReef;
            }
        } else  if (!score.getAlgaeNet() && score.getPos() != ScoringPos.GO_TO_SCORE && !hasCoral){
            // Algae
            tagLeftOffset = RobotConstants.AlignOffsets.algaeLeft; // Set left offset for center
            tagForwardOffset = RobotConstants.AlignOffsets.algaeBack; //temp: 0.65 is old value of algaeBack // 0.45
        } else {
            tagLeftOffset = 0;
        }

        debuggingTagForwardOffset = tagForwardOffset;
        debuggingTagLeftOffset = tagLeftOffset;
        

        // Calculate rotation relative to the target position
        double rotation = targetPos[2] - Math.PI;

        // Normalize the rotation to ensure it wraps around for the shortest distance
        if (rotation < -Math.PI) {
            rotation += 2 * Math.PI;
        } else if (rotation > Math.PI) {
            rotation -= 2 * Math.PI;
        }

        // Calculate the new x and y coordinates based on the offsets and rotation
        double x = targetPos[0] - tagForwardOffset * Math.cos(rotation) - tagLeftOffset * Math.sin(rotation);
        double y = targetPos[1] - tagForwardOffset * Math.sin(rotation) + tagLeftOffset * Math.cos(rotation);

        // Create an array with the calculated x, y, and rotation values and return it
        double[] out = {x, y, rotation};

        return out;
    }

    /**
     * Executes the drive control based on the current pose and target information.
     */
    @Override
    public void execute() {

        if (currentX > 8.775) {
            onRedSide = true;
        } else {
            onRedSide = false;
        }

        careAboutRotation = LimelightHelpers.getTargetCount("limelight-coral") > 1;

        // hasCoral = intake.hasCoral();
        robotRotation = drive.getState().Pose.getRotation().getDegrees();

        double time = timer.get();

        if (!hasCoral && time > .05 && !wentup && ((!careAboutRotation || (tid == possibleTags[0] || tid == possibleTags[1])) || usedBackLL)) {
            new CoordinationCommand(ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC).schedule();
            wentup = true;
        }
        if(!hasCoral && time > .06 && score.getAllAtSetpoints() && !intaking && ((!careAboutRotation || (tid == possibleTags[0] || tid == possibleTags[1])) || usedBackLL)) {
            new DualIntakeCommand(true).schedule();
            intaking = true;
        }

        // Stop checking limelight pose once we lose target tag if aligning to the right
        // this is important because the limelight cannot see the april tag the whole way on that side
        if (position == AlignPos.RIGHT && LimelightHelpers.getFiducialID("limelight-coral") != tid) {
            drive.runVision = false;
        } else {
            drive.runVision = true;
        }

        // Raise elevator right away for L1-L1.5
        if (!score.getAlgae() && (score.getDesiredLevel() == 1 || score.getDesiredLevel() == 0) && !up && hasCoral && ((!careAboutRotation || (tid == possibleTags[0] || tid == possibleTags[1])) || usedBackLL) || (algae && score.getPos() != ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC)) {
            up = true;
            new CoordinationCommand(ScoringPos.GO_TO_SCORE).schedule();
        }

        if (algae) {
            if (onRedSide) {
                possibleTags[0] = 3;
                possibleTags[1] = -1;
            } else {
                possibleTags[0] = 16;
                possibleTags[1] = -1;
            }
        } else if ((-30 < robotRotation && robotRotation < 30) || ((-150 > robotRotation && robotRotation > -180) || (150 < robotRotation && robotRotation < 180))) {
            if (!onRedSide) {
                possibleTags[0] = 18;
                possibleTags[1] = 21;
            } else {
                possibleTags[0] = 7;
                possibleTags[1] = 10;
            }
        } else if ((30 < robotRotation && robotRotation < 90) || (-90 > robotRotation && robotRotation > -150)) {
            if (!onRedSide) {
                possibleTags[0] = 17;
                possibleTags[1] = 20;
            } else {
                possibleTags[0] = 11;
                possibleTags[1] = 8;
            }
        } else if (-30 > robotRotation && robotRotation > -90 || (90 < robotRotation && robotRotation < 150)) {
            if (!onRedSide) {
                possibleTags[0] = 19;
                possibleTags[1] = 22;
            } else {
                possibleTags[0] = 9;
                possibleTags[1] = 6;
            }
        }

        if (hasTarget && ((!careAboutRotation || (tid == possibleTags[0] || tid == possibleTags[1])) || usedBackLL) || (possibleTags[0] == 3 || possibleTags[0] == 16)) {
            // Scoot forward to scoring position once initial target is reached
            if (atSetpoint(0.06, 0.3) && !score.getAlgae() && !(score.getScoringLevel() == 1)) {
                double[] pose = getAlignPos(aprilTagLocationMap.get(tid),RobotConstants.AlignOffsets.scoreCoralBack);
                if (score.getDesiredLevel() == 0) {
                    pose = getAlignPos(aprilTagLocationMap.get(tid),RobotConstants.AlignOffsets.tripleL1CoralBack);
                } else if (score.getDesiredLevel() == 3) {
                    pose = getAlignPos(aprilTagLocationMap.get(tid),RobotConstants.AlignOffsets.scoreL3Back);
                }
                debuggingCenterAlignIssue = "Scoot";
                FpidX.setSetpoint(pose[0]);
                FpidY.setSetpoint(pose[1]);
                FpidRotate.setSetpoint(pose[2]);
                BpidX.setSetpoint(pose[0]);
                BpidY.setSetpoint(pose[1]);
                BpidRotate.setSetpoint(pose[2]);
            }
            else if (atSetpoint(0.06, 0.3) && !hasCoral && score.getPos() != ScoringPos.GO_TO_SCORE) {
                double[] pose = getAlignPos(aprilTagLocationMap.get(tid),RobotConstants.AlignOffsets.algaeIn);
                debuggingCenterAlignIssue = "Algae";
                FpidX.setSetpoint(pose[0]);
                FpidY.setSetpoint(pose[1]);
                FpidRotate.setSetpoint(pose[2]);
                BpidX.setSetpoint(pose[0]);
                BpidY.setSetpoint(pose[1]);
                BpidRotate.setSetpoint(pose[2]);
            }
            else if (atSetpoint(0.06, 0.3) && algae && score.getPos() == ScoringPos.GO_TO_SCORE) {
                double[] pose = getAlignPos(aprilTagLocationMap.get(tid),RobotConstants.AlignOffsets.procIn);
                debuggingCenterAlignIssue = "Proc";
                FpidX.setSetpoint(pose[0]);
                FpidY.setSetpoint(pose[1]);
                FpidRotate.setSetpoint(pose[2]);
                BpidX.setSetpoint(pose[0]);
                BpidY.setSetpoint(pose[1]);
                BpidRotate.setSetpoint(pose[2]);
            }

            // Non l4 coral go to score
            if (atSetpoint(0.5, 0.8)) {
                if (score.getDesiredLevel() != 4 && !up && !score.getAlgae() && hasCoral) {
                    up = true;
                    new CoordinationCommand(ScoringPos.GO_TO_SCORE).schedule();
                }
            }

            // Send elevator up if within tolerance at L4
            if (atSetpoint(0.3, 0.6)) {
                if (score.getDesiredLevel() == 4 && !up && !score.getAlgae() && hasCoral) {
                    up = true;
                    new CoordinationCommand(ScoringPos.GO_TO_SCORE).schedule();
                }
            }

            // Rumble controller to let driver know robot is ready to score
            if (atSetpoint(0.1, 0.2)) {
                controller.setRumble(RumbleType.kBothRumble, 0.5);
                if (up && !hasScored && (score.getDesiredLevel() != 1 || score.getDesiredLevel() != 0)) {
                    if (score.getScoringLevel() == 3) {
                        L3ScoreAndWait.schedule();
                    } else {
                    // new ScoringCommand().schedule();
                    }
                    hasScored = true;
                }
            } else {
                controller.setRumble(RumbleType.kBothRumble, 0);
            }
            // Get the current pose of the drive system
            currentPose = drive.getState().Pose;

            // Calculate drive power
            double powerX;
            double powerY;
            if (!usedBackLL) {
                powerX = FpidX.calculate(currentPose.getX()) * (MathUtil.clamp(time * 0.7, 1, 0));
                powerY = FpidY.calculate(currentPose.getY()) * (MathUtil.clamp(time * 0.7, 1, 0));   
            } else {
                powerX = BpidX.calculate(currentPose.getX()) * (MathUtil.clamp(time * 0.7, 1, 0));
                powerY = BpidY.calculate(currentPose.getY()) * (MathUtil.clamp(time * 0.7, 1, 0));      
            }

            // Slow down at L4
            if (score.getScoringLevel() == 4 && score.getPos() == ScoringPos.GO_TO_SCORE) {
                powerX = MathUtil.clamp(powerX, -1, 1);
                powerY = MathUtil.clamp(powerY, -1, 1);
            } 
            else if(!hasCoral && score.getPos() == ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC) {
                powerX = MathUtil.clamp(powerX, -1.5, 1.5);
                powerY = MathUtil.clamp(powerY, -1.5, 1.5);
            }
            else {
                powerX = MathUtil.clamp(powerX, -2, 2); // -2, 2
                powerY = MathUtil.clamp(powerY, -2, 2); // -2, 2
            }

            powerX += .05*Math.signum(powerX);
            powerY += .05*Math.signum(powerY);
            
            double xError;
            double yError;
            if (!usedBackLL) {
                xError = Math.abs(FpidX.getSetpoint() - currentPose.getX());
                yError = Math.abs(FpidY.getSetpoint() - currentPose.getY());
            } else {
                xError = Math.abs(BpidX.getSetpoint() - currentPose.getX());
                yError = Math.abs(BpidY.getSetpoint() - currentPose.getY());
            }

            debuggingXError = xError;
            debuggingYError = yError;

            // Calculate the rotational power and clamp it between -2 and 2
            double powerRotate;
            if (!usedBackLL) {
                powerRotate = FpidRotate.calculate(currentPose.getRotation().getRadians());
            } else {
                powerRotate = BpidRotate.calculate(currentPose.getRotation().getRadians());
            }

            powerRotate = MathUtil.clamp(powerRotate, -6, 6); //-4, 4

            if (redAlliance) {
                powerX *= -1;
                powerY *= -1;
            }

            // Calculate magnitude of velocity and power
            double xVel = drive.getState().Speeds.vxMetersPerSecond;
            double yVel = drive.getState().Speeds.vyMetersPerSecond;
            double powMag = powerX * powerX + powerY * powerY;
            double velMag = xVel * xVel + yVel * yVel;
            debuggingPowMag = powMag;
            debuggingVelMag = velMag;

            // If power is above a certain threshold and velocity is near zero, must be stuck on a coral
            if (powMag > 0.3 && velMag < 0.02) {
                stuckCounter++;
            } else {
                stuckCounter = 0;
            }

            // I KILLED IT YIPPEEE
            // --- Comment this to disable coral detection ---
            // Change arm position to account for coral
            if (stuckCounter > 5 && hasCoral) {
                score.setCoralInFront(true);
                if (!up) {
                    new CoordinationCommand(ScoringPos.GO_TO_SCORE).schedule();
                    up = true;
                }
            }


            // ------------------------------------------------

            SwerveRequest request = driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
            

            // Set the drive control with the created request
            drive.setControl(request);
        }

        if (RobotConstants.debugging.AlignDebugging) {
            debugging();
        }
    }

    public boolean atSetpoint() {
        return atSetpoint(0.03, 0.2);
    }

    public boolean atSetpoint(double translationTolerance, double rotationTolerance) {
        //We use front LL pid stuff here instead of checking which one to use due to how both have the same setpoint and the currentPose.get... is the same for both
        return Math.abs(FpidX.getSetpoint() - currentPose.getX()) < translationTolerance && Math.abs(FpidY.getSetpoint() - currentPose.getY()) < translationTolerance && FpidRotate.getError() < rotationTolerance;
    }

    private void debugging() {
        Logger.recordOutput("Reefscape/Align/Tid", tid);
        Logger.recordOutput("Reefscape/Align/Tag Left offset", debuggingTagLeftOffset);
        Logger.recordOutput("Reefscape/Align/Tag Forawrd Offset", debuggingTagForwardOffset);
        Logger.recordOutput("Reefscape/Align/FL Drive AMP Pull", drive.getModule(1).getDriveMotor().getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Reefscape/Align/On Red Side?", onRedSide);
        Logger.recordOutput("Reefscape/Align/Debugging centering issue", debuggingCenterAlignIssue);
        Logger.recordOutput("Reefscape/Align/Running Front LL", runFrontLL);
        Logger.recordOutput("Reefscape/Align/Running Back LL", runBackLL);
        Logger.recordOutput("Reefscape/Align/x error", Math.abs(FpidX.getSetpoint() - currentPose.getX()));
        Logger.recordOutput("Reefscape/Align/y error", Math.abs(FpidY.getSetpoint() - currentPose.getY()));
        Logger.recordOutput("Reefscape/Align/ErrorMag", debuggingXError * debuggingXError + debuggingYError * debuggingYError);
        Logger.recordOutput("Reefscape/Align/rot error", FpidRotate.getError());
        Logger.recordOutput("Reefscape/Align/VelMag", debuggingVelMag);
        Logger.recordOutput("Reefscape/Align/PowerMag", debuggingPowMag);
        Logger.recordOutput("Reefscape/Align/Possible Tags", possibleTags);
    }

    /* ----------- Finishers ----------- */

    /**
     * Ends the current drive control, either when the command is finished or interrupted.
     *
     * @param interrupted True if the command was interrupted, false otherwise.
     */
    @Override
    public void end(boolean interrupted) {
        // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
        SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        drive.runVision = true;
        // Set the drive control with the stop request to halt all movement
        drive.setControl(stop);
        controller.setRumble(RumbleType.kBothRumble, 0);
    }
}
