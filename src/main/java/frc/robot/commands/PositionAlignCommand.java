package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants.AlignPos;
import frc.robot.Constants.RobotConstants.AlignOffsets;
import frc.robot.Constants.FieldConstants.ReefConstants.BlueReefConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.RedReefConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ScoringPos;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;

/**
 * The goal of this command is to use our auto align system without the need of seeing an april tag
 */
public class PositionAlignCommand extends Command {


    /*  ┌───────────────────────────┐
     *  |          Fields           |
     *  └───────────────────────────┘
     */ 

     
    /* --------------- Subsystem Instances --------------- */
    CommandSwerveDrivetrain drivetrain;
    DualIntakeSubsystem intakeInstance = DualIntakeSubsystem.getInstance();
    CoordinationSubsytem coordSubInstance = CoordinationSubsytem.getInstance();

    /* --------------- Unique Scoring Commands --------------- */
    private SequentialCommandGroup L3DelayedScore = new SequentialCommandGroup(new WaitCommand(0.25).andThen(new ScoringCommand()));

    /* --------------- PIDs --------------- */
    private PIDController pidX = new PIDController(2, 0, 0);
    private PIDController pidY = new PIDController(2, 0, 0);
    private PIDController pidR = new PIDController(3, 0, 0);

    /* --------------- Timer --------------- */
    private Timer timer = new Timer();

    /* --------------- Variables --------------- */
    private int targetID = -1;
    private int desiredLevel = 1;
    private boolean hasStateChanged;
    private AlignPos alignPos;
    private Pose2d currentFieldPose;
    private ScoringPos currentState;
    private boolean redAlliance;
    private boolean haveCoral;
    private boolean haveAlgae;
    private boolean hasScoredYet;

    /* --------------- State Specific limitations --------------- */
    private boolean haveStartedIntaking;
    private boolean algaeIntakeStateChange;

    /* --------------- Drive Request --------------- */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric() // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    /*  ┌───────────────────────────┐
     *  |       Initialization      |
     *  └───────────────────────────┘
     */ 


    /**
     * Configures instances of drive subsystem, which pole we want to go to, and april tag locations
     * @param drivetrain
     * @param alignPos
     */
    public PositionAlignCommand(CommandSwerveDrivetrain drivetrain, AlignPos alignPos) {
        this.drivetrain = drivetrain;
        this.alignPos = alignPos;

        pidR.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        currentFieldPose = drivetrain.getState().Pose;
        currentState = coordSubInstance.getPos();

        desiredLevel = coordSubInstance.getDesiredLevel();
        targetID = calculateClosestTag();

        haveCoral = intakeInstance.hasCoral();
        haveAlgae = intakeInstance.hasAlgae();

        redAlliance = !FieldConstants.isBlueAlliance;

        hasStateChanged = false;

        algaeIntakeStateChange = false;
        haveStartedIntaking = false;

        hasScoredYet = false;

        timer.restart();

        initialAlignment();
    }

    /**
     * Only use case is to make the @Overridden Initialize method less crowded
     */
    private void initialAlignment() {
        double forwardOffset = AlignOffsets.firstCoralBack;
        if (haveAlgae) {
            targetID = FieldConstants.isBlueAlliance ? 16 : 3;
            forwardOffset = AlignOffsets.procOut;
        }
        if (!haveAlgae && desiredLevel == 0) {
            forwardOffset = AlignOffsets.tripleL1CoralBack;
        }

        double[] tmpOffsetArr = calculateAlignPosition(FieldConstants.getTag3dPose(targetID).toPose2d(), forwardOffset);
        double[] tmpPowerArr = calculateDrivePower(tmpOffsetArr);
        setSwerveRequest(tmpPowerArr);
    }


    /*  ┌───────────────────────────┐
     *  |      Periodical loop      |
     *  └───────────────────────────┘
     */ 


    @Override
    public void execute() {


        /* --------------- State Changing --------------- */
        double time = timer.get();
        if (!hasStateChanged && (!algaeIntakeStateChange || !haveStartedIntaking)) {
            updateState(time);
        }


        /* --------------- Movement --------------- */

        double forwardOffset;

        /* ---------- Scoot ---------- */
        if (atSetpoint(0.06, 0.3) && !haveAlgae && desiredLevel != 1) {
            switch(desiredLevel) {
                case 0:
                    forwardOffset = AlignOffsets.tripleL1CoralBack;
                    break;
                case 3:
                    forwardOffset = AlignOffsets.scoreL3Back;
                    break;
                default:
                    forwardOffset = AlignOffsets.scoreCoralBack;
                    break;
            }
            setSwerveRequest(
                calculateDrivePower(
                    calculateAlignPosition(currentFieldPose, forwardOffset)));
        }
        /* ---------- Algae ---------- */
         else if (atSetpoint(0.06, 0.3) && !haveCoral && currentState != ScoringPos.GO_TO_SCORE) {
            forwardOffset = AlignOffsets.algaeIn;
            setSwerveRequest(
                calculateDrivePower(
                    calculateAlignPosition(currentFieldPose, forwardOffset)));
        }
         else if (atSetpoint(0.06, 0.3) && haveAlgae && currentState == ScoringPos.GO_TO_SCORE) {
            forwardOffset = AlignOffsets.procIn;
            setSwerveRequest(
                calculateDrivePower(
                    calculateAlignPosition(currentFieldPose, forwardOffset)));
         }


         /* --------------- Scoring --------------- */

         if (atSetpoint(0.1, 0.2) && hasStateChanged && !hasScoredYet && (desiredLevel != 1 || desiredLevel != 0)) {
            switch (desiredLevel) {
                case 3:
                    L3DelayedScore.schedule();
                    break;
                default:
                    new ScoringCommand().schedule();
                    break;
            }
         }
    }

    /**
     * Helper Method for execute where we do checks to see if we should change our state
     * @param time how much time has passed since start of command, used for sepearation of order
     */
    private void updateState(double time) {

        /* ---------- Algae Intake from Reef ---------- */  
        if (!haveCoral && time > 0.05 && !algaeIntakeStateChange) {
            new CoordinationCommand(ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC).schedule();;
            algaeIntakeStateChange = true;
        }
        if (!haveCoral && coordSubInstance.getAllAtSetpoints() && time > 0.06 && !haveStartedIntaking) {
            new DualIntakeCommand(true).schedule();
            haveStartedIntaking = true;
        }

        /* ---------- L1-L1.5 and Proc Score ---------- */
        if (
            (!hasStateChanged && !haveAlgae && haveCoral && (desiredLevel == 0 || desiredLevel == 1))
             || (haveAlgae && currentState != ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC))
        {
            new CoordinationCommand(ScoringPos.GO_TO_SCORE).schedule();
            hasStateChanged = true;
        }

        /* ---------- Non-L4 Coral ---------- */
        if (atSetpoint(0.5, 0.8) && desiredLevel != 4 && !hasStateChanged && haveCoral && !haveAlgae) {
            hasStateChanged = true;
            new CoordinationCommand(ScoringPos.GO_TO_SCORE).schedule();
        }

        /* ---------- L4 Coral ---------- */
        if (atSetpoint(0.3, 0.6) && !hasStateChanged && desiredLevel == 4 && haveCoral && !haveAlgae) {
            hasStateChanged = true;
            new CoordinationCommand(ScoringPos.GO_TO_SCORE).schedule();
        }
    }


    /*  ┌───────────────────────────┐
     *  |        Calculations       |
     *  └───────────────────────────┘
     */ 


    /**
     * Calculates which reef side / april tag robot is currently closest to
     * @return ID of april tag
     */
    private int calculateClosestTag() {

        int closestTagID = -1;
        Pose2d currentFieldPose = drivetrain.getState().Pose;
        List<Pose2d> possibleReefSides = List.of();

        //Depending if on blue alliance or red alliance
        if (FieldConstants.isBlueAlliance) {
            possibleReefSides = BlueReefConstants.blueAlliancePoseToTagIDsMap.keySet().stream().toList();

            closestTagID = BlueReefConstants.blueAlliancePoseToTagIDsMap.get(currentFieldPose.nearest(possibleReefSides));
        } else {
            possibleReefSides = RedReefConstants.redAlliancePoseToTagIDsMap.keySet().stream().toList();

            closestTagID = RedReefConstants.redAlliancePoseToTagIDsMap.get(currentFieldPose.nearest(possibleReefSides));
        }

        return closestTagID;
    }

    /**
     * Calculate the relative Distance and rotational errors from robot position to April Tag location of the target reef side
     * @param aprilTagPose 2D pose of April tag for targeted reef
     * @param tagForwardOffset Forward/Backward Distance away from Reef we wish to be (Meters)
     * @return Double array which holds the distance from robot to april tag of reef side in (X, Y, Rot)
     */
    private double[] calculateAlignPosition(Pose2d aprilTagPose, double tagForwardOffset) {
        /* --------------- Offset Calculations --------------- */
        //Calculate how far in and to the side we wish to be relative to the april tag
        double tagLeftOffset;
        if (haveCoral && !haveAlgae && currentState != ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC) {
            tagLeftOffset = (desiredLevel == 1  || desiredLevel == 0) ? RobotConstants.AlignOffsets.leftReefL1 :RobotConstants.AlignOffsets.leftReef;
            if (alignPos == AlignPos.RIGHT) {
                tagLeftOffset = (desiredLevel == 1  || desiredLevel == 0) ? RobotConstants.AlignOffsets.rightReefL1 :RobotConstants.AlignOffsets.rightReef;
            }
        } else if (!haveCoral && currentState != ScoringPos.GO_TO_SCORE) {
            tagLeftOffset = AlignOffsets.algaeLeft;
            tagForwardOffset = AlignOffsets.algaeBack;
        } else {
            tagLeftOffset = 0;
        }

        /* --------------- Setpoint Calculations --------------- */

        //Calculate Rotational Error from robot to reef side
        double rotationalError = aprilTagPose.getRotation().getRadians();
        rotationalError -= Math.PI;

        //Normalize Error to ensure it wraps for shortest distance
        if (rotationalError < -Math.PI) {
            rotationalError += 2 * Math.PI;
        } else if (rotationalError > Math.PI) {
            rotationalError -= 2 * Math.PI;
        }

        //Calculate X & Y Error from robot to April Tag of Reef Side
        double xError = aprilTagPose.getX() - tagForwardOffset * Math.cos(rotationalError) - tagLeftOffset * Math.sin(rotationalError);
        double yError = aprilTagPose.getY() - tagForwardOffset * Math.sin(rotationalError) + tagLeftOffset * Math.cos(rotationalError);

        double[] out = {xError, yError, rotationalError};

        return out;
    }

    /**
     * Used to keep main methods like initialize and execute clean and organized.
     * <p>
     * Calculates the power in which the drive request should use
     * </p>
     * @param pidSetpoints Offsets that the CalculateAlignPositions method should have given
     * @return Double array of which the drive request should use (xVeloc, yVeloc, rotVeloc)
     */
    private double[] calculateDrivePower(double[] pidSetpoints) {
        //used for acceleration
        double time = timer.get();

        //Powers in which the drive request will use
        double xPower = pidX.calculate(currentFieldPose.getX()) * (MathUtil.clamp(time * 0.7, 1, 0));
        double yPower = pidY.calculate(currentFieldPose.getY()) * (MathUtil.clamp(time * 0.7, 1, 0));

        //Adjustments of the power depending on certain conditions
        if (currentState == ScoringPos.GO_TO_SCORE && desiredLevel == 4) {
            xPower = MathUtil.clamp(xPower, -1, 1);
            yPower = MathUtil.clamp(yPower, -1, 1);
        } else if (!haveCoral && currentState == ScoringPos.ALGAE_INTAKE_REEF_DYNAMIC) {
            xPower = MathUtil.clamp(xPower, -1.5, 1.5);
            yPower = MathUtil.clamp(yPower, -1.5, 1.5);
        } else {
            xPower = MathUtil.clamp(xPower, -2, 2);
            yPower = MathUtil.clamp(yPower, -2, 2);
        }

        xPower += .05*Math.signum(xPower);
        yPower += .05*Math.signum(yPower);

        double rPower = pidR.calculate(currentFieldPose.getRotation().getRadians());
        rPower = MathUtil.clamp(rPower, -6, 6);

        //Flips power depending on alliance due to IMU being different
        if (redAlliance) {
            xPower *= -1;
            yPower *= -1;
        }

        double[] out = {xPower, yPower, rPower};
        
        return out;
    }


    /*  ┌───────────────────────────┐
     *  |       Swerve Request      |
     *  └───────────────────────────┘
     */


    /**
     * Used for having a clean way to set Swerve with just needing to provide a previously calculated array
     * @param powerArr array which contains calculated power values
     */
    private void setSwerveRequest(double[] powerArr) {
        SwerveRequest swerveRequest = driveRequest
                                            .withVelocityX(powerArr[0])
                                            .withVelocityY(powerArr[1])
                                            .withRotationalRate(powerArr[2]);
        
        drivetrain.setControl(swerveRequest);
    }


    /*  ┌───────────────────────────┐
     *  |    Setpoint Tolerences    |
     *  └───────────────────────────┘
     */ 


    public boolean atSetpoint() {
        return atSetpoint(0.03, 0.2);
    }

    public boolean atSetpoint(double translationTolerance, double rotationTolerance) {
        return Math.abs(pidX.getSetpoint() - currentFieldPose.getX()) < translationTolerance && Math.abs(pidY.getSetpoint() - currentFieldPose.getY()) < translationTolerance && pidR.getError() < rotationTolerance;
    }


    /*  ┌───────────────────────────┐
     *  |            End            |
     *  └───────────────────────────┘
     */ 


     /**
     * Ends the current drive control, either when the command is finished or interrupted.
     *
     * @param interrupted True if the command was interrupted, false otherwise.
     */
    @Override
    public void end(boolean interrupted) {
        // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
        SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        drivetrain.runVision = true;
        // Set the drive control with the stop request to halt all movement
        drivetrain.setControl(stop);
    }

}