package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignHelper.AlignPos;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * The goal of this command is to use our auto align system without the need of seeing an april tag
 */
public class PositionAlignCommand extends Command {

    /* --------------- Subsystem Instances --------------- */
    CommandSwerveDrivetrain drivetrain;

    /* --------------- April Tag Location Map --------------- */
    private HashMap<Pose2d, Integer> apriltagLocationMap = new HashMap<>();

    /* --------------- PIDs --------------- */
    private PIDController pidX = new PIDController(2, 0, 0);
    private PIDController pidY = new PIDController(2, 0, 0);
    private PIDController pidR = new PIDController(3, 0, 0);

    /* --------------- Variables --------------- */
    private int targetID = -1;
    private AlignPos alginPos;
    private Pose2d currentFieldPosition;
    private boolean redAlliance;

    /**
     * Configures instances of drive subsystem, which pole we want to go to, and april tag locations
     * @param drivetrain
     * @param alignPos
     */
    public PositionAlignCommand(CommandSwerveDrivetrain drivetrain, AlignPos alignPos) {
        this.drivetrain = drivetrain;
        this.alginPos = alignPos;

        /* ------------------------------ AprilTag location map ------------------------------ */
        //                             X,           Y,         Rotation
        Pose2d tag3 =  new Pose2d(11.560833, 8.055626, new Rotation2d(3 * Math.PI / 2));
        Pose2d tag6 =  new Pose2d(13.474446, 3.306318, new Rotation2d(5 * Math.PI / 3.0));
        Pose2d tag7 =  new Pose2d(13.890498, 4.0259,   new Rotation2d(0));
        Pose2d tag8 =  new Pose2d(13.474446, 4.745482, new Rotation2d(Math.PI / 3.0));
        Pose2d tag9 =  new Pose2d(12.643358, 4.745482, new Rotation2d(2 * Math.PI / 3.0));
        Pose2d tag10 = new Pose2d(12.227306, 4.0259,   new Rotation2d(Math.PI));
        Pose2d tag11 = new Pose2d(12.643358, 3.306318, new Rotation2d(4 * Math.PI / 3.0));
        Pose2d tag16 = new Pose2d(5.987553,   -0.003810, new Rotation2d(Math.PI / 2));
        Pose2d tag17 = new Pose2d(4.0739,    3.3063,   new Rotation2d(4 * Math.PI / 3.0));
        Pose2d tag18 = new Pose2d(3.6576,    4.0259,   new Rotation2d(Math.PI));
        Pose2d tag19 = new Pose2d(4.0739,    4.7455,   new Rotation2d(2 * Math.PI / 3.0));
        Pose2d tag20 = new Pose2d(4.9047,    4.7455,   new Rotation2d(Math.PI / 3.0));
        Pose2d tag21 = new Pose2d(5.3210,    4.0259,   new Rotation2d(0));
        Pose2d tag22 = new Pose2d(4.9047,    3.3063,   new Rotation2d(5 * Math.PI / 3.0));
        apriltagLocationMap.put(tag3,  3);
        apriltagLocationMap.put(tag6,  6);
        apriltagLocationMap.put(tag7,  7);
        apriltagLocationMap.put(tag8,  8);
        apriltagLocationMap.put(tag9,  9);
        apriltagLocationMap.put(tag10, 10);
        apriltagLocationMap.put(tag11, 11);
        apriltagLocationMap.put(tag16, 16);
        apriltagLocationMap.put(tag17, 17);
        apriltagLocationMap.put(tag18, 18);
        apriltagLocationMap.put(tag19, 19);
        apriltagLocationMap.put(tag20, 20);
        apriltagLocationMap.put(tag21, 21);
        apriltagLocationMap.put(tag22, 22);
        /* ----------------------------------------------------- */

        pidR.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        currentFieldPosition = drivetrain.getState().Pose;
    }

    @Override
    public void execute() {
        
    }
    /**
     * Calculates which reef side / april tag robot is currently closest to
     * @return ID of april tag
     */
    private int calculateClosestTag() {
        
        int closestTagID = -1;
        Pose2d currentFieldPose = drivetrain.getState().Pose;
        List<Pose2d> possibleReefSides = List.of();

        return closestTagID;
    }

}