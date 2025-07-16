package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class AlignCommand extends Command {

    private HashMap<Integer, double[]> tags = new HashMap<>();
    private PIDController PIDx = new PIDController(6, 0, 0);
    private PIDController PIDy = new PIDController(6, 0, 0);
    private PIDController PIDRotation = new PIDController(8, 0, 0);

    private Pose3d currentPose = new Pose3d();
    private int currentTag;
    private boolean redAlliance;

    /*
     * Steps:
     * Initialize
     * 1. Make a map of all the x, y, rotation of each tag position
     * 2. Make PIDs 
     * 3. Find the target tag
     * 3. Get the adjusted target
     *    a. Adjust x position based on which side you're going to
     *    b. Adjust y position based on
     *    c. Adjust rotation to make it rotate from its shortest side
     * 4. Put it as the set point in the PID
     * Execute
     * 1. Set drive.runVision if in a position where you can see the tag
     * 2. Calculate the power with PID and current position
     * 3. Clamp it
     * 4. Change the direction of the power based on which alliance it is
     * 5. Set drive request with the new powers
     */
    @Override
    public void initialize() {
        /* --------------- AprilTag location map --------------- */ 
        //                X,         Y,        Rotation
        double[] tag6 =  {13.474446, 3.306318, 5 * Math.PI / 3.0};
        double[] tag7 =  {13.890498, 4.0259,   0};
        double[] tag8 =  {13.474446, 4.745482, Math.PI / 3.0};
        double[] tag9 =  {12.643358, 4.745482, 2 * Math.PI / 3.0};
        double[] tag10 = {12.227306, 4.0259,   Math.PI};
        double[] tag11 = {12.643358, 3.306318, 4 * Math.PI / 3.0};
        double[] tag17 = {4.0739,    3.3063,   4 * Math.PI / 3.0};
        double[] tag18 = {3.6576,    4.0259,   Math.PI};
        double[] tag19 = {4.0739,    4.7455,   2 * Math.PI / 3.0};
        double[] tag20 = {4.9047,    4.7455,   Math.PI / 3.0};
        double[] tag21 = {5.3210,    4.0259,   0};
        double[] tag22 = {4.9047,    3.3063,   5 * Math.PI / 3.0};
        tags.put(6, tag6);
        tags.put(7, tag7);
        tags.put(8, tag8);
        tags.put(9, tag9);
        tags.put(10, tag10);
        tags.put(11, tag11);
        tags.put(17, tag17);
        tags.put(18, tag18);
        tags.put(19, tag19);
        tags.put(20, tag20);
        tags.put(21, tag21);
        tags.put(22, tag22);
        /* ----------------------------------------------------- */
        currentTag = (int)LimelightHelpers.getFiducialID("limelight-coral");
        redAlliance = DriverStation.getAlliance().get() == Alliance.Red;

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
