package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class BetterAutoAlignTest extends Command {
    
    private CommandSwerveDrivetrain drive;
    private double robotRotation;

    Timer timer = new Timer();

    private int[] possibleTags = new int[2];

    public BetterAutoAlignTest(CommandSwerveDrivetrain drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        timer.restart();
        possibleTags[0] = -1;
        possibleTags[1] = -1;
        robotRotation = 0;
    }

    @Override
    public void execute() {
        double rawRobotRotation = drive.getState().Pose.getRotation().getDegrees();
        // robotRotation = rawRobotRotation * (180 / Math.PI);
        robotRotation = rawRobotRotation;

        if ((-30 < robotRotation && robotRotation < 30) || ((-150 > robotRotation && robotRotation > -180) || (150 < robotRotation && robotRotation < 180))) {
            possibleTags[0] = 18;
            possibleTags[1] = 21;
        } else if ((30 < robotRotation && robotRotation < 90) || (-90 > robotRotation && robotRotation > -150)) {
            possibleTags[0] = 17;
            possibleTags[1] = 20;
        } else if (-30 > robotRotation && robotRotation > -90 || (90 < robotRotation && robotRotation < 150)) {
            possibleTags[0] = 19;
            possibleTags[1] = 22;
        }
        Logger.recordOutput("Reefscape/BetterAlign/Possible April Tags", possibleTags);
        Logger.recordOutput("Reefscape/BetterAlign/Robot Rotation", robotRotation);
        Logger.recordOutput("Reefscape/BetterAlign/Raw Robot Rotation", rawRobotRotation);
        Logger.recordOutput("Reefscape/BetterAlign/Command Timer", timer.get());
    }

    @Override
    public boolean isFinished() {
        // return timer.get() > 10;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

}
