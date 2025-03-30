package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;

public class DriverIntakeCommand extends Command {

    DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    CoordinationSubsytem score = CoordinationSubsytem.getInstance();
    CommandSwerveDrivetrain drive;
    CommandXboxController driveController;

    DriveForwardCommand forward;


    public DriverIntakeCommand (CommandXboxController driveController, CommandSwerveDrivetrain drive) {
        this.drive = drive;
        this.driveController = driveController;
        forward = new DriveForwardCommand(drive, driveController);
    }

    @Override
    public void initialize() {
        if (score.getPos() == ScoringPos.INTAKE_CORAL) {
            if (driveController.getLeftTriggerAxis() > 0.05) {
                if (!forward.isScheduled()) {
                    forward.schedule();
                }
            } else {
                forward.cancel();
            }
        } else {
            new GoToScorePosCommand().schedule();
        }

        // if (!score.getAlgae()) {
        //     new CoordinationCommand(ScoringPos.INTAKE_CORAL).andThen(new DualIntakeCommand(false)).andThen(new CoordinationCommand(ScoringPos.CORAL_STORE)).schedule();
        // } else {
        //     new GoToScorePosCommand().schedule();
        // }
    }
    
}
