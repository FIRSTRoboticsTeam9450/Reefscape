package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private boolean hasCoral;


    public DriverIntakeCommand (CommandXboxController driveController, CommandSwerveDrivetrain drive) {
        this.drive = drive;
        this.driveController = driveController;
        forward = new DriveForwardCommand(drive, driveController);
    }

    @Override
    public void initialize() {
        boolean use1Controller = SmartDashboard.getBoolean("Experimental Keybinds choosen setting", false);
        if (use1Controller) {
            hasCoral = intake.hasCoral();
            if (score.getPos() == ScoringPos.INTAKE_CORAL) {
                if (driveController.getLeftTriggerAxis() > 0.05) {
                    if (!forward.isScheduled()) {
                        forward.schedule();
                    }
                } else {
                    forward.cancel();
                }
            } else if (hasCoral) {
                new GoToScorePosCommand().schedule();
            } else {
                new CoordinationCommand(ScoringPos.INTAKE_ALGAE)
                    .andThen(new DualIntakeCommand(true)).schedule();
            }
        } else {
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
        }
    }

    @Override
    public boolean isFinished() {
        return driveController.getLeftTriggerAxis() < 0.02;
    }

    @Override
    public void end(boolean interrupted) {
        forward.cancel();
    }
    
}
