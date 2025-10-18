package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DiffWristSubsystem;

/*
 * Roll wrist to opposite side
 */
public class RotateLock extends Command{
    
    /* ----- Subsystem Instance ----- */
    public final CommandSwerveDrivetrain drivetrain;

    /* ----------- Initialization ----------- */

    public RotateLock(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if()
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
    }

    @Override
    public void end(boolean interrupted) {
    }

}
