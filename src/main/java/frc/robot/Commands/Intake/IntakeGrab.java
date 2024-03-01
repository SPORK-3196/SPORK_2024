package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class IntakeGrab extends Command {
        
    private Intake intake;

    public IntakeGrab(Intake intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.grab();
    }

    @Override
    public void end(boolean interrupted) {
        intake.Keep();
    }
}
