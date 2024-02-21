package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class IntakeBump extends Command {
    public Intake intake;

    public IntakeBump(Intake intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.FloorPos();
        intake.feed();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.ShooterPos();
        intake.Keep();
    }
}
