package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kIntake;
import frc.robot.Subsystems.Intake;

public class Vomit extends Command {
    
    private Intake intake;

    public Vomit(Intake intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.spitPos();
    }

    @Override
    public void execute() {
        if(intake.getPos() >= kIntake.spitPos - 2){
            intake.spit();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.Keep();
        intake.ShooterPos();
    }


}
