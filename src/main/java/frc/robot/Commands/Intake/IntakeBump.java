package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class IntakeBump extends Command {
    private Intake mintake;

    public IntakeBump(Intake mintake){
        this.mintake = mintake;

        addRequirements(mintake);
    }

    @Override
    public void initialize() {
        mintake.FloorPos();
        mintake.feed();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        mintake.ShooterPos();
        mintake.Keep();
    }
}
