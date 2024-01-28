package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class MoveIntake extends Command{
    
    public Intake mIntake;
    public double Pos;

    public MoveIntake(Intake intake, double Position){
        mIntake = intake;

        addRequirements(mIntake);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
