package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class MoveIntake extends Command{
    
    public Intake mIntake;
    public double Pos;

    public MoveIntake(Intake intake, double Position){
        mIntake = intake;
        Pos = Position;

        addRequirements(mIntake);
    }

    @Override
    public void execute() {
        if(Pos == 90){
            mIntake.ShooterPos();
        }else if(Pos == 270){
            mIntake.FloorPos();
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
