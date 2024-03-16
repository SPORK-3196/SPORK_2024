package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Intake;

public class AutoIntake extends Command{    
    private Intake mIntake;

    public AutoIntake(Intake mIntake){
        this.mIntake = mIntake;

        addRequirements(mIntake);
    }

    @Override
    public void initialize() {
        mIntake.FloorPos();
        mIntake.grab();
    }

    @Override
    public void execute() {
        if(!mIntake.NoteIn.get()){
            Robot.secondary.setRumble(RumbleType.kBothRumble, 1);
            mIntake.ShooterPos();
        }else{
            Robot.secondary.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.ShooterPos();
        mIntake.Keep();
        Robot.secondary.setRumble(RumbleType.kBothRumble, 0);
    }

}
