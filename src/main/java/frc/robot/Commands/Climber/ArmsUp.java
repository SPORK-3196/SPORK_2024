package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb;

public class ArmsUp extends Command {
    
    public Climb Climb;
    private double speed;


    public ArmsUp(Climb mClimb, double mspeed){
        Climb = mClimb;
        speed = mspeed;
        addRequirements(Climb);
    }

    @Override
    public void execute() {
        if(Climb.LeftArmEncoder.getPosition() >= Climb.RightArmEncoder.getPosition()){
            Climb.LeftUp(0);
            Climb.RightUp(speed);
        }else if (Climb.RightArmEncoder.getPosition() >= Climb.LeftArmEncoder.getPosition()) {
            Climb.LeftUp(speed);
            Climb.RightUp(0);
        } else{
            Climb.LeftUp(speed);
            Climb.RightUp(speed);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        Climb.LeftStop();
        Climb.RightStop();
    }
}

