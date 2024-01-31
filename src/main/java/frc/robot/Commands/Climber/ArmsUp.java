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
        Climb.LeftUp(speed);
        Climb.RightUp(speed);
    }
    
    @Override
    public void end(boolean interrupted) {
        Climb.LeftStop();
        Climb.RightStop();
    }
}

