package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake;

public class Intake extends SubsystemBase {

private Spark IntakeNeo = new Spark(0);

    public Intake(){
    


    }

    public void intake(){
        IntakeNeo.set(kIntake.IntakeSpeed);
    }

    public void feed(){
        IntakeNeo.set(kIntake.FeedSpeed);
    }




    
}