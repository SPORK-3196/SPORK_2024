package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake;

public class Intake extends SubsystemBase {

private CANSparkMax IntakeNeo = new CANSparkMax(kIntake.IntakePort, MotorType.kBrushless);

    public Intake(){
    


    }

    public void intake(){
        IntakeNeo.set(kIntake.IntakeSpeed);
    }

    public void feed(){
        IntakeNeo.set(kIntake.FeedSpeed);
    }


    // TODO Add intake in and out 
    

    // TODO add logic to stop intake when a piece is in

    
    // TODO add Logic to Feed note when intake is up and when the shooter is spooling up


    
}