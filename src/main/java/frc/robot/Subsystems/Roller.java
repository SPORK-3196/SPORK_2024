package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kRollerBars;

public class Roller extends SubsystemBase {
    
    private CANSparkMax RollerNeo = new CANSparkMax(kRollerBars.RollerNeoPort, MotorType.kBrushless);

    // test for github!!!

    public Roller(){
        RollerNeo.setIdleMode(kRollerBars.RollerIdle);
        RollerNeo.setInverted(kRollerBars.RollerInvert);
    }

    public void RunRoller(){
        
    }

    // TODO method to bring the rollers into positon


    

    

    




}
