package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kRollerBars;

public class Roller extends SubsystemBase {
    
    private CANSparkMax RollerNeo = new CANSparkMax(kRollerBars.RollerNeoPort, MotorType.kBrushless);


    public Roller(){
        RollerNeo.setIdleMode(kRollerBars.RollerIdle);
        RollerNeo.setInverted(kRollerBars.RollerInvert);
    }

    public void RunRoller(){
        RollerNeo.set(kRollerBars.RollerSpeed);
    }

    // TODO method to bring the rollers into positon

    

    




}