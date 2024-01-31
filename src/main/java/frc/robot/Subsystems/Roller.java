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

    public void NoRoller(){

    }

    public Command SpinRoller(boolean amp){
        if(amp){
            return this.run(() -> RunRoller());
        }else{
            return this.run(null);
        }
    }

    // TODO method to bring the rollers into positon


    

    

    




}
