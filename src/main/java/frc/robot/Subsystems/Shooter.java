package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter;

public class Shooter extends SubsystemBase {
    
    private CANSparkMax RightNEO = new CANSparkMax(kShooter.RightNeoPort, MotorType.kBrushless);
    private CANSparkMax LeftNEO = new CANSparkMax(kShooter.LeftNeoPort, MotorType.kBrushless);

    public Shooter(){
        RightNEO.setInverted(kShooter.RightInvert);
        LeftNEO.setInverted(kShooter.LeftInvert);

    }

    // set shooter to a speed
    public void setShooterSpeed(double speed){
    }

    // TODO AMP vs Shooting speed change
}
