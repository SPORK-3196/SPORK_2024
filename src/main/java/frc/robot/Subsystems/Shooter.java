package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter;

public class Shooter extends SubsystemBase {
    
    private CANSparkMax RightNEO = new CANSparkMax(kShooter.RightNeoPort, MotorType.kBrushless);
    private CANSparkMax LeftNEO = new CANSparkMax(kShooter.LeftNeoPort, MotorType.kBrushless);

    public Shooter(){
        RightNEO.setIdleMode(kShooter.ShooterIdleMode);
        LeftNEO.setIdleMode(kShooter.ShooterIdleMode);
        RightNEO.setInverted(kShooter.RightInvert);
        LeftNEO.setInverted(kShooter.LeftInvert);
    }

    // set shooter to a speed
    public void setShooterSpeed(double speed){


    // TODO set the speed of the motors to a vaiable "speed"

    }

    // TODO set shooters to move constanly to reduce shoot time 

    // TODO AMP vs Shooting speed change

    
}
