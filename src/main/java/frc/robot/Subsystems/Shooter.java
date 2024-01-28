package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter;

public class Shooter extends SubsystemBase {
    
    private CANSparkMax RightNEO = new CANSparkMax(kShooter.RightNeoPort, MotorType.kBrushless);
    private CANSparkMax LeftNEO = new CANSparkMax(kShooter.LeftNeoPort, MotorType.kBrushless);
    private RelativeEncoder RightEncoder = RightNEO.getEncoder();
    private RelativeEncoder LeftEncoder = LeftNEO.getEncoder();

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

    public boolean isShooterToSpeed(){
        if(RightEncoder.getVelocity() >= kShooter.TargetVelocity && LeftEncoder.getVelocity() >= kShooter.TargetVelocity){
            return true;
        }else{
            return false;
        }
    }

    // TODO set shooters to move constanly to reduce shoot time 

    // TODO AMP vs Shooting speed change

    
}
