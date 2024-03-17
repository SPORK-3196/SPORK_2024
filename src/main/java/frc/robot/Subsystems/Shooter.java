package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
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

        RightNEO.enableVoltageCompensation(12);
        LeftNEO.enableVoltageCompensation(12);

        RightNEO.setSmartCurrentLimit(50);
    }

    // set shooter to a speed
    public void setShooterSpeed(double volts){
            RightNEO.setVoltage(volts);
            LeftNEO.setVoltage(volts);
    }

    public Command RunShooter(DoubleSupplier TriggerAxis){
        if(TriggerAxis.getAsDouble() >= 0.1){
            return this.runEnd(() -> setShooterSpeed(kShooter.ShooterVolts), () -> ShooterIdle());
        }
        return null;
    }

    public double getShooterSpeed(){
        return RightNEO.get() + LeftNEO.get() / 2 ; 
    }

    public void ShooterIdle(){
        setShooterSpeed(kShooter.IdleSpeed);
    }
    
}
