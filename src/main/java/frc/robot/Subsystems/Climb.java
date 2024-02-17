package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;

public class Climb extends SubsystemBase{

    private CANSparkMax RightArm = new CANSparkMax(kClimber.RightArmPort, MotorType.kBrushless);
    private CANSparkMax LeftArm = new CANSparkMax(kClimber.LeftArmPort, MotorType.kBrushless);

    private SparkLimitSwitch RightArmBottom;
    private SparkLimitSwitch RightArmTop;

    private SparkLimitSwitch LeftArmBottom;
    private SparkLimitSwitch LeftArmTop;

    public RelativeEncoder RightArmEncoder;
    public RelativeEncoder LeftArmEncoder;
    
    public Climb(){
        RightArm.setIdleMode(kClimber.ClimberIdle);
        LeftArm.setIdleMode(kClimber.ClimberIdle);
        RightArm.setInverted(kClimber.RightArmInvert);
        LeftArm.setInverted(kClimber.LeftArmInvert);

        // Right Limit Switchs
        RightArmBottom = RightArm.getForwardLimitSwitch(Type.kNormallyClosed);
        RightArmTop = RightArm.getReverseLimitSwitch(Type.kNormallyClosed);

        // Left Limit Switch
        LeftArmBottom = LeftArm.getForwardLimitSwitch(Type.kNormallyOpen);
        LeftArmTop = RightArm.getReverseLimitSwitch(Type.kNormallyClosed);

        // Encoders
        LeftArmEncoder = LeftArm.getEncoder();
        RightArmEncoder = RightArm.getEncoder();
        
    }

    public void LeftUp(double speed){
        if(LeftArmTop.isPressed()){
            LeftArm.set(speed);
        }else{
            LeftStop();
        }
    }

    public void LeftDown(double speed){
        if (LeftArmBottom.isPressed()) {
            LeftArm.set(-speed);
        }else{
            LeftStop();
        }
    }

    public void LeftStop(){
        LeftArm.set(0);
    }

    public void RightUp(double speed){
        if (RightArmTop.isPressed()) {
            RightArm.set(-speed);
        }else{
            RightStop();
        }
    }

    public void RightDown(double speed){
        if (RightArmBottom.isPressed()) {
            RightArm.set(-speed);
        }else{
            RightStop();
        }
    }

    public void RightStop(){
        RightArm.set(0);
    }

    // TODO get driver input
}
