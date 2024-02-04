package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;

public class Climb extends SubsystemBase{

    private CANSparkMax RightArm = new CANSparkMax(kClimber.RightArmPort, MotorType.kBrushless);
    private CANSparkMax LeftArm = new CANSparkMax(kClimber.LeftArmPort, MotorType.kBrushless);

    private SparkLimitSwitch rightArmBottom;
    private SparkLimitSwitch rightArmTop;

    private SparkLimitSwitch LeftArmBottom;
    private SparkLimitSwitch LeftArmTop;
    
    public Climb(){
        RightArm.setIdleMode(kClimber.ClimberIdle);
        LeftArm.setIdleMode(kClimber.ClimberIdle);
        RightArm.setInverted(kClimber.RightArmInvert);
        LeftArm.setInverted(kClimber.LeftArmInvert);

        // Right Limit Switches
        rightArmBottom = RightArm.getForwardLimitSwitch(Type.kNormallyOpen);
        rightArmTop = RightArm.getReverseLimitSwitch(Type.kNormallyOpen);

        // Left Limit Switches
        LeftArmBottom = LeftArm.getForwardLimitSwitch(Type.kNormallyOpen);
        LeftArmTop = LeftArm.getReverseLimitSwitch(Type.kNormallyOpen);

    }

    public void LeftUp(double speed){
        if(!LeftArmTop.isPressed()){
            LeftArm.set(speed);
        }else{
            LeftStop();
        }
    }

    public void LeftDown(double speed){
        if (!LeftArmBottom.isPressed()) {
            LeftArm.set(-speed);
        }else{
            LeftStop();
        }
    }

    public void LeftStop(){
        LeftArm.set(0);
    }

    public void RightUp(double speed){
        if (!rightArmTop.isPressed()) {
            RightArm.set(-speed);
        }else{
            RightStop();
        }
    }

    public void RightDown(double speed){
        if (!rightArmBottom.isPressed()) {
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
