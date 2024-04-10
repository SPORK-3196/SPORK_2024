package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;

public class Climb extends SubsystemBase {

  private CANSparkMax RightArm = new CANSparkMax(
    kClimber.RightArmPort,
    MotorType.kBrushless
  );
  private CANSparkMax LeftArm = new CANSparkMax(
    kClimber.LeftArmPort,
    MotorType.kBrushless
  );

  public RelativeEncoder RightArmEncoder;
  public RelativeEncoder LeftArmEncoder;

  public Climb() {
    RightArm.setIdleMode(kClimber.ClimberIdle);
    LeftArm.setIdleMode(kClimber.ClimberIdle);
    RightArm.setInverted(kClimber.RightArmInvert);
    LeftArm.setInverted(kClimber.LeftArmInvert);

    // Encoders
    LeftArmEncoder = LeftArm.getEncoder();
    RightArmEncoder = RightArm.getEncoder();
    LeftArmEncoder = LeftArm.getEncoder();
  }

  public void LeftUp(double speed) {
    LeftArm.set(speed);
  }

  public void LeftDown(double speed) {
    LeftArm.set(-speed);
  }

  public void LeftStop() {
    LeftArm.set(0);
  }

  public void RightUp(double speed) {
    RightArm.set(speed);
  }

  public void RightDown(double speed) {
    RightArm.set(-speed);
  }

  public void RightStop() {
    RightArm.set(0);
  }
}
