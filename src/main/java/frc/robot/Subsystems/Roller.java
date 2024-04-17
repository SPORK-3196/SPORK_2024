package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kRollerBars;

public class Roller extends SubsystemBase {

  private CANSparkMax RollerNEO = new CANSparkMax(
    kRollerBars.RollerNeoPort,
    MotorType.kBrushless
  );
  private CANSparkMax RollerNEO550 = new CANSparkMax(
    kRollerBars.RollerNeo550Port,
    MotorType.kBrushless
  );
  private RelativeEncoder RollerEncoder;
  private SparkPIDController RollerPID;

  public Roller() {
    RollerNEO.setIdleMode(kRollerBars.RollerIdle);
    RollerNEO.setInverted(kRollerBars.RollerInvert);
    RollerNEO.setSmartCurrentLimit(15);
    RollerNEO550.setInverted(kRollerBars.RollerNeo550Invert);
    RollerEncoder = RollerNEO.getEncoder();

    RollerPID = RollerNEO.getPIDController();
    RollerPID.setP(0.05);
  }

  public void RunRoller() {
    RollerNEO550.set(kRollerBars.RollerSpeed);
  }

  public void StopRoller() {
    RollerNEO550.set(0);
  }

  public void RollerBrake() {
    RollerNEO.setIdleMode(IdleMode.kBrake);
  }

  public void RollerCoast() {
    RollerNEO.setIdleMode(IdleMode.kCoast);
  }

  public double getRollerPos() {
    return RollerEncoder.getPosition();
  }

  public void RollerDown() {
    RollerPID.setReference(kRollerBars.RollerRefDown, ControlType.kPosition);
  }

  public void RollerUp() {
    RollerPID.setReference(kRollerBars.RollerRefUp, ControlType.kPosition);
  }
}
