package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake;

public class Intake extends SubsystemBase {

  private CANSparkMax IntakeAxis = new CANSparkMax(
    kIntake.IntakeAxisPort,
    MotorType.kBrushless
  );
  private CANSparkMax IntakeNeo = new CANSparkMax(
    kIntake.IntakePort,
    MotorType.kBrushless
  );

  public RelativeEncoder IntakeEncoder = IntakeAxis.getEncoder();

  public SparkPIDController IntakePID;

  // Limit switches
  public DigitalInput NoteIn = new DigitalInput(0);
  public SparkLimitSwitch SpeakerLimit = IntakeAxis.getReverseLimitSwitch(
    Type.kNormallyClosed
  );
  public SparkLimitSwitch FloorStop = IntakeAxis.getForwardLimitSwitch(
    Type.kNormallyClosed
  );

  public Intake() {
    IntakeNeo.setIdleMode(kIntake.IntakeIdle);
    IntakeNeo.setInverted(kIntake.IntakeInvert);
    IntakeAxis.setIdleMode(kIntake.IntakeAxisIdle);
    IntakeAxis.setInverted(kIntake.IntakeAxisInvert);

    IntakePID = IntakeAxis.getPIDController();
    IntakePID.setP(kIntake.kP);
    IntakePID.setI(kIntake.KI);
    IntakePID.setD(kIntake.kD);
  }

  // Intake Notes
  public void grab() {
    IntakeNeo.set(kIntake.IntakeSpeed);
  }

  public void Keep() {
    IntakeNeo.set(0);
  }

  public double getPos() {
    return IntakeEncoder.getPosition();
  }

  // Feed Notes into the shooter
  public void feed() {
    IntakeNeo.setIdleMode(IdleMode.kCoast);
    IntakeNeo.set(kIntake.FeedSpeed);
  }

  // spit Notes out of the intake
  public void spit() {
    IntakeNeo.set(-kIntake.IntakeSpeed);
  }

  public boolean isRunning() {
    return IntakeNeo.get() == 0;
  }

  public double getSpeed() {
    return IntakeNeo.get();
  }

  // To the window
  public void FloorPos() {
    IntakePID.setP(0.08);
    IntakePID.setReference(kIntake.FloorPickup, ControlType.kPosition);
  }

  public Command FeedShooter() {
    return this.runEnd(() -> this.feed(), () -> this.Keep());
  }

  // To the wall
  public void ShooterPos() {
    IntakePID.setP(kIntake.kP);
    IntakePID.setReference(kIntake.ShooterPos, ControlType.kPosition);
  }

  public void spitPos() {
    IntakePID.setReference(kIntake.spitPos, ControlType.kPosition);
  }

  // Honestly this seems kinda stupid but it might be better than seting the motor to 0
  public Command Stop() {
    return this.runOnce(() ->
        IntakePID.setReference(
          IntakeEncoder.getPosition(),
          ControlType.kPosition
        )
      );
  }
}
