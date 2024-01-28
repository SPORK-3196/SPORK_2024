package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake;

public class Intake extends SubsystemBase {

private CANSparkMax IntakeAxis = new CANSparkMax(kIntake.IntakeAxisPort, MotorType.kBrushless);
private CANSparkMax IntakeNeo = new CANSparkMax(kIntake.IntakePort, MotorType.kBrushless);

public RelativeEncoder IntakeEncoder = IntakeAxis.getEncoder();

private SparkPIDController IntakePID;

public SparkLimitSwitch NoteIn = IntakeNeo.getForwardLimitSwitch(Type.kNormallyOpen);

    public Intake(){
    IntakeNeo.setIdleMode(kIntake.IntakeIdle);
    IntakeNeo.setInverted(kIntake.IntakeInvert);
    IntakeAxis.setIdleMode(kIntake.IntakeAxisIdle);
    IntakeAxis.setInverted(kIntake.IntakeAxisInvert);

    IntakePID = IntakeAxis.getPIDController();
    }

    // Intake game pieces
    public void grab(){
        IntakeNeo.set(kIntake.IntakeSpeed);
    }

    public void Keep(){
        IntakeNeo.set(0);
    }

    // Feed pieces into the shooter
    public void feed(){
        IntakeNeo.set(-kIntake.FeedSpeed);
    }

    // spit pieces out of the intake
    public void spit(){
        IntakeNeo.set(-kIntake.IntakeSpeed);
    }

    // To the window
    public Command FloorPos(){
        return this.runOnce(() -> IntakePID.setReference(kIntake.FloorPickup, ControlType.kPosition));
    }

    // To the wall
    public Command ShooterPos(){
        return this.runOnce(() -> IntakePID.setReference(kIntake.ShooterFeed, ControlType.kPosition));
    }

    // TODO add avalibility for old intake design
    

    // TODO add logic to stop intake when a piece is in

    
    // TODO add Logic to Feed note when intake is up and when the shooter is spooling up


    
}