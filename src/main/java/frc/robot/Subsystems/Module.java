package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;

public class Module extends SubsystemBase {

  public SwerveModuleState State;
  private Rotation2d offset;

  public SparkPIDController DrivePID;
  public PIDController AzumuthPID;

  public CANSparkMax AzumuthNEO;
  public CANSparkMax DriveNEO;

  private SimpleMotorFeedforward OpenLoopFF = new SimpleMotorFeedforward(
    0.046,
    2.67,
    0.113
  );

  public RelativeEncoder DriveEncoder;

  public CANcoder absoluteEncoder;
  private CANcoderConfiguration config;

  public Module(
    int TurnNeoID,
    int DriveID,
    int absoluteEncoderID,
    Rotation2d offset
  ) {
    this.offset = offset;
    State = new SwerveModuleState();
    config = new CANcoderConfiguration();

    AzumuthNEO = new CANSparkMax(TurnNeoID, MotorType.kBrushless);
    AzumuthNEO.setInverted(true);
    AzumuthNEO.setIdleMode(IdleMode.kBrake);
    AzumuthNEO.setSmartCurrentLimit(15);
    AzumuthNEO.enableVoltageCompensation(12);

    DriveNEO = new CANSparkMax(DriveID, MotorType.kBrushless);
    DriveNEO.setIdleMode(IdleMode.kBrake);
    DriveNEO.setInverted(true);
    DriveNEO.setSmartCurrentLimit(40);
    DriveNEO.enableVoltageCompensation(12);

    DriveEncoder = DriveNEO.getEncoder();
    DriveEncoder.setPosition(0);

    absoluteEncoder = new CANcoder(absoluteEncoderID);
    var absoluteEncoderConfigu = absoluteEncoder.getConfigurator();
    config.MagnetSensor.AbsoluteSensorRange =
      AbsoluteSensorRangeValue.Unsigned_0To1;
    absoluteEncoderConfigu.apply(config);

    DrivePID = DriveNEO.getPIDController();
    DrivePID.setP(1);

    AzumuthPID = new PIDController(3, 0, 0.1);
    AzumuthPID.enableContinuousInput(0, 1);
  }

  public void setState(SwerveModuleState dState, boolean IsClosedloop) {
    dState = SwerveModuleState.optimize(dState, getCANangle());

    if(IsClosedloop){
      DrivePID.setReference(dState.speedMetersPerSecond,
      ControlType.kVelocity,
      0,
      OpenLoopFF.calculate(dState.speedMetersPerSecond));
    }else{
      DriveNEO.set(OpenLoopFF.calculate(dState.speedMetersPerSecond));
    }

    if (Math.abs(dState.speedMetersPerSecond) > kSwerve.MaxSpeed * 0.01) {
      var out = AzumuthPID.calculate(
        getCANangle().getRotations(),
        dState.angle.getRotations()
      );
      AzumuthNEO.set(out);
    } else {
      AzumuthNEO.set(0);
    }
  }

  public Rotation2d getCANangle() {
    return Rotation2d.fromRotations(
      absoluteEncoder.getAbsolutePosition().getValueAsDouble() -
      offset.getRotations()
    );
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      DriveEncoder.getPosition() * 1 / 28 ,
      getCANangle()
    );
  }

  public SwerveModuleState getstate() {
    return new SwerveModuleState(DriveEncoder.getVelocity(), getCANangle());
  }
}
