package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;
import frc.robot.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {

  public static Module FL = new Module(
    kSwerve.frontLeftSteer,
    kSwerve.frontLeftDrive,
    kSwerve.kFrontLeftDriveAbsoluteEncoderPort,
    kSwerve.FlOffset
  );
  public static Module FR = new Module(
    kSwerve.frontRightSteer,
    kSwerve.frontRightDrive,
    kSwerve.kFrontRightDriveAbsoluteEncoderPort,
    kSwerve.FrOffset
  );
  public static Module BL = new Module(
    kSwerve.backLeftSteer,
    kSwerve.backLeftDrive,
    kSwerve.kBackLeftDriveAbsoluteEncoderPort,
    kSwerve.BlOffset
  );
  public static Module BR = new Module(
    kSwerve.backRightSteer,
    kSwerve.backRightDrive,
    kSwerve.kBackRightDriveAbsoluteEncoderPort,
    kSwerve.BrOffset
  );

  private SwerveDrivePoseEstimator Pose;
  private ChassisSpeeds chassisSpeedsRR;
  //private LimelightHelpers.PoseEstimate LimeLightPoseMes;
  public Field2d field2d;

  public Swerve() {
    Pose =
      new SwerveDrivePoseEstimator(
        kSwerve.kinematics,
        gyroAngle(),
        getPositions(),
        new Pose2d()
      );
    chassisSpeedsRR = new ChassisSpeeds();
    ConfigureBuilder();
    field2d = new Field2d();
    SmartDashboard.putData(field2d);
    field2d.setRobotPose(getPose());
  }

  public Rotation2d gyroAngle() {
    return Robot.gyro.getRotation2d();
  }

  // if(LimelightHelpers.getCurrentPipelineIndex("") == 1){
  // var all = DriverStation.getAlliance();
  // if(all.isPresent()){
  //     if (all.get() == DriverStation.Alliance.Red){
  //         LimeLightPoseMes = LimelightHelpers.getBotPoseEstimate_wpiRed("");
  //     }else{
  //         LimeLightPoseMes = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
  //     }
  //     if (LimeLightPoseMes.tagCount >= 2 ) {
  //         Pose.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
  //         Pose.addVisionMeasurement(
  //             LimeLightPoseMes.pose,
  //             LimeLightPoseMes.timestampSeconds);
  //     }
  //     }
  // }

  @Override
  public void periodic() {
    updatePose();
    field2d.setRobotPose(getPose());
    chassisSpeedsRR = kSwerve.kinematics.toChassisSpeeds(getStates());
  }

  public void ZeroGyro() {
    Robot.gyro.reset();
  }

  public Rotation2d gyroRate() {
    return new Rotation2d(Robot.gyro.getRate());
    // Degrees/sec
  }

  public void Drive(ChassisSpeeds Speeds) {
    var targetStates = kSwerve.kinematics.toSwerveModuleStates(Speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kSwerve.MaxSpeed);
    setStates(targetStates);
  }

  public void DriveRR(ChassisSpeeds speeds) {
    var targetStates = kSwerve.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kSwerve.MaxSpeed);
    setStates(targetStates);
  }

  public Command teleDrive(
    DoubleSupplier translation,
    DoubleSupplier strafe,
    DoubleSupplier rotation,
    BooleanSupplier boost
  ) {
    return this.run(() ->
        Drive(
          Joystickcontrol(
            translation.getAsDouble(),
            strafe.getAsDouble(),
            rotation.getAsDouble(),
            boost.getAsBoolean()
          )
        )
      );
  }

  private ChassisSpeeds Joystickcontrol(
    double xTrans,
    double yTrans,
    double Rotation,
    boolean boost
  ) {
    if (Math.abs(xTrans) <= kSwerve.kDeadband) xTrans = 0;
    if (Math.abs(yTrans) <= kSwerve.kDeadband) yTrans = 0;
    if (Math.abs(Rotation) <= kSwerve.kDeadband) Rotation = 0;

    xTrans = Math.copySign(xTrans * xTrans, xTrans);
    yTrans = Math.copySign(yTrans * yTrans, yTrans);
    Rotation = Math.copySign(Rotation * Rotation, Rotation);

    var ROspeeds = new ChassisSpeeds();

    if(!boost) {
    ROspeeds = new ChassisSpeeds(
      xTrans * kSwerve.MaxSpeed,
      yTrans * kSwerve.MaxSpeed,
      Rotation * kSwerve.MaxSpeed
    );
    }else{
    ROspeeds = new ChassisSpeeds(
      xTrans * kSwerve.MaxSpeed,
      yTrans * kSwerve.MaxSpeed,
      Rotation * kSwerve.MaxAngularSpeed
    );
    }
  
    return ChassisSpeeds.fromFieldRelativeSpeeds(ROspeeds, gyroAngle());
  }

  public Pose2d getPose() {
    return Pose.getEstimatedPosition();
  }

  public void updatePose() {
    Pose.update(gyroAngle(), getPositions());
  }

  public void resetPose(Pose2d pose2d) {
    Pose.resetPosition(gyroAngle(), getPositions(), pose2d);
  }

  public void Xconfig() {
    FL.setState(
      new SwerveModuleState(
        0,
        new Rotation2d(Math.PI / 4 + kSwerve.FlOffset.getRadians())
      )
    );
    FR.setState(
      new SwerveModuleState(
        0,
        new Rotation2d(Math.PI / 4 + kSwerve.FrOffset.getRadians())
      )
    );
    BL.setState(
      new SwerveModuleState(
        0,
        new Rotation2d(Math.PI / 4 + kSwerve.BlOffset.getRadians())
      )
    );
    BR.setState(
      new SwerveModuleState(
        0,
        new Rotation2d(Math.PI / 4 + kSwerve.BrOffset.getRadians())
      )
    );
  }

  public void ConfigureBuilder() {
    NamedCommands.registerCommand(
      "Zero Gyro",
      new InstantCommand(() -> this.ZeroGyro())
    );
    NamedCommands.registerCommand(
      "Intake Down",
      new InstantCommand(() -> Robot.mIntake.FloorPos())
    );
    NamedCommands.registerCommand(
      "Intake Up",
      new InstantCommand(() -> Robot.mIntake.ShooterPos())
    );
    NamedCommands.registerCommand(
      "Run Intake",
      new InstantCommand(() -> Robot.mIntake.grab())
    );
    NamedCommands.registerCommand(
      "Stop Intake",
      new InstantCommand(() -> Robot.mIntake.Keep())
    );
    NamedCommands.registerCommand(
      "Feed Intake",
      new InstantCommand(() -> Robot.mIntake.feed())
    );
    NamedCommands.registerCommand(
      "Shooter 80%",
      new InstantCommand(() -> Robot.mShooter.setShooterSpeed(0.8))
    );
    NamedCommands.registerCommand(
      "Shooter 100%",
      new InstantCommand(() -> Robot.mShooter.setShooterSpeed(1))
    );
    NamedCommands.registerCommand(
      "Shooter 0%",
      new InstantCommand(() -> Robot.mShooter.setShooterSpeed(0))
    );

    // TODO Rotation  
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      () -> chassisSpeedsRR,
      chassisSpeedsRR -> DriveRR(chassisSpeedsRR),
      new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0),
        new PIDConstants(0, 0, 0),
        Units.feetToMeters(2),
        kSwerve.DRIVETRAIN_TRACKWIDTH_METERS / 2,
        new ReplanningConfig(false, false)
      ),
      () -> {
        var Alliance = DriverStation.getAlliance();
        if (Alliance.isPresent()) {
          return Alliance.get() == DriverStation.Alliance.Red;
        }
        return true;
      },
      this
    );
  }

  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      FL.getstate(),
      FR.getstate(),
      BL.getstate(),
      BR.getstate(),
    };
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      FL.getPosition(),
      FR.getPosition(),
      BL.getPosition(),
      BR.getPosition(),
    };
  }

  public void setStates(SwerveModuleState[] state) {
    FL.setState(state[0]);
    FR.setState(state[1]);
    BL.setState(state[2]);
    BR.setState(state[3]);
  }
}
