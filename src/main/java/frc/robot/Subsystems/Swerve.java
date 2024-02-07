package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kSwerve;

public class Swerve extends SubsystemBase {
    
    public static Module FL = new Module(
    kSwerve.frontLeftSteer,
    kSwerve.frontLeftDrive,
    kSwerve.kFrontLeftDriveAbsoluteEncoderPort,
    kSwerve.FlOffset);
    public static Module FR = new Module(
    kSwerve.frontRightSteer,
    kSwerve.frontRightDrive,
    kSwerve.kFrontRightDriveAbsoluteEncoderPort,
    kSwerve.FrOffset);
    public static Module BL = new Module(
    kSwerve.backLeftSteer,
    kSwerve.backLeftDrive,
    kSwerve.kBackLeftDriveAbsoluteEncoderPort,
    kSwerve.BlOffset);
    public static Module BR = new Module(
    kSwerve.backRightSteer,
    kSwerve.backRightDrive,
    kSwerve.kBackRightDriveAbsoluteEncoderPort,
    kSwerve.BrOffset);

    private SwerveDrivePoseEstimator Pose;
    private ChassisSpeeds speeds;

    public Swerve(){
        Pose = new SwerveDrivePoseEstimator(kSwerve.kinematics,
        new Rotation2d(gyroAngle().getDegrees()),
        new SwerveModulePosition[]{
            FL.getPosition(),
            FR.getPosition(),
            BL.getPosition(),
            BR.getPosition()
        },
        new Pose2d(0, 0, new Rotation2d()));
        speeds = new ChassisSpeeds();
        ConfigureBuilder();
    }

    public Rotation2d gyroAngle(){
        return Robot.gyro.getRotation2d();
    }

    @Override
    public void periodic(){
        Pose.update(gyroAngle(), getPositions());
    }  

    public void ZeroGyro(){
        Robot.gyro.reset();
    }

    public Rotation2d gyroRate(){
        return new Rotation2d(Robot.gyro.getRate());
        // Degrees/sec
    }

    public void Drive(ChassisSpeeds dSpeeds){
        var targetStates = kSwerve.kinematics.toSwerveModuleStates(dSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kSwerve.MaxSpeed);
        setStates(targetStates);
    }

    public void DriveRR(ChassisSpeeds speeds){
        var targetStates = kSwerve.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kSwerve.MaxSpeed);
        setStates(targetStates);
    }

    public ChassisSpeeds getChassisSpeedsRR(){
        return speeds;
    }


    public Command teleDrive(
    DoubleSupplier translation,
    DoubleSupplier strafe,
    DoubleSupplier rotation){
        return this.run(
            () -> 
            Drive(Joystickcontrol(
                translation.getAsDouble(),
                strafe.getAsDouble(),
                rotation.getAsDouble())
            )
        );
    }

    private ChassisSpeeds Joystickcontrol(
        double x,
        double Y,
        double z
    ){
        if(Math.abs(x) <= kSwerve.kDeadband) x = 0;
        if(Math.abs(Y) <= kSwerve.kDeadband) Y = 0;
        if(Math.abs(z) <= kSwerve.kDeadband) z = 0;
        
        x = Math.copySign(x*x, x);
        Y = Math.copySign(Y*Y, Y);
        z = Math.copySign(z*z, z);

        var ROspeeds = new ChassisSpeeds(x * kSwerve.MaxSpeed, Y * kSwerve.MaxSpeed, z * kSwerve.MaxAngularSpeed);

        speeds = ROspeeds;
        
        return ChassisSpeeds.fromFieldRelativeSpeeds(ROspeeds, gyroAngle());
    }

    public Pose2d getPose(){
        return Pose.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose){
        Pose.resetPosition(gyroAngle(), getPositions(), Pose.getEstimatedPosition());
    }

    public void ConfigureBuilder(){
        AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getChassisSpeedsRR,
        this::DriveRR,
        kAuto.AutoConfig,
        () -> {
            var All = DriverStation.getAlliance();
            if (All.isPresent()) {
                return All.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this);
    }

    public SwerveModuleState[] getStates(){
        return new SwerveModuleState[]{
            FL.getstate(),
            FR.getstate(),
            BL.getstate(),
            BR.getstate()
        };
    }

    public SwerveModulePosition[] getPositions(){
        return new SwerveModulePosition[]{
            FL.getPosition(),
            FR.getPosition(),
            BL.getPosition(),
            BR.getPosition()
        };
    }

    public void setStates(SwerveModuleState[] state){
        FL.setState(state[0]);
        FR.setState(state[1]);
        BL.setState(state[2]);
        BR.setState(state[3]);
    }

}
