package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
        new Pose2d(4, 4, new Rotation2d()));
        speeds = new ChassisSpeeds();
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

    public Command RunPath(PathPlannerPath Path, boolean Alliance){
        return this.runOnce(() -> 
        new FollowPathHolonomic(
        Path,
        this::getPose,
        this::getChassisSpeeds,
        (speeds) -> Drive(speeds),
        kAuto.AutoConfig,
        () -> Alliance,
        this));
    }

    public void Drive(ChassisSpeeds dSpeeds){
        var targetStates = kSwerve.kinematics.toSwerveModuleStates(dSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kSwerve.MaxSpeed);

        setStates(targetStates);

        speeds = dSpeeds;
    }

    public ChassisSpeeds getChassisSpeeds(){
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

        speeds = new ChassisSpeeds(x * kSwerve.MaxSpeed, Y * kSwerve.MaxSpeed, z);

        return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyroAngle());
    }

    public Pose2d getPose(){
        return Pose.getEstimatedPosition();
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
