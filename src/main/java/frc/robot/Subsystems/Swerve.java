package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

    private SwerveDriveOdometry Pose;
    private ChassisSpeeds chassisSpeedsRR;
    public Field2d field2d;

    public Swerve(){
        Pose = new SwerveDriveOdometry(
        kSwerve.kinematics,
        gyroAngle(), 
        getPositions(),
        new Pose2d(2,4, gyroAngle()));
        chassisSpeedsRR = new ChassisSpeeds();
        ConfigureBuilder();
        field2d = new Field2d();
        SmartDashboard.putData(field2d);
        field2d.setRobotPose(getPose());
    }

    public Rotation2d gyroAngle(){
        return Robot.gyro.getRotation2d();
    }

    @Override
    public void periodic(){
        updatePose();   
        field2d.setRobotPose(getPose());
        chassisSpeedsRR = kSwerve.kinematics.toChassisSpeeds(getStates());
    }  

    public void ZeroGyro(){
        Robot.gyro.reset();
    }

    public Rotation2d gyroRate(){
        return new Rotation2d(Robot.gyro.getRate());
        // Degrees/sec
    }

    public void Drive(ChassisSpeeds Speeds){
        var targetStates = kSwerve.kinematics.toSwerveModuleStates(Speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kSwerve.MaxSpeed);
        setStates(targetStates);
    }

    public void DriveRR(ChassisSpeeds speeds){
        var targetStates = kSwerve.kinematics.toSwerveModuleStates(speeds);
        setStates(targetStates);
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

        var ROspeeds = new ChassisSpeeds(x * kSwerve.MaxSpeed, Y * kSwerve.MaxSpeed, z * kSwerve.MaxSpeed);

        return ChassisSpeeds.fromFieldRelativeSpeeds(ROspeeds, gyroAngle());
    }

    public Pose2d getPose(){
        return Pose.getPoseMeters();
    }

    public void updatePose(){
        Pose.update(gyroAngle(), getPositions());
    }

    public void resetPose(Pose2d pose2d){
        Pose.resetPosition(gyroAngle(), getPositions(), pose2d);
    }

    public void Xconfig(){
        FL.setState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4 + kSwerve.FlOffset.getRadians())));
        FR.setState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4 + kSwerve.FrOffset.getRadians())));
        BL.setState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4 + kSwerve.BlOffset.getRadians())));
        BR.setState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4 + kSwerve.BrOffset.getRadians())));
    }

    public void ConfigureBuilder(){
        NamedCommands.registerCommand("Zero Gyro",new InstantCommand(() -> this.ZeroGyro()));
        // NamedCommands.registerCommand(getName(), null);
        // NamedCommands.registerCommand(getName(), null);
        // NamedCommands.registerCommand(getName(), null);

        // TODO Rotation
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            () -> chassisSpeedsRR,
            this::DriveRR,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5,0,0), 
                new PIDConstants(0,0,0), 
                Units.feetToMeters(2), 
                kSwerve.DRIVETRAIN_TRACKWIDTH_METERS/2, 
                new ReplanningConfig()),
                () -> {
                    var Alliance = DriverStation.getAlliance();
                    if(Alliance.isPresent()){
                        return Alliance.get() == DriverStation.Alliance.Red;
                    }
                        return true;
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
