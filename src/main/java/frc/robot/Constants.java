package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    //TODO constants need to be tested and finalised

    //TODO get others to work on this

    public static class kVision{
        







    }


    public static class kAuto {
      public static HolonomicPathFollowerConfig AutoConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(2,0,0.1),
        new PIDConstants(2, 0, 0.1),
        kSwerve.MaxSpeed,
        Units.inchesToMeters(12),
        new ReplanningConfig(false, false)
        );
    }

    public static class kShooter {

        //Setup 
        public static int RightNeoPort = 10;
        public static int LeftNeoPort = 11;
        public static boolean RightInvert = true;
        public static boolean LeftInvert = false;
        public static IdleMode ShooterIdleMode = IdleMode.kBrake; // may change

        // Targets
        public static double IdleSpeed = 0.2; // can change
        public static double TargetVelocity = 0.5; // M/s

    }

    public static class kIntake {
        // Setup
        public static int IntakePort = 9;
        public static int IntakeAxisPort = 14;
        public static boolean IntakeInvert = false;
        public static boolean IntakeAxisInvert = false;
        public static IdleMode IntakeAxisIdle = IdleMode.kBrake;
        public static IdleMode IntakeIdle = IdleMode.kBrake;

        // Positions
        public static double FloorPickup = 0.0;  //TODO change these values
        public static double ShooterFeed = 0.0; 

        // Speeds
        public static double IntakeSpeed = 0.5;
        public static double FeedSpeed = 0.2;
        // PID Values
        public static double kP = 0;
        public static double KI = 0;
        public static double kD = 0;
    }


    public static class kRollerBars {
        public static int RollerNeoPort = 9; // TODO change 
        public static double RollerSpeed = 0.5; // test rollers
        public static boolean RollerInvert = false;
        public static IdleMode RollerIdle = IdleMode.kBrake;
        
    }
    

    public static class kClimber {
        //setup
        public static int RightArmPort = 12;
        public static int LeftArmPort = 13;
        public static boolean RightArmInvert = false;
        public static boolean LeftArmInvert = false;
        public static IdleMode ClimberIdle = IdleMode.kBrake;


        // Speeds
        public static double ClimbSpeed = 0.5;

    }

    public static class kSwerve {
        private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24);
        private static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24);
        
        public static final double kDeadband = 0.1;
        
        public static final double MaxAngularSpeed = Units.feetToMeters(5); // m/s
        public static final double MaxSpeed = Units.feetToMeters(5);  // m/s
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double WheelCircumference = wheelDiameter * Math.PI;
        public static final double RotationGearRatio = (150/7)/1;
        public static final double DriveGearRatio = 8.14/1;

        //Spark Max IDs 
        public static final int frontLeftDrive = 1;
        public static final int frontLeftSteer = 5;
        public static final int backLeftDrive = 2;
        public static final int backLeftSteer = 6;
        public static final int backRightDrive = 4;
        public static final int backRightSteer = 8;
        public static final int frontRightDrive = 3;
        public static final int frontRightSteer = 7;

        //CRTE CANcoder IDs
        public static int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static int kFrontRightDriveAbsoluteEncoderPort = 4;
        public static int kBackRightDriveAbsoluteEncoderPort = 1;

        // Swerve module Offsets
        public static Rotation2d FlOffset = Rotation2d.fromRotations(0.72);
        public static Rotation2d FrOffset = Rotation2d.fromRotations(0.31);
        public static Rotation2d BlOffset = Rotation2d.fromRotations(0.08);
        public static Rotation2d BrOffset = Rotation2d.fromRotations(0.96);
        

        public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
                // Front left
                new Translation2d(kSwerve.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                kSwerve.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                // Front right
                new Translation2d(kSwerve.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                        -kSwerve.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                // Back left
                new Translation2d(-kSwerve.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                kSwerve.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                // Back right
                new Translation2d(-kSwerve.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                        -kSwerve.DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
        );
    }
}
