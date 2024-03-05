package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    //TODO constants need to be tested and finalised

    public static class kVision{



    }


    public static class kAuto {

    }

    public static class kShooter {

        //Setup 
        public static int RightNeoPort = 25;
    public static int LeftNeoPort = 26;
        public static boolean RightInvert = false;
        public static boolean LeftInvert = true;
        public static IdleMode ShooterIdleMode = IdleMode.kCoast; // may change

        // Targets
        public static double ShootSpeed = 1; // 0.28;
        public static double ampSpeed = 0.26; // will change
        public static double IdleSpeed = 0.1; // will change
        public static double TargetVelocity = 0.5;

    }

    public static class kIntake {

        // Setup
        public static int IntakePort = 23;
        public static int IntakeAxisPort = 24;
        public static boolean IntakeInvert = false;
        public static boolean IntakeAxisInvert = false;
        public static IdleMode IntakeAxisIdle = IdleMode.kCoast;
        public static IdleMode IntakeIdle = IdleMode.kBrake;

        // Positions
        public static double FloorPickup = 49;  //TODO change these values
        public static double ShooterPos = 0.0; 
        public static double spitPos = 24.0; 

        // Speeds
        public static double IntakeSpeed = -0.5;
        public static double FeedSpeed = 1;
        
        // PID Values
        public static double kP = 0.03;
        public static double KI = 0;
        public static double kD = 0.0;
    }


    public static class kRollerBars {
        public static int RollerNeoPort = 9; // TODO change 
        public static double RollerSpeed = 0.5; // test rollers
        public static boolean RollerInvert = false;
        public static IdleMode RollerIdle = IdleMode.kBrake;
        
    }
    

    public static class kClimber {
        //setup
        public static int RightArmPort = 19;
        public static int LeftArmPort = 16;
        public static boolean RightArmInvert = false;
        public static boolean LeftArmInvert = false;
        public static IdleMode ClimberIdle = IdleMode.kBrake;


        // Speeds
        public static double ClimbSpeed = 0.45;

    }

    public static class kSwerve {
        private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24);
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24);
        
        public static final double kDeadband = 0.1;
        
        public static final double MaxAngularSpeed = Units.feetToMeters(4); // m/s
        public static final double MaxSpeed = Units.feetToMeters(5);  // m/s
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double WheelCircumference = wheelDiameter * Math.PI;
        public static final double RotationGearRatio = (150/7)/1;
        public static final double DriveGearRatio = 8.14/1;

        //Spark Max IDs 
        public static final int frontLeftDrive = 8;
        public static final int frontLeftSteer = 5;
        public static final int backLeftDrive = 10;
        public static final int backLeftSteer = 15;
        public static final int backRightDrive = 20;
        public static final int backRightSteer = 21;
        public static final int frontRightDrive = 18;
        public static final int frontRightSteer = 17;

        //CRTE CANcoder IDs
        public static int kFrontLeftDriveAbsoluteEncoderPort = 4;
        public static int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static int kBackRightDriveAbsoluteEncoderPort = 3;

        // Swerve module Offsets
        public static Rotation2d FlOffset = Rotation2d.fromRotations(0.75);
        public static Rotation2d FrOffset = Rotation2d.fromRotations(0.98);
        public static Rotation2d BlOffset = Rotation2d.fromRotations(0.15);
        public static Rotation2d BrOffset = Rotation2d.fromRotations(0.41);

        // public static Rotation2d FlOffset = Rotation2d.fromRotations(0.72);
        // public static Rotation2d FrOffset = Rotation2d.fromRotations(0.31);
        // public static Rotation2d BlOffset = Rotation2d.fromRotations(0.08);
        // public static Rotation2d BrOffset = Rotation2d.fromRotations(0.96);
        

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
