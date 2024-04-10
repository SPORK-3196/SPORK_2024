// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Climber.ArmsDown;
import frc.robot.Commands.Climber.ArmsUp;
import frc.robot.Commands.Intake.AutoIntake;
import frc.robot.Commands.Intake.IntakeFeed;
import frc.robot.Commands.Intake.IntakeGrab;
import frc.robot.Commands.Intake.Vomit;
import frc.robot.Commands.LEDcolors;
import frc.robot.Commands.RunAmp;
import frc.robot.Commands.Shooter.RunShooter;
import frc.robot.Constants.kClimber;
import frc.robot.OI.oDriver;
import frc.robot.OI.oIntake;
import frc.robot.OI.oPDH;
import frc.robot.OI.oSecondary;
import frc.robot.OI.oShooter;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lighting;
import frc.robot.Subsystems.Roller;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  public static AHRS gyro = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kMXP);

  // Subsystem Initalising
  public SendableChooser<Command> autoChooser;
  public static Intake mIntake = new Intake();
  public static Climb mClimb = new Climb();
  public static Shooter mShooter = new Shooter();
  public static Lighting mLighting = new Lighting();
  public static Roller mRoller = new Roller();
  public static Swerve mSwerve = new Swerve();
  // Controllers
  public static XboxController driver = new XboxController(0);
  public static XboxController secondary = new XboxController(1);

  public static DigitalInput NoteIn = new DigitalInput(1);

  // Driver buttons
  public JoystickButton driver_a_Button = new JoystickButton(
    driver,
    XboxController.Button.kA.value
  );
  public JoystickButton driver_b_Button = new JoystickButton(
    driver,
    XboxController.Button.kB.value
  );
  public JoystickButton driver_x_Button = new JoystickButton(
    driver,
    XboxController.Button.kX.value
  );
  public JoystickButton driver_y_Button = new JoystickButton(
    driver,
    XboxController.Button.kY.value
  );

  // Driver bumpers and triggers
  public JoystickButton driver_left_Bumper = new JoystickButton(
    driver,
    XboxController.Button.kLeftBumper.value
  );
  public JoystickButton driver_Right_Bumper = new JoystickButton(
    driver,
    XboxController.Button.kRightBumper.value
  );
  public JoystickButton driver_Left_Trigger = new JoystickButton(
    driver,
    XboxController.Axis.kLeftTrigger.value
  );
  public JoystickButton driver_Right_Trigger = new JoystickButton(
    driver,
    XboxController.Axis.kRightTrigger.value
  );

  // Driver misc buttons
  public JoystickButton driver_Start = new JoystickButton(
    driver,
    XboxController.Button.kStart.value
  );
  public JoystickButton driver_Back = new JoystickButton(
    driver,
    XboxController.Button.kBack.value
  );
  public JoystickButton driver_RJSD = new JoystickButton(
    driver,
    XboxController.Button.kRightStick.value
  );
  public JoystickButton driver_LJSD = new JoystickButton(
    driver,
    XboxController.Button.kLeftStick.value
  );

  // Secondary buttons
  public JoystickButton secondary_a_Button = new JoystickButton(
    secondary,
    XboxController.Button.kA.value
  );
  public JoystickButton secondary_b_Button = new JoystickButton(
    secondary,
    XboxController.Button.kB.value
  );
  public JoystickButton secondary_x_Button = new JoystickButton(
    secondary,
    XboxController.Button.kX.value
  );
  public JoystickButton secondary_y_Button = new JoystickButton(
    secondary,
    XboxController.Button.kY.value
  );

  // Secondary POVS

  // Secondary bumpers and triggers
  public JoystickButton secondary_left_Bumper = new JoystickButton(
    secondary,
    XboxController.Button.kLeftBumper.value
  );
  public JoystickButton secondary_Right_Bumper = new JoystickButton(
    secondary,
    XboxController.Button.kRightBumper.value
  );
  public JoystickButton secondary_Left_Trigger = new JoystickButton(
    secondary,
    XboxController.Axis.kLeftTrigger.value
  );
  public JoystickButton secondary_Right_Trigger = new JoystickButton(
    secondary,
    XboxController.Axis.kRightTrigger.value
  );

  // Secondary misc buttons
  public JoystickButton secondary_Start = new JoystickButton(
    secondary,
    XboxController.Button.kStart.value
  );
  public JoystickButton secondary_Back = new JoystickButton(
    secondary,
    XboxController.Button.kBack.value
  );

  public JoystickButton secondary_RJSD = new JoystickButton(
    secondary,
    XboxController.Button.kRightStick.value
  );
  public JoystickButton secondary_LJSD = new JoystickButton(
    secondary,
    XboxController.Button.kLeftStick.value
  );

  public static boolean NoteIN;
  public static boolean boost;

  // Camera
  UsbCamera Cam = CameraServer.startAutomaticCapture(0);
  UsbCamera Cam2 = CameraServer.startAutomaticCapture(1);
  PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

  @Override
  public void robotInit() {
    boost = true;
    mSwerve.setDefaultCommand(
      mSwerve.teleDrive(
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX(),
        () -> boost
      )
    );
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);


    Cam.setFPS(28);
    Cam.setResolution(244, 244);
    Cam2.setFPS(28);
    Cam2.setResolution(244, 244);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    NoteIN = NoteIn.get();

    SmartDashboard.putBoolean("Note", !NoteIN);
    SmartDashboard.putBoolean("slow", !driver.getRightStickButton());

    // Driver SmartDashboard output
    if (driver.isConnected()) {
      oDriver.a_Button = driver.getAButton();
      oDriver.b_Button = driver.getBButton();
      oDriver.x_Button = driver.getXButton();
      oDriver.y_Button = driver.getYButton();

      oDriver.kBack = driver.getBackButton();
      oDriver.kStart = driver.getStartButton();

      oDriver.kRightBumper = driver.getRightBumper();
      oDriver.kLeftBumper = driver.getLeftBumper();

      oDriver.kRightTrigger = driver.getRightTriggerAxis();
      oDriver.kLeftTrigger = driver.getLeftTriggerAxis();
    }

    // Secondary SmartDashboard output
    if (secondary.isConnected()) {
      oSecondary.a_Button = secondary.getAButton();
      oSecondary.b_Button = secondary.getBButton();
      oSecondary.x_Button = secondary.getXButton();
      oSecondary.y_Button = secondary.getYButton();

      oSecondary.kBack = secondary.getBackButton();
      oSecondary.kStart = secondary.getStartButton();

      oSecondary.kRJSD = secondary.getRightStickButton();
      oSecondary.kLJSD = secondary.getLeftStickButton();

      oSecondary.kRightBumper = secondary.getRightBumper();
      oSecondary.kLeftBumper = secondary.getLeftBumper();

      oSecondary.kRightTrigger = secondary.getRightTriggerAxis();
      oSecondary.kLeftTrigger = secondary.getLeftTriggerAxis();

      oSecondary.kPOV = secondary.getPOV();
    }

    oPDH.Channel_0 = PDH.getCurrent(0);
    oPDH.Channel_1 = PDH.getCurrent(1);
    oPDH.Channel_2 = PDH.getCurrent(2);
    oPDH.Channel_3 = PDH.getCurrent(3);
    oPDH.Channel_4 = PDH.getCurrent(4);
    oPDH.Channel_5 = PDH.getCurrent(5);
    oPDH.Channel_6 = PDH.getCurrent(6);
    oPDH.Channel_7 = PDH.getCurrent(7);
    oPDH.Channel_8 = PDH.getCurrent(8);
    oPDH.Channel_9 = PDH.getCurrent(9);
    oPDH.Channel_10 = PDH.getCurrent(10);
    oPDH.Channel_11 = PDH.getCurrent(11);
    oPDH.Channel_12 = PDH.getCurrent(12);
    oPDH.Channel_13 = PDH.getCurrent(13);
    oPDH.Channel_14 = PDH.getCurrent(14);
    oPDH.Channel_15 = PDH.getCurrent(15);
    oPDH.Channel_16 = PDH.getCurrent(16);
    oPDH.Channel_17 = PDH.getCurrent(17);
    oPDH.Channel_18 = PDH.getCurrent(18);
    oPDH.Channel_19 = PDH.getCurrent(19);
    oPDH.Channel_20 = PDH.getCurrent(20);
    oPDH.Channel_21 = PDH.getCurrent(21);
    oPDH.Channel_22 = PDH.getCurrent(22);
    oPDH.Channel_23 = PDH.getCurrent(23);

    oPDH.kChannel_0.setDouble(oPDH.Channel_0);
    oPDH.kChannel_1.setDouble(oPDH.Channel_1);
    oPDH.kChannel_2.setDouble(oPDH.Channel_2);
    oPDH.kChannel_3.setDouble(oPDH.Channel_3);
    oPDH.kChannel_4.setDouble(oPDH.Channel_4);
    oPDH.kChannel_5.setDouble(oPDH.Channel_5);
    oPDH.kChannel_6.setDouble(oPDH.Channel_6);
    oPDH.kChannel_7.setDouble(oPDH.Channel_7);
    oPDH.kChannel_8.setDouble(oPDH.Channel_8);
    oPDH.kChannel_9.setDouble(oPDH.Channel_9);
    oPDH.kChannel_10.setDouble(oPDH.Channel_10);
    oPDH.kChannel_11.setDouble(oPDH.Channel_11);
    oPDH.kChannel_12.setDouble(oPDH.Channel_12);
    oPDH.kChannel_13.setDouble(oPDH.Channel_13);
    oPDH.kChannel_14.setDouble(oPDH.Channel_14);
    oPDH.kChannel_15.setDouble(oPDH.Channel_15);
    oPDH.kChannel_16.setDouble(oPDH.Channel_16);
    oPDH.kChannel_17.setDouble(oPDH.Channel_17);
    oPDH.kChannel_18.setDouble(oPDH.Channel_18);
    oPDH.kChannel_19.setDouble(oPDH.Channel_19);
    oPDH.kChannel_20.setDouble(oPDH.Channel_20);
    oPDH.kChannel_21.setDouble(oPDH.Channel_21);
    oPDH.kChannel_22.setDouble(oPDH.Channel_22);
    oPDH.kChannel_23.setDouble(oPDH.Channel_23);

    // stream to SmartDashboard if DriverStation is not in game
    if (!DriverStation.isFMSAttached()) {
      oDriver.a_Button_Entry.setBoolean(oDriver.a_Button);
      oDriver.b_Button_Entry.setBoolean(oDriver.b_Button);
      oDriver.x_Button_Entry.setBoolean(oDriver.x_Button);
      oDriver.y_Button_Entry.setBoolean(oDriver.y_Button);

      oDriver.kBack_Entry.setBoolean(oDriver.kBack);
      oDriver.kStart_Entry.setBoolean(oDriver.kStart);

      oDriver.kLeftBumper_Entry.setBoolean(oDriver.kLeftBumper);
      oDriver.kRightBumper_Entry.setBoolean(oDriver.kRightBumper);

      oDriver.kLeftTrigger_Entry.setDouble(oDriver.kLeftTrigger);
      oDriver.kRightTrigger_Entry.setDouble(oDriver.kRightTrigger);

      oSecondary.a_Button_Entry.setBoolean(oSecondary.a_Button);
      oSecondary.b_Button_Entry.setBoolean(oSecondary.b_Button);
      oSecondary.x_Button_Entry.setBoolean(oSecondary.x_Button);
      oSecondary.y_Button_Entry.setBoolean(oSecondary.y_Button);

      oSecondary.kBack_Entry.setBoolean(oSecondary.kBack);
      oSecondary.kStart_Entry.setBoolean(oSecondary.kStart);

      oSecondary.kRJSD_Entry.setBoolean(oSecondary.kRJSD);
      oSecondary.kLJSD_Entry.setBoolean(oSecondary.kLJSD);

      oSecondary.kLeftBumper_Entry.setBoolean(oSecondary.kLeftBumper);
      oSecondary.kRightBumper_Entry.setBoolean(oSecondary.kRightBumper);

      oSecondary.kLeftTrigger_Entry.setDouble(oSecondary.kLeftTrigger);
      oSecondary.kRightTrigger_Entry.setDouble(oSecondary.kRightTrigger);

      oSecondary.kPOV_Entry.setDouble(oSecondary.kPOV);
    }

    if (!DriverStation.isFMSAttached()) {
      oIntake.IntakePos = mIntake.getPos();
      oIntake.IntakeRun = !mIntake.isRunning();
      oIntake.IntakeSpeed = mIntake.getSpeed();
      oIntake.LimitDown = mIntake.FloorStop.isPressed();
      oIntake.LimitUp = mIntake.SpeakerLimit.isPressed();

      oShooter.ShooterSpeed = mShooter.getShooterSpeed();

      oIntake.kIntakePos_Entry.setDouble(oIntake.IntakePos);
      oIntake.kIntakeRun_Entry.setBoolean(oIntake.IntakeRun);
      oIntake.kIntakeSpeed_Entry.setDouble(oIntake.IntakeSpeed);
      oIntake.kLimitDown.setBoolean(oIntake.LimitDown);
      oIntake.kLimitUp.setBoolean(oIntake.LimitUp);

      oShooter.kShooterSpeed_Entry.setDouble(oShooter.ShooterSpeed);
    }
  }

  @Override
  public void disabledInit() {
    mLighting.ChangeColor(LEDcolors.kNoAllience);
  }

  @Override
  public void disabledPeriodic() {
    if (isRed()) {
      mLighting.ChangeColor(LEDcolors.kRedAllience);
    } else {
      mLighting.ChangeColor(LEDcolors.kBlueAllience);
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    mClimb.LeftDown(kClimber.ClimbSpeed);
    mClimb.RightDown(kClimber.ClimbSpeed);
    m_autonomousCommand = autoChooser.getSelected();

    if (!(m_autonomousCommand == null)) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    mShooter.ShooterIdle();
    if (!(m_autonomousCommand == null)) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // one button intake
    if (secondary.getPOV() == 180) {
      mIntake.FloorPos();
    }
    // Run Amp Config
    if (secondary.getPOV() == 0) {
      mIntake.ShooterPos();
    }
    // Rollers up
    if (secondary.getPOV() == 90) {
      mRoller.RollerDown();
    }
    // Rollers Down
    if (secondary.getPOV() == 270) {
      mRoller.RollerUp();
    }
    // Boost
    if (driver.getRightStickButton()){
      boost = true;
    }
    boost = false;
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {
    // Driver Button Bindings
    driver_a_Button.toggleOnTrue(
      new InstantCommand(() -> mSwerve.ZeroGyro(), mSwerve)
    );
    driver_x_Button.whileTrue(
      new InstantCommand(() -> mSwerve.Xconfig(), mSwerve)
    );

    // Auto Zero
    // driver_RJSD.whileTrue(new AutoNoteTrack(mSwerve));

    // Secondary Button Bindings

      // Scoring

        // Shooter
    secondary_b_Button.whileTrue(new RunShooter(mShooter)); // save for later ig
    // secondary_LJSD.whileTrue(new RunAmp(mShooter));

        // Intake
    secondary_x_Button.whileTrue(new Vomit(mIntake));
    secondary_RJSD.whileTrue(new IntakeGrab(mIntake));
    secondary_LJSD.whileTrue(new IntakeFeed(mIntake));
    secondary_a_Button.whileTrue(new AutoIntake(mIntake, mLighting));
    secondary_y_Button.whileTrue(new RunAmp(mShooter, mIntake, mRoller));

    // Climber
    secondary_left_Bumper.whileTrue(new ArmsUp(mClimb, mIntake, kClimber.ClimbSpeed)).whileFalse(new ArmsDown(mClimb, mIntake, kClimber.ClimbSpeed));
    secondary_Right_Bumper.whileTrue(new IntakeFeed(mIntake));
  }

  public boolean isRed() {
    var Alliance = DriverStation.getAlliance();
    if (Alliance.isPresent()) {
      return Alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}
