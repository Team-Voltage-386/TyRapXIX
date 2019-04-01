package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drive.LevelOneAuto;
import frc.robot.commands.drive.LevelTwoAuto;
import frc.robot.commands.drive.LoganContributions;
import frc.robot.commands.drive.LoganContributionsCargo;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndgameClimbSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static ArmSubsystem armSubsystem = new ArmSubsystem();
  public static CameraSubsystem cameraSubsystem = new CameraSubsystem();
  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static EndgameClimbSubsystem endgameClimbSubsystem = new EndgameClimbSubsystem();
  public static ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
  public static OI oi;

  Command autonomousCommand;
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  NetworkTableEntry ballError;
  NetworkTableEntry numberOfRects;
  NetworkTableEntry pairCenterPi;
  NetworkTableEntry screenCenterPi;

  double rectsNum;
  public static double pairCenter;
  public static double screenCenter;
  public static double error;
  double defaultValue;
  boolean clearForVision;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    defaultValue = -1.0;
    oi = new OI();
    // chooser.addOption("My Auto", new MyAutoCommand());

    // SmartDashboard.putData("Auto mode", m_chooser);
    // SmartDashboard.putNumber("elbowPK ", 1.2);
    // SmartDashboard.putNumber("elbowIK ", 0.02);
    // SmartDashboard.putNumber("elbowDK ", 0);
    // SmartDashboard.putNumber("shoulderPK ", -25);
    // SmartDashboard.putNumber("shoulderDK ", 0);
    // SmartDashboard.putNumber("shoulderIK ", 0);
    // SmartDashboard.putNumber("elbowResetPK ", 1.4);
    // SmartDashboard.putNumber("Downwards Elbow Limiter", 0.8);
    SmartDashboard.putNumber("kp", 0.0075);
    SmartDashboard.putNumber("ki", 0.0);
    SmartDashboard.putNumber("kd", 0.5);
    // SmartDashboard.putNumber("elbowPK ", ArmSubsystem.ELBOW_PK);
    // SmartDashboard.putNumber("elbowIK ", ArmSubsystem.ELBOW_IK);
    // SmartDashboard.putNumber("elbowDK ", ArmSubsystem.ELBOW_DK);
    // SmartDashboard.putNumber("ElbowDownLimiter ",
    // ArmSubsystem.DOWNWARDS_ELBOW_LIMITER);
    // SmartDashboard.putNumber("MaxErrorForIUse ",
    // ArmSubsystem.MAX_ERROR_FOR_I_USE);

    // autoChooser.setName("Auto Chooser");
    // autoChooser.setDefaultOption("Manual Hatch", new LoganContributions());
    // autoChooser.addOption("Level 2", new LevelTwoAuto());
    // autoChooser.addOption("Level 1", new LevelOneAuto());
    // autoChooser.addOption("Manual Cargo", new LoganContributionsCargo());

    // SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Level 2 auto", new LevelTwoAuto());
    SmartDashboard.putData("Level 1 auto", new LevelOneAuto());

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Front Ultrasonic", driveSubsystem.getUltrasonicDistance());
    // SmartDashboard.putString("Current Climb Command",
    // endgameClimbSubsystem.getCurrentCommandName());
    // SmartDashboard.putString("ArmCurrentCommand",
    // armSubsystem.getCurrentCommandName());

    driveSubsystem.displayDiagnostics();
    armSubsystem.displayDiagnostics();
    endgameClimbSubsystem.displayDiagnostics();
    // manipulatorSubsystem.displayDiagnostics();

    NetworkTableInstance testInstance = NetworkTableInstance.getDefault();
    testInstance.startServer();
    testInstance.setServerTeam(386, 1735);
    NetworkTable table = testInstance.getTable("datatable");

    ballError = table.getEntry("W");
    numberOfRects = table.getEntry("X");
    pairCenterPi = table.getEntry("Y");
    screenCenterPi = table.getEntry("Z");

    error = ballError.getDouble(defaultValue);
    rectsNum = numberOfRects.getDouble(defaultValue);
    pairCenter = pairCenterPi.getDouble(defaultValue);
    screenCenter = 93;// 97 ??? screenCenterPi.getDouble(defaultValue);

    if (rectsNum > 0) {
      clearForVision = true;
    } else {
      clearForVision = false;
    }

    SmartDashboard.putNumber("Rects", rectsNum);
    SmartDashboard.putNumber("Average Target Center", pairCenter);
    SmartDashboard.putBoolean("Target Vision Ready", clearForVision);
    SmartDashboard.putNumber("Defined Screen Center", screenCenter);
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This method is executed once when autonomous mode is started.
   */
  @Override
  public void autonomousInit() {
    Robot.driveSubsystem.resetEncoders();
    // autonomousCommand = autoChooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This makes sure that the autonomous stops running when teleop starts running.
   * If you want the autonomous to continue until interrupted by another command,
   * remove this line or comment it out.
   */
  @Override
  public void teleopInit() {
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.cancel();
    // }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
