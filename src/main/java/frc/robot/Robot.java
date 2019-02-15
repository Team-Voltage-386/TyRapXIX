/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.testCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BeakSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CargoManipSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndgameClimbSubsystem;
import frc.robot.subsystems.SpikeSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static ArmSubsystem armSubsystem = new ArmSubsystem();
  public static BeakSubsystem beakSubsystem = new BeakSubsystem();
  public static CameraSubsystem cameraSubsystem = new CameraSubsystem();
  public static CargoManipSubsystem cargoManipSubsystem = new CargoManipSubsystem();
  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static EndgameClimbSubsystem endgameClimbSubsystem = new EndgameClimbSubsystem();
  public static SpikeSubsystem spikeSubsystem = new SpikeSubsystem();
  public static OI m_oi;

  NetworkTableEntry numberOfPairs;
  // NetworkTableEntry imgWidth;
  // NetworkTableEntry imgHeight;
  double pairsNum;
  double defaultValue;
  // double defaultWidth;
  // double defaultHeight;
  // double actualWidth;
  // double actualHeight;

  Command m_autonomousCommand;
  Command testerCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // encoder and sensor labels
  public static final String ENCODER_TALON_1 = "Encoder Talon 1";
  public static final String ENCODER_TALON_3 = "Encoder Talon 3";

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    defaultValue = 2.0;
    //spikeSubsystem.spikeToggle();
    // defaultValue[1] = 3.0;
    // defaultWidth = 5;
    // defaultHeight = 5;
    // NetworkTableInstance testInstance = NetworkTableInstance.getDefault();
    // NetworkTable table = testInstance.getTable("datatable");
    // testEntry = table.getEntry("X");
    // testInstance.startClientTeam(386);
    // testValue = testEntry.getDouble(2.0);
  
   

    m_oi = new OI();
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    Robot.armSubsystem.resetEncoder();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    testerCommand = new testCommand();

    NetworkTableInstance testInstance = NetworkTableInstance.getDefault();
    testInstance.startServer();
    testInstance.setServerTeam(386, 1735);
    NetworkTable table = testInstance.getTable("datatable");
    numberOfPairs = table.getEntry("X");
    // imgWidth = table.getEntry("Y");
    // imgHeight = table.getEntry("Z");
    pairsNum = numberOfPairs.getDouble(defaultValue);
    // actualHeight = imgHeight.getDouble(defaultHeight);
    // defaultValue = new double[3];
    // defaultValue[0] = 1;
    // valueOfEntry = testEntry.getDoubleArray(defaultValue);
 
    SmartDashboard.putNumber("Yaw Degree", Robot.driveSubsystem.getPigeonYPR()[0]);
    SmartDashboard.putNumber("Pitch Degree", Robot.driveSubsystem.getPigeonYPR()[1]);
    SmartDashboard.putNumber("Roll Degree", Robot.driveSubsystem.getPigeonYPR()[2]); 
    SmartDashboard.putNumber("Number of Pairs",pairsNum);
    // SmartDashboard.putNumber("R",valueOfEntry[0]);
    // SmartDashboard.putNumber("G",valueOfEntry[1]);
    // SmartDashboard.putNumber("B",valueOfEntry[2]);
    // SmartDashboard.putNumber("Width",actualWidth);
    // SmartDashboard.putNumber("Height",actualHeight);
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
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = new NetworkTableTest();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    driveSubsystem.displayDiagnostics();


  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
