// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // TODO: double check what type of compressor module we are using
  // private final Compressor m_phCompressor = new Compressor(2, PneumaticsModuleType.REVPH);

  //for superstructure
  SwerveDrive swerveDrive = new SwerveDrive();
  Intake intake = new Intake();
  Shooter shooter = new Shooter();
  ColorSensor colorSensor = new ColorSensor();
  Climber climber = new Climber();

  SuperStructure superStructure = new SuperStructure(swerveDrive, intake, shooter, colorSensor, climber);

  private Auto autonomous = new Auto();
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Shuffleboard.getTab("main").add("xbox", OI.xboxDrive);

    Shuffleboard.getTab("main").addNumber("pose/x", ()->swerveDrive.getPose2d().getX());
    Shuffleboard.getTab("main").addNumber("pose/y", ()->swerveDrive.getPose2d().getY());
    Shuffleboard.getTab("main").addNumber("pose/theta", ()->swerveDrive.getPose2d().getRotation().getDegrees());

    autonomous.robotInit();

    CameraServer.startAutomaticCapture();

    //SUPER STRUCTURE STUFF
    superStructure.robotInit();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    autonomous.robotPeriodic();

    //SUPER STRUCTURE STUFF
    superStructure.robotPeriodic();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    autonomous.autonomousInit();

    //SUPER STRUCTURE STUFF
    superStructure.autonomousInit();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    autonomous.autonomousPeriodic();

    //SUPER STRUCTURE STUFF
    superStructure.autonomousPeriodic();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    //SUPER STRUCTURE STUFF
    superStructure.teleopInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //SUPER STRUCTURE STUFF
    superStructure.teleopPeriodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    //SUPER STRUCTURE STUFF
    superStructure.disabledInit();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    //SUPER STRUCTURE STUFF
    superStructure.disabledPeriodic();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

    //SUPER STRUCTURE STUFF
    superStructure.testInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    //SUPER STRUCTURE STUFF
    superStructure.testPeriodic();
  }
}
