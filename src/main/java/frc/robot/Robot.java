// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.*;


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
  //for superstructure
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();

  SuperStructure superStructure = new SuperStructure(swerveDrive, intake, shooter, climber); 
  
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();

    Shuffleboard.getTab("computil").add("xboxDrive", OI.xboxDrive);
    Shuffleboard.getTab("computil").add("xboxClimb", OI.xboxClimb);

    Shuffleboard.getTab("computil").addNumber("pose/x", ()->swerveDrive.getPose2d().getX());
    Shuffleboard.getTab("computil").addNumber("pose/y", ()->swerveDrive.getPose2d().getY());
    Shuffleboard.getTab("computil").addNumber("pose/theta", ()->swerveDrive.getPose2d().getRotation().getDegrees());

    // CameraServer.startAutomaticCapture();

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
    //SUPER STRUCTURE STUFF
    superStructure.autonomousInit();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
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
