// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class SuperStructure extends Base {

    private SwerveDrive m_SwerveDrive;
    private Intake m_Intake;
    private Shooter m_Shooter;
    private ColorSensor m_ColorSensor;
    private Climber m_Climber;
    



    public SuperStructure (SwerveDrive swerveDrive, Intake intake, Shooter shooter, ColorSensor colorSensor, Climber climber) {
        
        m_SwerveDrive = swerveDrive;
        m_Intake = intake;
        m_Shooter = shooter;
        m_ColorSensor = colorSensor;
        m_Climber = climber;

    }

    @Override
    public void robotInit() {

        m_SwerveDrive.robotInit();
        m_Intake.robotInit();
        m_Shooter.robotInit();
        m_ColorSensor.robotInit();
        m_SwerveDrive.robotInit();
       

    }

    @Override
    public void robotPeriodic() {

        m_SwerveDrive.changeOdometry(OI.shouldSetFieldRelative(), OI.shouldSetRobotRelative(), OI.getResetOdometry());
        m_ColorSensor.checkColor();

    }

    @Override
    public void teleopPeriodic() {

        m_SwerveDrive.driveWithXbox(OI.getDriveY(), OI.getDriveX(), OI.xboxDrive.getLeftTriggerAxis(), OI.xboxDrive.getRightTriggerAxis(), OI.xboxDrive.getRightY(), OI.xboxDrive.getRightX());
        m_Intake.changeState(OI.startIntake());
       // m_Shooter.shootAndTurn(OI.getRightTriggerAxisForShoot(), OI.getRightBumperForTurntable(), OI.getLeftBumperForTurntable());
      m_Climber.climberMove(OI.getSolenoidToggle(), OI.getArmsUp(), OI.getArmsDown(), false);

    }

    @Override
    public void autonomousInit() {

        m_SwerveDrive.resetOdometry();

    }

    @Override
    public void testPeriodic() {

        m_SwerveDrive.test(OI.xboxDrive.getAButton());

    }


    

}
