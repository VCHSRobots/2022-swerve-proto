// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class SuperStructure extends Base {

    private SwerveDrive m_SwerveDrive;
    private OI m_OI;
    private Intake m_Intake;
    private Shooter m_Shooter;
    private ColorSensor m_ColorSensor;
    private Climber m_Climber;

    public SuperStructure (SwerveDrive swerveDrive, OI oi, Intake intake, Shooter shooter, ColorSensor colorSensor, Climber climber) {
        
        m_SwerveDrive = swerveDrive;
        m_OI = oi;
        m_Intake = intake;
        m_Shooter = shooter;
        m_ColorSensor = colorSensor;
        m_Climber = climber;

    }

    @Override
    public void robotInit() {

        m_SwerveDrive.init();
        m_Intake.init();
        m_Shooter.init();
        m_ColorSensor.init();

        m_Climber.init();

    }

    @Override
    public void robotPeriodic() {

        m_SwerveDrive.changeOdometry(m_OI.shouldSetFieldRelative(), m_OI.shouldSetRobotRelative(), m_OI.getResetOdometry());
        m_ColorSensor.checkColor();

    }

    @Override
    public void teleopPeriodic() {

        m_SwerveDrive.driveWithXbox(m_OI.getDriveY(), m_OI.getDriveX(), OI.xboxDrive.getLeftTriggerAxis(), OI.xboxDrive.getRightTriggerAxis(), OI.xboxDrive.getRightY(), OI.xboxDrive.getRightX());
        m_Intake.changeState(m_OI.startIntake());
        m_Shooter.shootAndTurn(m_OI.getRightTriggerAxisForShoot(), m_OI.getRightBumperForTurntable(), m_OI.getLeftBumperForTurntable());
        m_Climber.tPeriodic(m_OI.getSolenoidToggle(), m_OI.getArmsUp(), m_OI.getArmsDown());

    }

    @Override
    public void testPeriodic() {

        m_SwerveDrive.test(OI.xboxDrive.getAButton());

    }


    

}
