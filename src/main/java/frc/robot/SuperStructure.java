// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;

/** Add your docs here. */
public class SuperStructure extends Base {

    private SwerveDrive m_SwerveDrive;
    private Intake m_Intake;
    private Shooter m_Shooter;
    private ColorSensor m_ColorSensor;
    private Climber m_Climber;
    ShuffleboardTab ShootMotor1Tab = Shuffleboard.getTab("Shooter1");

    NetworkTableEntry ntBotRPM = ShootMotor1Tab.add("Bot RPM", 1000).withPosition(3, 3).withSize(1, 1).getEntry();
    NetworkTableEntry ntTopRPM = ShootMotor1Tab.add("Top RPM", 1000).withPosition(3, 2).withSize(1, 1).getEntry();
    NetworkTableEntry ntFeetToRPM = ShootMotor1Tab.add("Feet To Top RPM", 17).withPosition(4, 2).withSize(1, 1)
            .getEntry();

    public SuperStructure(SwerveDrive swerveDrive, Intake intake, Shooter shooter, ColorSensor colorSensor,
            Climber climber) {

        m_SwerveDrive = swerveDrive;
        m_Intake = intake;
        m_Shooter = shooter;
        m_ColorSensor = colorSensor;
        m_Climber = climber;

    }

    @Override
    public void robotInit() {
        m_SwerveDrive.robotInit();
        m_Intake.init();
        m_Shooter.robotInit();
        m_ColorSensor.init();

        m_Climber.init();

    }

    @Override
    public void robotPeriodic() {

        m_SwerveDrive.changeOdometry(OI.shouldSetFieldRelative(), OI.shouldSetRobotRelative(), OI.getResetOdometry());
        m_ColorSensor.checkColor();

    }

    @Override
    public void teleopPeriodic() {

        m_SwerveDrive.driveWithXbox(OI.getDriveY(), OI.getDriveX(), OI.xboxDrive.getLeftTriggerAxis(),
                OI.xboxDrive.getRightTriggerAxis(), OI.xboxDrive.getRightY(), OI.xboxDrive.getRightX());
        m_Intake.changeState(OI.startIntake());

        m_Shooter.TurnTable(OI.getRightBumperForTurntable(), OI.getLeftBumperForTurntable());
        m_Climber.climberMove(OI.getSolenoidToggle(), OI.getArmsUp(), OI.getArmsDown());

        
        if (OI.getYButtonForShootRPM()) {
            // turn shooter on in rpm mode
            m_Shooter.shootingRPM(ntTopRPM.getNumber(0).doubleValue(), ntBotRPM.getNumber(0).doubleValue());
            

            if (m_Shooter.IsOkToShoot()) {
                // Load shooter
                m_Intake.loadShooter();
            }
        } else if (OI.getXButtonForToggleFeetToDist()) {
            // turn shooter on in Dist
            m_Shooter.shootingDist(ntFeetToRPM.getNumber(0).doubleValue());

            if (m_Shooter.IsOkToShoot()) {
                // load shooter
            }

        } else {

        }

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
