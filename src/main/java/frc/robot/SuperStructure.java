// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.*;
import edu.wpi.first.networktables.NetworkTableEntry;

/** Add your docs here. */
public class SuperStructure extends Base {

    private SwerveDrive m_SwerveDrive;
    private Intake m_Intake;
    private Shooter m_Shooter;
    private ColorSensor m_ColorSensor;
    // private Climber m_Climber;

    private final Compressor m_phCompressor = new Compressor(PneumaticsModuleType.REVPH);
    private final PneumaticHub m_ph = new PneumaticHub();

    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("ShooterSuper");
    
    NetworkTableEntry ntBotRPM = ShootMotorTab.add("Bot RPM", 1000).withPosition(3, 3).withSize(1, 1).getEntry();
    NetworkTableEntry ntTopRPM = ShootMotorTab.add("Top RPM", 1000).withPosition(3, 2).withSize(1, 1).getEntry();
    NetworkTableEntry ntFeetToRPM = ShootMotorTab.add("Feet To Top RPM", 17).withPosition(4, 2).withSize(1, 1)
            .getEntry();

    public SuperStructure(SwerveDrive swerveDrive, Intake intake, Shooter shooter) {

        m_SwerveDrive = swerveDrive;
        m_Intake = intake;
        m_Shooter = shooter;
        // m_Climber = climber;

    }

    @Override
    public void robotInit() {
        m_phCompressor.disable();
        m_SwerveDrive.robotInit();
        m_Intake.init();
        m_Shooter.robotInit();
        // m_Climber.robotInit();

        Shuffleboard.getTab("super").add("compressor", m_phCompressor);
        Shuffleboard.getTab("super").addNumber("compressor/pressure", () -> m_phCompressor.getPressure());
    }

    @Override
    public void robotPeriodic() {

        m_SwerveDrive.changeOdometry(OI.shouldSetFieldRelative(), OI.shouldSetRobotRelative(), OI.getResetOdometry());
        

    }

    @Override
    public void teleopPeriodic() {
        m_phCompressor.enableAnalog(90, 115);

        m_SwerveDrive.driveWithXbox(OI.getDriveY(), OI.getDriveX(), OI.xboxDrive.getLeftTriggerAxis(),
                OI.xboxDrive.getRightTriggerAxis(), OI.xboxDrive.getRightY(), OI.xboxDrive.getRightX());

        // intake control
        m_Intake.changeState(OI.startIntake(), OI.stopIntake());
        m_Shooter.TurnTable(OI.getRightBumperForTurntable(), OI.getLeftBumperForTurntable());

        //

        if (OI.getYButtonForShootRPM()) {
            // turn shooter on in rpm mode
            m_Shooter.shootingRPM(ntTopRPM.getNumber(0).doubleValue(), ntBotRPM.getNumber(0).doubleValue());

            if (m_Shooter.IsOkToShoot()) {
                // Load shooter
                m_Intake.loadShooter();

            }
        } else if (OI.getXButtonForShootDist()) {
            // turn shooter on in Dist
            m_Shooter.shootingDist(ntFeetToRPM.getNumber(0).doubleValue());

            if (m_Shooter.IsOkToShoot()) {
                // load shooter
                m_Intake.loadShooter();
            }
        } else {
            m_Shooter.turnOff();
        }

        if (OI.getXorYforShootingReleased()) {
            m_Intake.turnOffLoadShooter();
        }
        // climberControl(OI.getSolenoidReverse(), OI.getSolenoidForward(), OI.getArmsUp(), OI.getArmsDown());



    }

    @Override
    public void autonomousInit() {

        m_SwerveDrive.resetOdometry();

    }

    @Override
    public void disabledInit() {
        m_phCompressor.disable();
    }

    @Override
    public void testInit() {
        m_phCompressor.enableAnalog(80, 115);
    }

    @Override
    public void testPeriodic() {
        climberControl(OI.getSolenoidReverse(), OI.getSolenoidForward(), OI.getArmsUp(), OI.getArmsDown());


    }

    private void climberControl(boolean shortHookBack, boolean shortHookForward, boolean armsUp, boolean armsDown) {

        // solenoids
        // if (shortHookBack) {
        //     m_Climber.hooksReverse();
        // } else if (shortHookForward) {
        //     m_Climber.hooksForward();
        // }
        // // motors
        // if (armsUp) {
        //     m_Climber.armsUp();
        // } else if (armsDown) {
        //     m_Climber.armsDown();
    
        // } else {
        //     m_Climber.armsStop();
        // }

    }
}
