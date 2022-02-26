// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.*;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;



/** Add your docs here. */
public class SuperStructure extends Base {

    private static final String kDefaultAuto = "Auto1";
    private static final String kCustomAuto = "Auto2";
    private static final String kCustomAuto1 = "strafeleft";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private SwerveDrive m_SwerveDrive;
    private Intake m_Intake;
    private Shooter m_Shooter;
    private Climber m_Climber;
    private final VisionBall m_VisionBall;
    private final VisionShooter m_VisionShooter = new VisionShooter();

    Timer m_Timer = new Timer();

    private Auto m_auto = new Auto();

    private final Compressor m_phCompressor = new Compressor(PneumaticsModuleType.REVPH);
    private final PneumaticHub m_ph = new PneumaticHub();

    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("super");
    // shuffleboard: camera, is aimed, is ok to shoot, how many balls are in intake

    NetworkTableEntry ntBotRPM = ShootMotorTab.add("Bot RPM", 1900).withPosition(3, 3).withSize(1, 1).getEntry();
    NetworkTableEntry ntTopRPM = ShootMotorTab.add("Top RPM", 1900).withPosition(3, 2).withSize(1, 1).getEntry();
    NetworkTableEntry ntFeetToRPM = ShootMotorTab.add("Feet To Top RPM", 17).withPosition(4, 2).withSize(1, 1)
            .getEntry();
    
            
    

    
    
    

    public SuperStructure(SwerveDrive swerveDrive, Intake intake, Shooter shooter, Climber climber) {

        m_SwerveDrive = swerveDrive;
        m_Intake = intake;
        m_Shooter = shooter;
        m_Climber = climber;
        m_VisionBall = new VisionBall();

    }

    @Override
    public void robotInit() {
        m_phCompressor.disable();
        m_SwerveDrive.robotInit();
        m_Intake.init();
        m_Shooter.robotInit();
        m_Climber.robotInit();
        m_auto.robotInit();
        m_VisionBall.robotInit();
        m_VisionShooter.robotInit();

        Shuffleboard.getTab("super").add("swervedrive", m_SwerveDrive);
        Shuffleboard.getTab("super").add("compressor", m_phCompressor);
        Shuffleboard.getTab("super").addNumber("compressor/pressure", () -> m_phCompressor.getPressure());
        Shuffleboard.getTab("super").addBoolean("IsOkToShoot", () -> m_Shooter.IsOkToShoot());
        // Shuffleboard.getTab("super").addNumber("Closed Loop Error Top", () -> m_Shooter.closedLoopErrorTop());
        // Shuffleboard.getTab("super").addNumber("Closed Loop Error Bot", () -> m_Shooter.closedLoopErrorBot());
        Shuffleboard.getTab("super").addNumber("Camera Based Distance", () -> m_VisionShooter.getDistance());
        Shuffleboard.getTab("super").addNumber("current top RPM", () -> m_Shooter.getTopMotorRPM());
        Shuffleboard.getTab("super").addNumber("Current bot RPM", () -> m_Shooter.getBotMotorRPM());

        Shuffleboard.getTab("super").addBoolean("Is Ball in Loader", () -> m_Intake.isBallAtLoad());
        Shuffleboard.getTab("super").addBoolean("Is Ball in Middle", () -> m_Intake.isBallAtMiddle());
        Shuffleboard.getTab("super").add(CameraServer.putVideo("mmal_service_16.1-output", 2000, 3000));
        Shuffleboard.getTab("super").addNumber("Shooter Yaw", () -> m_VisionShooter.getYaw());

        // auto chooser
        m_chooser.setDefaultOption("Auto1", kDefaultAuto);
        m_chooser.addOption("Auto2", kCustomAuto);
        m_chooser.addOption("strafeleft", kCustomAuto1);
        Shuffleboard.getTab("Auto").add("Auto Choose", m_chooser);
    }

    @Override
    public void robotPeriodic() {
        m_SwerveDrive.changeOdometry(OI.shouldSetFieldRelative(), OI.shouldSetRobotRelative(), OI.getResetOdometry());
        m_Intake.robotPeriodic();
        m_auto.robotPeriodic();
        m_VisionShooter.calculateAngleError();
    }

    @Override
    public void teleopPeriodic() {
        m_phCompressor.enableAnalog(90, 115);

        // testing purposaes, changes intake pnuematics
        if(OI.forwardIntake()) {
            m_Intake.setIntakePnuematic(true);
        } else if(OI.reverseIntake()) {
            m_Intake.setIntakePnuematic(false);
        }

        // DRIVING //
        // VISION GET BALL
        if (OI.getVisionBallEngaged()) {
            ChassisSpeeds speeds = m_VisionBall.followBall();
            m_SwerveDrive.driveFromChassisSpeeds(speeds);
        } else {
            // XBOX DRIVING CODE
            m_SwerveDrive.driveWithXbox(OI.getDriveY(), OI.getDriveX(), OI.getDriveRot(),
                    OI.getCenterOfRotationFrontLeft(),
                    OI.getCenterOfRotationFrontRight());
        }

        // INTAKE STATE UPDATE
        m_Intake.changeState(OI.startIntake(), OI.stopIntake());

        // INTAKE / SHOOTING
        if (OI.getYButtonForShootRPM()) {
            // turn shooter on in rpm mode
            m_Shooter.shootingRPM(ntTopRPM.getNumber(0).doubleValue(), ntBotRPM.getNumber(0).doubleValue());

            if (m_Shooter.IsOkToShoot()) {
                // Load shooter
                m_Intake.loadShooter();

            } else {
                m_Intake.countinueIntakeMotors();
            }
        } else if (OI.getXButtonForShootDist()) {
            double distFeet = m_VisionShooter.getDistance();

            // turn shooter on in Dist
            m_Shooter.shootingDist(distFeet);

            if (m_Shooter.IsOkToShoot()) {
                // load shooter
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }
        } else {
            m_Shooter.turnOff();
        }

        // when shooting released, stop loading
        if (OI.getXorYforShootingReleased()) {
            m_Intake.turnOffLoadShooter();
        }
        // climberControl(OI.getSolenoidReverse(), OI.getSolenoidForward(),
        // OI.getArmsUp(), OI.getArmsDown());
        if (OI.getZeroOfTurnTableTalon()) {
            m_Shooter.setTurnTableToZero();
        }

        // TURNTABLE
        // if not zeroed, zero the turntable
        if (!m_Shooter.m_hasBeenCalibrated) {
            m_Shooter.setTurnTableToZero();

        }
        // // manual control of turntable
        else if (OI.getAimTurret()) {
            // m_Shooter.aimTurret(m_VisionShooter.getYaw());
            m_Shooter.aimTurretTalonOnboard(m_VisionShooter.getYaw());
        } else {
            m_Shooter.TurnTable(OI.getRightBumperForTurntable(), OI.getLeftBumperForTurntable());
        }
        // }
        if(OI.fortFiveTurnTable()){// A
            m_Shooter.setTurnTableAngleFortFive();
        }
        if(OI.hundredTurnTable()){// X
            m_Shooter.setTurnTableAngleHundred();
        }
        if(OI.negFortFiveTurnTable()){// Y
            m_Shooter.setTurnTableAngleNegFortFive();
        }
        if(OI.negHundredTurnTable()){// B
            m_Shooter.setTurnTableAngleNegHundred();
        }
        


        // CLIMBER
        climberControl(OI.getSolenoidReverse(), OI.getSolenoidForward(),
                OI.getArmsUp(), OI.getArmsDown());
    }

    @Override
    public void autonomousInit() {
        m_auto.autonomousInit();
        m_Intake.autonomousInit();

        PathPlannerState state = new PathPlannerState();
        state.poseMeters = new Pose2d();
        state.holonomicRotation = new Rotation2d();

        if (m_chooser.getSelected() == "Auto1") {
            state = m_auto.getInitialState_auto1();
        } else if (m_chooser.getSelected() == "Auto2") {
            state = m_auto.getInitialState_auto2();
        } else if (m_chooser.getSelected() == "strafeleft") {
            state = m_auto.getInitialState_auto3();
        }

        m_SwerveDrive.resetOdometry(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));

        m_Timer.reset();
    }

    @Override
    public void autonomousPeriodic() {
        if (m_chooser.getSelected() == "Auto1") {
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds_Auto1(m_SwerveDrive.getPose2d()));

            // shoot during auto
            if (m_Timer.get() > 1.25 && m_Timer.get() < 5) {
                m_Shooter.shootingRPM(ntTopRPM.getNumber(0).doubleValue(), ntBotRPM.getNumber(0).doubleValue());
                if (m_Shooter.IsOkToShoot()) {
                    m_Intake.loadShooter();
                }
            } else {
                m_Shooter.turnOff();
                m_Intake.turnOffLoadShooter();
            }

            m_Intake.changeState(false, false);

        } else if (m_chooser.getSelected() == "Auto2") {
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds_Auto2(m_SwerveDrive.getPose2d()));


            m_Intake.changeState(false, false);

            if(!m_Intake.getHasDetectedMiddle() || !m_auto.isTrajectoryCompleted()) {
                return;
            }

            m_Shooter.shootingRPM(ntTopRPM.getNumber(0).doubleValue(), ntBotRPM.getNumber(0).doubleValue());

            if (!m_Shooter.IsOkToShoot()) {
                return;
            }

            m_Intake.loadShooter();
            m_Timer.start();

            if(m_Timer.get() < 3) {
                return;
            }

            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();

        } else if (m_chooser.getSelected() == "strafeleft") {
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds_Auto3(m_SwerveDrive.getPose2d()));
        }
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

        if (OI.getDriveForward()) {
            m_SwerveDrive.drive(0.01, 0.00, 0, false);
        } else if (OI.getDriveLeft()) {
            m_SwerveDrive.drive(0.00, 0.01, 0, false);
        } else if (OI.getDriveReverse()) {
            m_SwerveDrive.drive(-0.01, 0.00, 0, false);
        } else if (OI.getDriveRight()) {
            m_SwerveDrive.drive(0.00, -0.01, 0, false);
        } else {
            m_SwerveDrive.drive(0, 0, 0, false);
        }
        

    }

    private void climberControl(boolean shortHookBack, boolean shortHookForward, boolean armsUp, boolean armsDown) {

        // solenoids
        if (shortHookBack) {
            m_Climber.hooksReverse();
        } else if (shortHookForward) {
            m_Climber.hooksForward();
        }
        // motors
        if (armsUp) {
            m_Climber.armsUp();
        } else if (armsDown) {
            m_Climber.armsDown();

        } else {
            m_Climber.armsStop();
        }

    }
}
