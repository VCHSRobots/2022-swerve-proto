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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.state.RobotState;
import frc.robot.subsystems.*;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;

/** Add your docs here. */
public class SuperStructure extends Base {

    private static final String kDefaultAuto = "Auto1";
    private static final String kCustomAuto = "Auto2";
    private static final String kCustomAuto1 = "Auto3";
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private SwerveDrive m_SwerveDrive;
    private Intake m_Intake;
    private Shooter m_Shooter;
    private Climber m_Climber;
    private final VisionBall m_VisionBall;
    private final VisionShooter m_VisionShooter = new VisionShooter();

    private int m_zeroBallCounter = 0;
    private double m_timestamp = 0;
    private RobotState m_state;
    private Thread m_cameraThread;
    boolean m_barfTimerStarted = false;

    Timer m_Timer = new Timer();
    int m_autoStep = 0;

    private Auto m_auto = new Auto();

    private final Compressor m_phCompressor = new Compressor(PneumaticsModuleType.REVPH);
    private final PneumaticHub m_ph = new PneumaticHub();

    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("super");
    // shuffleboard: camera, is aimed, is ok to shoot, how many balls are in intake

    NetworkTableEntry ntBotRPM = ShootMotorTab.add("Bot RPM", 1900).withPosition(3, 5).withSize(1, 1).getEntry();
    NetworkTableEntry ntTopRPM = ShootMotorTab.add("Top RPM", 1900).withPosition(3, 4).withSize(1, 1).getEntry();
    NetworkTableEntry ntFeetToRPM = ShootMotorTab.add("Feet To Top RPM", 17).withPosition(0, 0).withSize(1, 1)
            .getEntry();

    public SuperStructure(SwerveDrive swerveDrive, Intake intake, Shooter shooter, Climber climber) {

        m_SwerveDrive = swerveDrive;
        m_Intake = intake;
        m_Shooter = shooter;
        m_Climber = climber;
        m_VisionBall = new VisionBall();

    }

    private int pastAngle;

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

        m_cameraThread = new Thread(
                () -> {
                    UsbCamera camera = CameraServer.startAutomaticCapture();
                });
        m_cameraThread.setDaemon(true);
        m_cameraThread.start();

        Shuffleboard.getTab("super").add("swervedrive", m_SwerveDrive).withPosition(8, 1).withSize(2, 2);
        Shuffleboard.getTab("super").add("compressor", m_phCompressor).withPosition(8, 3).withSize(2, 2);
        Shuffleboard.getTab("super").addNumber("compressor/pressure", () -> m_phCompressor.getPressure());

        Shuffleboard.getTab("super").addBoolean("IsOkToShoot", () -> m_Shooter.IsOkToShoot()).withPosition(4, 1);
        // Shuffleboard.getTab("super").addNumber("Closed Loop Error Top", () ->
        // m_Shooter.closedLoopErrorTop());
        // Shuffleboard.getTab("super").addNumber("Closed Loop Error Bot", () ->
        // m_Shooter.closedLoopErrorBot());
        Shuffleboard.getTab("super").addNumber("Camera Based Distance", () -> m_VisionShooter.getDistance())
                .withPosition(6, 2).withSize(2, 1);
        Shuffleboard.getTab("shooter debug").addNumber("Camera to Target Yaw", () -> m_VisionShooter.getYaw());
        Shuffleboard.getTab("shooter debug").addNumber("Turret", () -> m_Shooter.getTurretAngleDegrees());
        Shuffleboard.getTab("shooter debug").addNumber("Turret CANcoder", () -> m_Shooter.getTurretAngleCANcoder());
        Shuffleboard.getTab("super").addNumber("current top RPM", () -> m_Shooter.getTopMotorRPM());
        Shuffleboard.getTab("super").addNumber("Current bot RPM", () -> m_Shooter.getBotMotorRPM());
        Shuffleboard.getTab("super").addBoolean("Is Ball in Loader", () -> m_Intake.isBallAtLoad());
        Shuffleboard.getTab("super").addBoolean("Is Ball in Middle", () -> m_Intake.isBallAtMiddle());
        Shuffleboard.getTab("super").addBoolean("Both balls loaded",
                () -> m_Intake.isBallAtMiddle() && m_Intake.isBallAtLoad());
        Shuffleboard.getTab("super").add(CameraServer.putVideo("mmal_service_16.1-output", 2000, 3000));
        Shuffleboard.getTab("super").addNumber("pressure", () -> m_phCompressor.getPressure());

        // auto chooser
        m_chooser.setDefaultOption(kDefaultAuto, kDefaultAuto);
        m_chooser.addOption(kCustomAuto, kCustomAuto);
        m_chooser.addOption(kCustomAuto1, kCustomAuto1);
        Shuffleboard.getTab("Auto").add("Auto Choose", m_chooser);

        m_state = new RobotState(m_SwerveDrive.getPose2d(), Rotation2d.fromDegrees(m_Shooter.getTurretAngleDegrees()));
    }

    @Override
    public void robotPeriodic() {
        m_SwerveDrive.changeOdometry(OI.shouldSetFieldRelative(), OI.shouldSetRobotRelative(), OI.getResetOdometry());
        m_Intake.robotPeriodic();
        m_auto.robotPeriodic();
        m_VisionShooter.calculateAngleError();

        m_Climber.checkZero();

        m_state.update(m_SwerveDrive.getPose2d(), Rotation2d.fromDegrees(m_Shooter.getTurretAngleDegrees()));
    }

    @Override
    public void teleopInit() {
        m_Shooter.turnOff();
        m_Intake.turnOffLoadShooter();
        m_Climber.hooksReverse();
    }

    @Override
    public void teleopPeriodic() {
        m_phCompressor.enableAnalog(90, 110);

        // testing purposaes, changes intake pnuematics
        if (OI.forwardIntake()) {
            m_Intake.setIntakePnuematic(true);
        } else if (OI.reverseIntake()) {
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
                    false, false);
        }

        // INTAKE STATE UPDATE
        m_Intake.changeState(OI.startIntake(), OI.stopIntake());

        // INTAKE / SHOOTING

        if (OI.getUnjam()) {
            // m_Shooter.shootingRPM(2700, 2700);
            // if (m_Intake.getNumberOfBallsHolding() == 0) {
            // m_barfTimerStarted = false;
            // m_Timer.reset();
            // m_Timer.stop();
            // m_Intake.turnOffLoadShooter();
            // }
            // if (m_barfTimerStarted && m_Timer.advanceIfElapsed(0.5)) {
            // m_Intake.loadShooter();
            // } else if (!m_barfTimerStarted) {
            // m_Timer.reset();
            // m_Timer.start();
            // m_barfTimerStarted = true;
            // }
            m_Intake.unjamShooter();

        } else if (OI.getBarf()) {
            m_Shooter.setBarfVoltage();
            if (m_Shooter.isSpinningFastEnoughForBarf()) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (OI.getRightTriggerForShooting()) {
            // camera center shot
            m_Shooter.shootingRPM(3000, 2400);
            if (m_Shooter.IsOkToShoot()) {
                // Load shooter
                m_Intake.loadShooter();

            } else {
                m_Intake.countinueIntakeMotors();
            }
        } else if (OI.getRightBumperForShootingClose()) {
            // turn shooter on in rpm mode
            m_Shooter.shootingRPM(2500, 2650);
            if (m_Shooter.IsOkToShoot()) {
                // Load shooter
                m_Intake.loadShooter();

            } else {
                m_Intake.countinueIntakeMotors();
            }
        } else if (OI.getLeftBumperForShootingFar()) {
            // 4000, 2900 for vision target on bot of screen
            m_Shooter.shootingRPM(5100, 3000);
            if (m_Shooter.IsOkToShoot()) {
                // load shooter
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (OI.xboxDrive.getYButton()) {
            m_Shooter.shootingRPM(ntTopRPM.getDouble(0), ntBotRPM.getDouble(0));
            if (m_Shooter.IsOkToShoot()) {
                // load shooter
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (OI.getAimTurret()) {
            // m_Shooter.warmUp();
        } else if(m_Intake.getBothBallsLoaded()) {
            // speed up shooter automatically
            m_Shooter.shootingRPM(ntTopRPM.getDouble(0), ntBotRPM.getDouble(0));
        } else {
            m_Shooter.turnOff();
        }

        // when shooting released, stop loading
        if (OI.getShootingReleased()) {
            m_Intake.turnOffLoadShooter();
        }

        // climberControl(OI.getSolenoidReverse(), OI.getSolenoidForward(),
        // OI.getArmsUp(), OI.getArmsDown());
        // if (OI.getZeroOfTurnTableTalon()) {
        // m_Shooter.setTurnTableToZero();
        // }
        if (OI.getArmsDown() && OI.getArmsUp()) {
            m_Climber.setClimberToZero();
        } else {
            climberControl(OI.getSolenoidReverse(), OI.getSolenoidForward(),
                    OI.getArmsUp(), OI.getArmsDown());
        }

        // // TURNTABLE
        if (OI.getAimTurret()) {
            // m_Shooter.aimTurret(m_VisionShooter.getYaw());
            m_Shooter.aimTurret(m_VisionShooter.getYaw());
        } else if (OI.aimWithPose()) {
            m_Shooter.aimTurret(m_state.getTurretAimingAngle().getDegrees());
        } else {
            m_Shooter.TurnTable(OI.getRightTurntable(),
                    OI.getLeftTurntable());
        }

        if (OI.fortFiveTurnTable()) {// A
            m_Shooter.setTurnTableAngleFortFive();
        }
        if (OI.hundredTurnTable()) {// X
            m_Shooter.setTurnTableAngleHundred();
        }
        if (OI.negFortFiveTurnTable()) {// Y
            m_Shooter.setTurnTableAngleNegFortFive();
        }
        if (OI.negHundredTurnTable()) {// B
            m_Shooter.setTurnTableAngleHundred();
        }

        // CLIMBER

        // left & right bumpers hold down
        // if (!m_Climber.isCalibrated()) {
        // m_Climber.setClimberToZero();
        // }
        if (OI.getArmsDown() && OI.getArmsUp()) {
            m_Climber.setClimberToZero();
        } else {
            climberControl(OI.getSolenoidReverse(), OI.getSolenoidForward(),
                    OI.getArmsUp(), OI.getArmsDown());
        }
    }

    // auto coding

    @Override
    public void autonomousInit() {
        m_autoStep = 0;
        m_auto.autonomousInit();
        m_Intake.autonomousInit();
        m_Climber.hooksReverse();

        m_timestamp = Timer.getFPGATimestamp();

        PathPlannerState state = new PathPlannerState();
        state.poseMeters = new Pose2d();
        state.holonomicRotation = new Rotation2d();

        if (m_chooser.getSelected() == "Auto1") {
            m_auto.setupAuto1p1();
        } else if (m_chooser.getSelected() == "Auto2") {
            m_auto.setupAuto2();
        } else if (m_chooser.getSelected() == "Auto3") {
            m_auto.setupAuto3();
        }
        state = m_auto.getInitialState();

        m_SwerveDrive.resetOdometry(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));

        m_Timer.reset();
    }

    @Override
    public void autonomousPeriodic() {
        if (m_chooser.getSelected() == "Auto1") {
            Auto1();

        } else if (m_chooser.getSelected() == "Auto2") {
            Auto2();

        } else if (m_chooser.getSelected() == "Auto3") {
            Auto3();
        }
    }

    public void Auto1() {
        // m_Shooter.setTurretAngle(m_state.getTurretToCenterHub().getRotation().getDegrees());

        if (m_autoStep == 0) {
            double targetAngle = 168;
            m_Shooter.setTurretAngle(targetAngle);
            double angle = m_Shooter.getTurretAngleDegrees();
            if (angle < 0) {
                angle += 360;
            }
            // shoot ball holding
            m_Shooter.shootingRPM(1000, 3250);

            if (m_Shooter.IsOkToShoot() && Math.abs(targetAngle - angle) < 7) {
                m_Intake.loadShooter();
            }
            // when no more balls, go to next step
            if (m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 3) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 1;
                    m_Timer.reset();
                    m_Timer.start();
                }
            }
        } else if (m_autoStep == 1) {
            // wait until all balls thru shooter
            if (m_Timer.advanceIfElapsed(0.1)) {
                m_autoStep = 2;
                m_auto.setupAuto1p1();
            }
            m_Shooter.shootingRPM(1000, 3250);
            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            }

//INCREASE TIME THAT SHOOTER IS RUNNING BEFORE BEGINNING NEXT STEP IN AUTO (+1 to 2 sec)
//SHOOTER NOT SHOOTING ALL BALLS BEFORE MOVING ON
        } else if (m_autoStep == 2) {
            // follow first trajectory to pick up 2 balls
            // move to next step when traj complete
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 3;
                m_Timer.reset();
                m_Timer.start();
            }
            // warm up shooter if within 2 sec of traj end
            if ((m_auto.m_chosenTrajectory.getTotalTimeSeconds() - m_Timer.get()) < 2) {
                m_Shooter.warmUp();
            } else {
                m_Shooter.turnOff();
            }
            // update with trajectory
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds(m_SwerveDrive.getPose2d()));
            m_Intake.turnOn();
            double targetAngle = -138;
            m_Shooter.setTurretAngle(targetAngle);

        } else if (m_autoStep == 3) {
            // should have picked up 2 balls, or wait 1 second
            if (m_Intake.getNumberOfBallsHolding() > 1 || m_Timer.advanceIfElapsed(0.05)) {
                m_autoStep = 4; 
            }
            m_Shooter.warmUp();
            m_Intake.turnOn();

        } else if (m_autoStep == 4) {
            // shoot balls
            double targetAngle = -138;
            m_Shooter.setTurretAngle(targetAngle);
            // shoot ball holding

//INCREASE SHOOTING RPM BY ABOUT 20
            m_Shooter.shootingRPM(3080, 2800);

            if (m_Shooter.IsOkToShoot() && Math.abs(targetAngle - m_Shooter.getTurretAngleDegrees()) < 5) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
            // when no balls, move to next step
            if (m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 9) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 5;
                    m_Timer.reset();
                    m_Timer.start();
                }
            }
        } else if (m_autoStep == 5) {
            // wait 1/2 second, and setup next trajectory
            if (m_Timer.advanceIfElapsed(0.1)) {
                m_autoStep = 6;
                m_auto.setupAuto1p2();
            }

            m_Shooter.shootingRPM(3080, 2800);
            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (m_autoStep == 6) {
            // follow second trajectory to loading station
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 7;
                m_Timer.reset();
                m_Timer.start();
            }
            // warm up shooter if within 2 sec of traj end
            if ((m_auto.m_chosenTrajectory.getTotalTimeSeconds() - m_Timer.get()) < 2) {
                m_Shooter.warmUp();
            } else {
                m_Shooter.turnOff();
            }
            // update with trajectory
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds(m_SwerveDrive.getPose2d()));
            m_Intake.turnOn();
            m_Shooter.setTurretAngle(-175);

        } else if (m_autoStep == 7) {
            // picked up balls and stopped traj
            if (m_Intake.getNumberOfBallsHolding() > 1 || m_Timer.advanceIfElapsed(0.05)) {
                m_autoStep = 8;
            }
            m_Intake.turnOn();
            m_Shooter.warmUp();

        } else if (m_autoStep == 8) {
            // shoot balls
            double targetAngle = -175;
            m_Shooter.setTurretAngle(targetAngle);
            double angle = m_Shooter.getTurretAngleDegrees();
            if (angle > 0) {
                angle = angle - 360;
            }
            // if (angle < 0) {
            // angle += 360;
            // }
            // shoot ball holding
            m_Shooter.shootingRPM(2650, 2590);

            if (m_Shooter.IsOkToShoot() && Math.abs(targetAngle - angle) < 8) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
            if (m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 10) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 9;
                    m_Timer.reset();
                    m_Timer.start();
                }
            }
        } else if (m_autoStep == 9) {
            if (m_Timer.advanceIfElapsed(10)) {
                m_autoStep = 10;
            }
            m_Shooter.shootingRPM(2650, 2590);
            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (m_autoStep == 10) {
            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();
            SmartDashboard.putNumber("auto time", Timer.getFPGATimestamp() - m_timestamp);
        } else {
            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();
        }

        // always update intake state
        m_Intake.changeState(false, false);
    }

    // public abstract class Person {
    // protected String name;
    // protected int strength;
    // protected double health;

    // public void damagePerson(double damageAmount) {
    // health-=damageAmount;
    // if(health <= 0) {
    // System.out.println("YOU DIE LOWDLWDPWIOWFWDFI");
    // }
    // }

    // public void attackPerson()
    // }

    // public class DumbPerson extends Person {

    // public DumbPerson(String name, int strength, double health) {
    // this.name = name;
    // this.strength = strength;
    // this.health = health;
    // }

    // }

    public void Auto2() {

        if (m_autoStep == 0) {
            m_Shooter.setTurretAngle(175);
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 1;
                m_Timer.reset();
                m_Timer.start();
            }
            m_Intake.turnOn();
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds(m_SwerveDrive.getPose2d()));
        } else if (m_autoStep == 1) {
            m_Shooter.setTurretAngle(175);
            if (m_Intake.getNumberOfBallsHolding() > 1 || m_Timer.advanceIfElapsed(1)) {
                m_autoStep = 2;
            }
            m_SwerveDrive.stopModules();
            m_Intake.turnOn();
        } else if (m_autoStep == 2) {

            double targetAngle = 175;
            m_Shooter.setTurretAngle(targetAngle);
            double turretAngle = m_Shooter.getTurretAngleDegrees();
            if (turretAngle < 0) {
                turretAngle += 360;
            }

            if (Math.abs(targetAngle - turretAngle) < 7) {
                m_autoStep = 3;
            }
            // if(m_VisionShooter.getYaw() <= 0.1) {
            // m_autoStep++;
            // }
        } else if (m_autoStep == 3) {
            m_Shooter.setTurretAngle(175);

            m_Shooter.shootingRPM(2650, 2800);
            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            }
            if (m_Intake.getNumberOfBallsHolding() == 0) {
                m_autoStep++;
                m_Timer.reset();
                m_Timer.start();
            }
            m_SwerveDrive.stopModules();

        } else if (m_autoStep == 4) {
            if (m_Timer.advanceIfElapsed(5)) {
                m_autoStep++;
            }
            m_Shooter.shootingRPM(2650, 2800);
            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            }
            m_SwerveDrive.stopModules();
        } else if (m_autoStep == 5) {
            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();
            m_SwerveDrive.stopModules();
        } else {
            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();
            m_SwerveDrive.stopModules();
        }

        // always update intake state
        m_Intake.changeState(false, false);
    }

    public void Auto3() {
        if (m_autoStep == 0) {
            m_Shooter.setTurretAngle(m_state.getTurretAimingAngle().getDegrees());
            m_Shooter.shootingRPM(ntTopRPM.getNumber(0).doubleValue(), ntBotRPM.getNumber(0).doubleValue());
            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            }
            if (m_Intake.getNumberOfBallsHolding() == 0) {
                m_autoStep = 1;
                m_Timer.reset();
                m_Timer.start();

            }

        } else if (m_autoStep == 1) {
            m_Shooter.shootingRPM(ntTopRPM.getNumber(0).doubleValue(), ntBotRPM.getNumber(0).doubleValue());
            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            }
            if (m_Timer.advanceIfElapsed(0.5)) {
                m_autoStep = 2;
            }
        } else if (m_autoStep == 2) {
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds(m_SwerveDrive.getPose2d()));
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 3;

            }
            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();
        } else if (m_autoStep == 3) {
            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();

        }

        // always update intake state
        m_Intake.changeState(false, false);
    }

    @Override
    public void disabledInit() {
        m_phCompressor.disable();
    }

    @Override
    public void testInit() {
        m_phCompressor.enableAnalog(90, 110);
        m_Shooter.TurnTable(false, false);
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

                // // TURNTABLE

        if (OI.fortFiveTurnTable()) {// A
            m_Shooter.setTurretAngle(0);
        } else if (OI.hundredTurnTable()) {// X
            m_Shooter.setTurretAngle(-90);
        } else if (OI.negFortFiveTurnTable()) {// Y
            m_Shooter.setTurretAngle(180);
        } else if (OI.negHundredTurnTable()) {// B
            m_Shooter.setTurretAngle(90);
        } else if (OI.turntableVoltage()) {
            m_Shooter.setTurntableToNTValue();
        } else if (OI.aimTurretTest()) {
            // m_Shooter.aimTurret(m_VisionShooter.getYaw());
            m_Shooter.aimTurret(m_VisionShooter.getYaw());
        } else {
            m_Shooter.TurnTable(OI.getRightTurntable(),
                    OI.getLeftTurntable());
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
