// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;

import frc.robot.state.RobotState;

/** Add your docs here. */
public class SuperStructure extends Base {

    private static final String kDefaultAuto = "Auto1";
    private static final String kAuto1b = "Auto1b";
    private static final String kCustomAuto = "Auto2";
    private static final String kCustomAuto1 = "Auto3";
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private SwerveDrive m_SwerveDrive;
    private Intake m_Intake;
    private Shooter m_Shooter;
    private Climber m_Climber;
    // private final VisionBall m_VisionBall;
    private final VisionShooter m_VisionShooter = new VisionShooter();

    private int m_zeroBallCounter = 0;
    private double m_timestamp = 0;
    private RobotState m_state;
    private Thread m_cameraThread;
    boolean m_barfTimerStarted = false;
    boolean m_lastButtonWasClimber = false;

    Timer m_Timer = new Timer();
    int m_autoStep = 0;

    private Auto m_auto = new Auto();

    double m_shootingDistance = 0;
    boolean m_shootingDistanceFirstRun = true;

    private final Compressor m_phCompressor = new Compressor(PneumaticsModuleType.REVPH);
    private final PneumaticHub m_ph = new PneumaticHub();

    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("super");
    ShuffleboardTab CompTab = Shuffleboard.getTab("computil");
    // shuffleboard: camera, is aimed, is ok to shoot, how many balls are in intake

    NetworkTableEntry ntBotRPM = CompTab.add("Bot RPM", 5).withPosition(3, 5).withSize(1, 1).getEntry();
    NetworkTableEntry ntTopRPM = CompTab.add("Top RPM", 2.5).withPosition(3, 4).withSize(1, 1).getEntry();
    NetworkTableEntry ntFeetToRPM = CompTab.add("Feet To RPM", 17).withPosition(0, 0).withSize(1, 1)
            .getEntry();
    NetworkTableEntry ntLEDOn = CompTab.add("Limelight LED On", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();

    public SuperStructure(SwerveDrive swerveDrive, Intake intake, Shooter shooter, Climber climber) {

        m_SwerveDrive = swerveDrive;
        m_Intake = intake;
        m_Shooter = shooter;
        m_Climber = climber;
        // m_VisionBall = new VisionBall();

    }

    @Override
    public void robotInit() {
        m_phCompressor.disable();
        m_SwerveDrive.robotInit();
        m_Intake.init();
        m_Shooter.robotInit();
        m_Climber.robotInit();
        m_auto.robotInit();
        // m_VisionBall.robotInit();
        m_VisionShooter.robotInit();

        m_cameraThread = new Thread(
                () -> {
                    UsbCamera camera = CameraServer.startAutomaticCapture();
                    camera.setResolution(320, 240);
                });
        m_cameraThread.setDaemon(true);
        m_cameraThread.start();

        // Shuffleboard.getTab("super").add("compressor",
        // m_phCompressor).withPosition(12, 0).withSize(1, 1);
        Shuffleboard.getTab("super").addBoolean("IsOkToShoot", () -> m_Shooter.IsOkToShoot()).withPosition(0, 4);

        Shuffleboard.getTab("super").addNumber("Camera Based Distance", () -> m_VisionShooter.getDistance())
                .withPosition(5, 0).withSize(2, 1);
        Shuffleboard.getTab("computil").add("swervedrive", m_SwerveDrive).withPosition(8, 1).withSize(2, 2);

        Shuffleboard.getTab("debug").addNumber("Camera to Target Yaw", () -> m_VisionShooter.getYaw());
        Shuffleboard.getTab("debug").addNumber("Turret Angle", () -> m_Shooter.getTurretAngleDegrees());
        Shuffleboard.getTab("debug").addNumber("Turret CANcoder", () -> m_Shooter.getTurretAngleCANcoder());
        Shuffleboard.getTab("debug").addNumber("autoStep", () -> m_autoStep);

        Shuffleboard.getTab("super").addNumber("current top RPM", () -> m_Shooter.getTopMotorRPM()).withPosition(0, 0);
        Shuffleboard.getTab("super").addNumber("Current bot RPM", () -> m_Shooter.getBotMotorRPM()).withPosition(1, 0);
        Shuffleboard.getTab("super").addBoolean("Is Ball in Loader", () -> m_Intake.isBallAtLoad()).withPosition(0, 1);
        Shuffleboard.getTab("super").addBoolean("Is Ball in Middle", () -> m_Intake.isBallAtMiddle()).withPosition(0,
                2);
        Shuffleboard.getTab("super").addBoolean("Both balls loaded",
                () -> m_Intake.isBallAtMiddle() && m_Intake.isBallAtLoad()).withPosition(0, 3);
        // Shuffleboard.getTab("super").add(CameraServer.putVideo("limelight", 320,
        // 240)).withPosition(6, 1).withSize(5,
        // 6);
        // Shuffleboard.getTab("super").add(CameraServer.putVideo("USB Camera 0", 320,
        // 240)).withPosition(1, 1).withSize(6,
        // 5);
        Shuffleboard.getTab("super").addNumber("pressure", () -> m_phCompressor.getPressure()).withPosition(4, 0);

        // auto chooser
        m_chooser.setDefaultOption(kDefaultAuto, kDefaultAuto);
        m_chooser.addOption(kAuto1b, kAuto1b);
        m_chooser.addOption(kCustomAuto, kCustomAuto);
        m_chooser.addOption(kCustomAuto1, kCustomAuto1);
        Shuffleboard.getTab("super").add("Auto Choose", m_chooser).withSize(1, 1).withPosition(0, 5);

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

        // climber or shooter check
        if (OI.aimWithPose() || OI.getAimTurret() || OI.getRightBumperForWallShot() || OI.getLeftBumperForTapeShot()
                || OI.getRightTriggerForShooting() || OI.getUnjam() || OI.getBarf() || OI.startIntake()) {
            m_lastButtonWasClimber = false;
        } else if (OI.getArmsDown() || OI.getArmsUp() || OI.getSolenoidForward() || OI.getSolenoidReverse()
                || OI.getNxtClimb() || OI.getFinClimb()) {
            m_lastButtonWasClimber = true;
        }
    }

    @Override
    public void teleopInit() {
        m_Shooter.turnOff();
        m_Intake.turnOffLoadShooter();
        m_Climber.hooksReverse();
        m_Climber.eStop();
        m_shootingDistanceFirstRun = true;
        m_VisionShooter.LEDon();
    }

    @Override
    public void teleopPeriodic() {
        m_phCompressor.enableAnalog(90, 112);
        m_VisionShooter.LEDon();

        // CLIMBER
        m_Climber.control(OI.getSolenoidReverse(), OI.getSolenoidForward(), OI.getArmsUp(),
                OI.getArmsDown(), OI.getNxtClimb(), OI.getFinClimb(), OI.getClimbEStop());

        // testing purposaes, changes intake pnuematics
        if (OI.forwardIntake()) {
            m_Intake.setIntakePnuematic(true);
        } else if (OI.reverseIntake()) {
            m_Intake.setIntakePnuematic(false);
        }

        // DRIVING //
        // VISION GET BALL
        if (OI.getVisionBallEngaged()) {
            // ChassisSpeeds speeds = m_VisionBall.followBall();
            ChassisSpeeds speeds = new ChassisSpeeds();
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
            // m_Shooter.shootingRPM(3000, 2400);
            // m_Shooter.shootingRPM(ntTopRPM.getDouble(0), ntBotRPM.getDouble(0));
            if (m_shootingDistanceFirstRun) {
                m_shootingDistance = m_VisionShooter.getDistance();
                m_shootingDistanceFirstRun = false;
            }
            m_Shooter.shootingDist(m_shootingDistance);
            if (m_Shooter.IsOkToShoot()) {
                // Load shooter
                m_Intake.loadShooter();

            } else {
                m_Intake.countinueIntakeMotors();
            }
        } else if (OI.getRightBumperForWallShot()) {
            // turn shooter on in rpm mode
            // m_Shooter.shootingRPM(2500, 2650);
            m_Shooter.shootingRPM(3000, 2420);

            if (m_Shooter.IsOkToShoot()) {
                // Load shooter
                m_Intake.loadShooter();

            } else {
                m_Intake.countinueIntakeMotors();
            }
        } else if (OI.getLeftBumperForTapeShot()) {
            // 4000, 2900 for vision target on bot of screen
            // m_Shooter.shootingRPM(5100, 3000);
            m_Shooter.shootingRPM(2500, 2650); // tape speeds
            if (m_Shooter.IsOkToShoot()) {
                // load shooter
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (OI.xboxDrive.getYButton()) {
            // m_Shooter.shootingDist(ntFeetToRPM.getDouble(0));
            // m_Shooter.setVoltage(ntTopRPM.getDouble(0), ntBotRPM.getDouble(0));
            m_Shooter.shootingRPM(ntTopRPM.getDouble(0.0), ntBotRPM.getDouble(0.0));
            if (m_Shooter.IsOkToShoot()) {
                // if (OI.getAimTurret()) {

                // load shooter
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (DriverStation.isFMSAttached() && !m_lastButtonWasClimber) {
            // speed up shooter automatically
            m_Shooter.shootingDist(8);
        } else if (OI.getAimTurret()) {
            m_Shooter.shootingDist(8);
        } else {
            m_Shooter.turnOff();
        }

        // when shooting released, stop loading
        if (OI.getShootingReleased()) {
            m_Intake.turnOffLoadShooter();
            m_shootingDistanceFirstRun = true;
        }

        // // TURNTABLE
        if (m_lastButtonWasClimber) {
            m_Shooter.setTurretAngle(90);
        } else if (OI.getAimTurret()) {
            // m_Shooter.aimTurret(m_VisionShooter.getYaw());
            if (m_VisionShooter.getTargetValid()) {
                m_Shooter.aimTurret(m_VisionShooter.getYaw());
            } else {
                m_Shooter.setTurretAngle(m_state.getTurretAimingAngle().getDegrees());
            }
        } else if (OI.aimWithPose()) {
            m_Shooter.setTurretAngle(m_state.getTurretAimingAngle().getDegrees());
        } else if (m_Intake.getNumberOfBallsHolding() > 0 && !(OI.getRightTurntable() ||
                OI.getLeftTurntable())) {
            m_Shooter.setTurretAngle(m_state.getTurretAimingAngle().getDegrees());
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
    }

    // auto coding

    @Override
    public void autonomousInit() {
        m_autoStep = 0;
        m_auto.autonomousInit();
        m_Intake.autonomousInit();
        m_Climber.hooksReverse();
        m_Climber.eStop();
        m_VisionShooter.LEDon();

        m_timestamp = Timer.getFPGATimestamp();

        PathPlannerState state = new PathPlannerState();
        state.poseMeters = new Pose2d();
        state.holonomicRotation = new Rotation2d();

        if (m_chooser.getSelected() == kDefaultAuto) {
            m_auto.setupAuto1p1();
        } else if (m_chooser.getSelected() == kAuto1b) {
            m_auto.setupAuto1bp1();
        } else if (m_chooser.getSelected() == kCustomAuto) {
            m_auto.setupAuto2();
        } else if (m_chooser.getSelected() == kCustomAuto1) {
            m_auto.setupAuto3();
        }
        state = m_auto.getInitialState();

        m_SwerveDrive.resetOdometry(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));

        m_Timer.reset();
    }

    @Override
    public void autonomousPeriodic() {
        if (m_chooser.getSelected() == kDefaultAuto) {
            Auto1();
        } else if (m_chooser.getSelected() == kAuto1b) {
            Auto1b();
        } else if (m_chooser.getSelected() == kCustomAuto) {
            Auto2();
        } else if (m_chooser.getSelected() == kCustomAuto1) {
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
            // m_Shooter.shootingRPM(1000, 3250);
            if (m_shootingDistanceFirstRun) {
                m_shootingDistance = m_VisionShooter.getDistance();
                m_shootingDistanceFirstRun = false;
            }
            m_Shooter.shootingDist(8);
            if (m_Shooter.IsOkToShoot() && Math.abs(targetAngle - angle) < 7) {
                m_Intake.loadShooter();
            }
            // when no more balls, go to next step
            if (m_Shooter.IsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 7) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 2;
                    m_auto.setupAuto1p1();
                    m_Timer.reset();
                    m_Timer.start();
                    m_shootingDistanceFirstRun = true;

                }
            }
        } else if (m_autoStep == 1) {
            // wait until all balls thru shooter
            // if (m_Timer.advanceIfElapsed(0.1)) {
            // m_autoStep = 2;
            // m_auto.setupAuto1p1();
            // }
            // m_Shooter.shootingDist(m_VisionShooter.getDistance());
            // if (m_Shooter.IsOkToShoot()) {
            // m_Intake.loadShooter();
            // }

            // INCREASE TIME THAT SHOOTER IS RUNNING BEFORE BEGINNING NEXT STEP IN AUTO (+1
            // to 2 sec)
            // SHOOTER NOT SHOOTING ALL BALLS BEFORE MOVING ON
        } else if (m_autoStep == 2) {
            // follow first trajectory to pick up 2 balls
            // move to next step when traj complete
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 4;
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
            // if (m_Intake.getNumberOfBallsHolding() > 1 || m_Timer.advanceIfElapsed(0.05))
            // {
            // m_autoStep = 4;
            // }
            // m_Shooter.warmUp();
            // m_Intake.turnOn();

        } else if (m_autoStep == 4) {
            // shoot balls
            // double targetAngle = -138;
            // m_Shooter.setTurretAngle(targetAngle);
            m_Shooter.aimTurret(m_VisionShooter.getYaw());

            // shoot ball holding

            // INCREASE SHOOTING RPM BY ABOUT 20
            // m_Shooter.shootingRPM(3080, 2800);
            if (m_shootingDistanceFirstRun) {
                m_shootingDistance = m_VisionShooter.getDistance();
                m_shootingDistanceFirstRun = false;
            }
            m_Shooter.shootingDist(11);

            // if (m_Shooter.IsOkToShoot() && Math.abs(targetAngle -
            // m_Shooter.getTurretAngleDegrees()) < 5) {
            if (m_Shooter.IsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
            // when no balls, move to next step
            if (m_Shooter.IsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 9) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 5;
                    m_Timer.reset();
                    m_Timer.start();
                    m_shootingDistanceFirstRun = true;
                }
            }
        } else if (m_autoStep == 5) {
            // wait 1/2 second, and setup next trajectory
            if (m_Timer.advanceIfElapsed(0.1)) {
                m_autoStep = 6;
                m_auto.setupAuto1p2();
            }

            // m_Shooter.shootingRPM(3080, 2800);
            m_Shooter.shootingDist(m_shootingDistance);

            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (m_autoStep == 6) {
            // follow second trajectory to loading station
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 8;
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
            m_Shooter.setTurretAngle(185);

        } else if (m_autoStep == 7) {
            // picked up balls and stopped traj
            // if (m_Intake.getNumberOfBallsHolding() > 1 || m_Timer.advanceIfElapsed(0.05))
            // {
            // m_autoStep = 8;
            // }
            // m_Intake.turnOn();
            // m_Shooter.warmUp();

        } else if (m_autoStep == 8) {
            // shoot balls
            // double targetAngle = 185;
            // m_Shooter.setTurretAngle(targetAngle);
            m_Shooter.aimTurret(m_VisionShooter.getYaw());
            if (m_shootingDistanceFirstRun && m_VisionShooter.getYaw() < 3) {
                m_shootingDistance = m_VisionShooter.getDistance();
                m_shootingDistanceFirstRun = false;
            }
            m_Shooter.shootingDist(m_shootingDistance);

            // double angle = m_Shooter.getTurretAngleDegrees();
            // if (angle > 0) {
            // angle = angle - 360;
            // }
            // if (angle < 0) {
            // angle += 360;
            // }
            // shoot ball holding
            // m_Shooter.shootingRPM(2650, 2590);

            // if (m_Shooter.IsOkToShoot() && Math.abs(targetAngle - angle) < 8) {
            if (m_Shooter.IsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
            if (m_Shooter.IsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 6) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 9;
                    m_Timer.reset();
                    m_Timer.start();
                    m_shootingDistanceFirstRun = true;
                }
            }
        } else if (m_autoStep == 9) {
            SmartDashboard.putNumber("auto time", Timer.getFPGATimestamp() - m_timestamp);
            if (m_Timer.advanceIfElapsed(10)) {
                m_autoStep = 10;
            }
            m_Shooter.shootingDist(m_shootingDistance);
            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (m_autoStep == 10) {
            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();
        } else {
            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();
        }

        // always update intake state
        m_Intake.changeState(false, false);
    }

    public void Auto1b() {
        // default turret aiming code here

        // end turret code

        if (m_autoStep == 0) {
            // follow auto1b_part1 trajectory to pick up 1 balls
            // move to next step when traj complete
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 1;
                m_Timer.reset();
                m_Timer.start();
            }
            // always preheat shooter
            m_Shooter.warmUp();

            // update with trajectory
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds(m_SwerveDrive.getPose2d()));

            // intake on
            m_Intake.turnOn();
            // double targetAngle = -138;
            // m_Shooter.setTurretAngle(targetAngle);

        } else if (m_autoStep == 1) {
            // trajectory over, holding 2 balls, shoot them
            // double targetAngle = 168;
            // m_Shooter.setTurretAngle(targetAngle);
            // double angle = m_Shooter.getTurretAngleDegrees();
            // if (angle < 0) {
            //     angle += 360;
            // }
            // shoot ball holding
            // if (m_shootingDistanceFirstRun) {
            //     m_shootingDistance = m_VisionShooter.getDistance();
            //     m_shootingDistanceFirstRun = false;
            // }
            m_Shooter.shootingDist(Units.metersToFeet(3.13));
            // if (m_Shooter.IsOkToShoot() && Math.abs(targetAngle - angle) < 7) {
            if (m_Shooter.IsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 4) {
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }
            // when no more balls, go to next step
            if (m_Shooter.IsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 7) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 2;
                    m_auto.setupAuto1p1();
                    m_Timer.reset();
                    m_Timer.start();
                    m_shootingDistanceFirstRun = true;

                }
            }
        } else if (m_autoStep == 2) {
            // follow first trajectory to pick up 2 balls
            // move to next step when traj complete
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 4;
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
            // if (m_Intake.getNumberOfBallsHolding() > 1 || m_Timer.advanceIfElapsed(0.05))
            // {
            // m_autoStep = 4;
            // }
            // m_Shooter.warmUp();
            // m_Intake.turnOn();

        } else if (m_autoStep == 4) {
            // shoot balls
            // double targetAngle = -138;
            // m_Shooter.setTurretAngle(targetAngle);
            m_Shooter.aimTurret(m_VisionShooter.getYaw());

            // shoot ball holding

            // INCREASE SHOOTING RPM BY ABOUT 20
            // m_Shooter.shootingRPM(3080, 2800);
            if (m_shootingDistanceFirstRun) {
                m_shootingDistance = m_VisionShooter.getDistance();
                m_shootingDistanceFirstRun = false;
            }
            m_Shooter.shootingDist(11);

            // if (m_Shooter.IsOkToShoot() && Math.abs(targetAngle -
            // m_Shooter.getTurretAngleDegrees()) < 5) {
            if (m_Shooter.IsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
            // when no balls, move to next step
            if (m_Shooter.IsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 9) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 5;
                    m_Timer.reset();
                    m_Timer.start();
                    m_shootingDistanceFirstRun = true;
                }
            }
        } else if (m_autoStep == 5) {
            // wait 1/2 second, and setup next trajectory
            if (m_Timer.advanceIfElapsed(0.1)) {
                m_autoStep = 6;
                m_auto.setupAuto1p2();
            }

            // m_Shooter.shootingRPM(3080, 2800);
            m_Shooter.shootingDist(m_shootingDistance);

            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (m_autoStep == 6) {
            // follow second trajectory to loading station
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 8;
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
            m_Shooter.setTurretAngle(185);

        } else if (m_autoStep == 7) {
            // picked up balls and stopped traj
            // if (m_Intake.getNumberOfBallsHolding() > 1 || m_Timer.advanceIfElapsed(0.05))
            // {
            // m_autoStep = 8;
            // }
            // m_Intake.turnOn();
            // m_Shooter.warmUp();

        } else if (m_autoStep == 8) {
            // shoot balls
            // double targetAngle = 185;
            // m_Shooter.setTurretAngle(targetAngle);
            m_Shooter.aimTurret(m_VisionShooter.getYaw());
            if (m_shootingDistanceFirstRun && m_VisionShooter.getYaw() < 3) {
                m_shootingDistance = m_VisionShooter.getDistance();
                m_shootingDistanceFirstRun = false;
            }
            m_Shooter.shootingDist(m_shootingDistance);

            // double angle = m_Shooter.getTurretAngleDegrees();
            // if (angle > 0) {
            // angle = angle - 360;
            // }
            // if (angle < 0) {
            // angle += 360;
            // }
            // shoot ball holding
            // m_Shooter.shootingRPM(2650, 2590);

            // if (m_Shooter.IsOkToShoot() && Math.abs(targetAngle - angle) < 8) {
            if (m_Shooter.IsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
            if (m_Shooter.IsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 6) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 9;
                    m_Timer.reset();
                    m_Timer.start();
                    m_shootingDistanceFirstRun = true;
                }
            }
        } else if (m_autoStep == 9) {
            SmartDashboard.putNumber("auto time", Timer.getFPGATimestamp() - m_timestamp);
            if (m_Timer.advanceIfElapsed(10)) {
                m_autoStep = 10;
            }
            m_Shooter.shootingDist(m_shootingDistance);
            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (m_autoStep == 10) {
            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();
        } else {
            m_Shooter.turnOff();
            m_Intake.turnOffLoadShooter();
        }

        // always update intake state
        m_Intake.changeState(false, false);
    }

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
        // m_VisionShooter.LEDoff();
    }

    @Override
    public void disabledPeriodic() {
        if (ntLEDOn.getBoolean(true)) {
            m_VisionShooter.LEDon();
        } else {
            m_VisionShooter.LEDoff();
        }
    }

    @Override
    public void testInit() {
        m_phCompressor.enableAnalog(90, 110);
        m_Shooter.TurnTable(false, false);
        m_VisionShooter.LEDon();
        m_Climber.hooksReverse();
        m_Climber.eStop();
    }

    @Override
    public void testPeriodic() {
        // CLIMBER
        m_Climber.control(OI.getSolenoidReverse(), OI.getSolenoidForward(), OI.getArmsUp(), OI.getArmsDown(),
                OI.getNxtClimb(), OI.getFinClimb(), OI.getClimbEStop());

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

        // if (OI.fortFiveTurnTable()) {// A
        // m_Shooter.setTurretAngle(0);
        // } else if (OI.hundredTurnTable()) {// X
        // m_Shooter.setTurretAngle(-90);
        // } else if (OI.negFortFiveTurnTable()) {// Y
        // m_Shooter.setTurretAngle(180);
        // } else if (OI.negHundredTurnTable()) {// B
        // m_Shooter.setTurretAngle(90);
        // } else if (OI.turntableVoltage()) {
        // m_Shooter.setTurntableToNTValue();
        // } else if (OI.aimTurretTest()) {
        // // m_Shooter.aimTurret(m_VisionShooter.getYaw());
        // m_Shooter.aimTurret(m_VisionShooter.getYaw());
        // } else {
        // m_Shooter.TurnTable(OI.getRightTurntable(),
        // OI.getLeftTurntable());
        // }
    }
}
