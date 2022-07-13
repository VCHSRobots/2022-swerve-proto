// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.state.RobotState;
import frc.robot.subsystems.*;

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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

/** Add your docs here. */
public class SuperStructure extends Base {

    private static final String kAuto1b = "Auto1b";
    private static final String kAuto2 = "Auto2";
    private static final String kAutoMarsRock = "MarsRock";
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
    boolean m_autoAimEnabled = true;
    boolean m_turretClearForShot = false;
    boolean toggleShootAndRun = false;

    Timer m_Timer = new Timer();
    int m_autoStep = 0;

    private Auto m_auto = new Auto();

    double m_shootingDistance = 0;
    double desiredAngle = 0;
    int loopCounter = 0;
    double m_visionTeleopOffset = 1.5;

    private final Compressor m_phCompressor = new Compressor(PneumaticsModuleType.REVPH);
    private final PneumaticHub m_ph = new PneumaticHub(); // even though not used, keep here

    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("super");
    ShuffleboardTab CompTab = Shuffleboard.getTab("computil");
    // shuffleboard: camera, is aimed, is ok to shoot, how many balls are in intake

    NetworkTableEntry ntBotRPM = CompTab.add("Bot RPM", 5).withPosition(3, 5).withSize(1, 1).getEntry();
    NetworkTableEntry ntTopRPM = CompTab.add("Top RPM", 2.5).withPosition(3, 4).withSize(1, 1).getEntry();
    NetworkTableEntry ntFeetToRPM = CompTab.add("Feet To RPM", 17).withPosition(0, 0).withSize(1, 1)
            .getEntry();
    NetworkTableEntry ntLEDOn = CompTab.add("Limelight LED On", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();
    NetworkTableEntry ntShooterPreheatEnable = CompTab.add("Shooter Preheat", false)
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
        Shuffleboard.getTab("super").addBoolean("IsOkToShoot", () -> m_Shooter.ShooterIsOkToShoot()).withPosition(0, 4);

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
        m_chooser.setDefaultOption(kAuto1b, kAuto1b);
        m_chooser.addOption(kAuto2, kAuto2);
        m_chooser.addOption(kAutoMarsRock, kAutoMarsRock);
        Shuffleboard.getTab("super").add("Auto Choose", m_chooser).withSize(1, 1).withPosition(0, 5);

        m_state = new RobotState(m_SwerveDrive.getPose2d(), Rotation2d.fromDegrees(m_Shooter.getTurretAngleDegrees()));
    }

    @Override
    public void robotPeriodic() {
        m_SwerveDrive.changeOdometry(OI.shouldSetFieldRelative(), OI.shouldSetRobotRelative(),
                OI.getResetOdometryLaunchPad(), OI.getResetOdometryRightFender());
        m_Intake.robotPeriodic();
        m_auto.robotPeriodic();
        m_VisionShooter.calculateAngleError();
        m_VisionShooter.addDistanceToMovingAverage();

        m_Climber.checkZero();

        m_state.update(m_SwerveDrive.getPose2d(), Rotation2d.fromDegrees(m_Shooter.getTurretAngleDegrees()));

        // climber or shooter check
        if (OI.getAimTurret() || OI.getRightBumperForLaunchShot() || OI.getLeftBumperForTapeShot()
                || OI.getRightTriggerForShooting() || OI.getUnjam() || OI.getBarf() || OI.startIntake()
                || OI.getLeftTurntable() || OI.getRightTurntable()) {
            m_lastButtonWasClimber = false;
        } else if (OI.getArmsDown() || OI.getArmsUp() || OI.getSolenoidForward() || OI.getSolenoidReverse()
                || OI.getNxtClimb() || OI.getFinClimb() || OI.getClimbArmSpeedUp() > 0.2
                || OI.getClimbArmSpeedDown() > 0.2) {
            m_lastButtonWasClimber = true;
        }
    }

    @Override
    public void teleopInit() {
        m_Shooter.turnOff();
        m_Intake.turnOffLoadShooter();
        m_Climber.hooksReverse();
        m_Climber.eStop();
        m_VisionShooter.LEDon();
        m_VisionShooter.setOffset(m_visionTeleopOffset);
        m_SwerveDrive.setFieldRelative();
        m_autoAimEnabled = true;
        toggleShootAndRun = false;
    }

    @Override
    public void teleopPeriodic() {
        m_phCompressor.enableAnalog(90, 112);
        m_VisionShooter.LEDon();

        // CLIMBER
        // Send inputs to climber control.
        m_Climber.control(OI.getSolenoidReverse(), OI.getSolenoidForward(), OI.getArmsUp(), OI.getArmsDown(),
                OI.getNxtClimb(), OI.getFinClimb(), OI.getClimbArmSpeedDown(), OI.getClimbArmSpeedUp());

        // INTAKE STATE UPDATE
        m_Intake.changeState(OI.startIntake(), OI.stopIntake());

        // INTAKE / SHOOTING
        // If any climbing functions are active, turn off shooter wheels.
        if (OI.getUnjam()) {
            m_Intake.unjamShooter();

        } else if (OI.getBarf()) {
            m_Shooter.setBarfVoltage();
            if (m_Shooter.isSpinningFastEnoughForBarf()
                    && Math.abs(m_Shooter.getTurretAngleDegrees() - 180) < 4) {
                m_Intake.loadShooter();
            } else {
                m_Intake.turnOffLoadShooter();
            }
        } else if (OI.getRightTriggerForShooting()) {
            // camera center shot
            m_Shooter.shootingDist(m_VisionShooter.getMovingAverageDistance());
            if (m_Shooter.ShooterIsOkToShoot() && m_VisionShooter.isOnTarget()) {
                // Load shooter
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }
        } else if (OI.getRightBumperForLaunchShot()) {
            // turn shooter on in rpm mode
            m_Shooter.shootingRPM(4000, 3200);

            if (m_Shooter.ShooterIsOkToShoot()) {
                // Load shooter
                m_Intake.loadShooter();

            } else {
                m_Intake.countinueIntakeMotors();
            }
            // } else if (OI.getLeftBumperForTapeShot()) {
            // // 4000, 2900 for vision target on bot of screen
            // // m_Shooter.shootingRPM(5100, 3000);
            // m_Shooter.shootingRPM(2500, 2650); // tape speeds
            // if (m_Shooter.IsOkToShoot()) {
            // // load shooter
            // m_Intake.loadShooter();
            // } else {
            // m_Intake.turnOffLoadShooter();
            // }

        } else if (OI.ShootAndRun()) {
            toggleShootAndRun = !toggleShootAndRun;
        } else if (toggleShootAndRun) {
            // System.out.println("stable " + m_state.robotHasStableVelocity());
            // System.out.println("Shooter Vel " + m_Shooter.canShootWithVelocity());
            // System.out.println("Turret Good " + m_VisionShooter.withinErrWithTime());
            // System.out.println("Reasonable Dist " + m_state.isResonableDist(m_VisionShooter.getDistance()));

            // System.out.println(desiredAngle);
            // System.out.println("desired: " + (desiredAngle - m_state.turretDegreesOverEstimate()));
            // System.out.println("newDesired: " + desiredAngle);
            // System.out.println("actual: " + m_VisionShooter.getYaw());

            if (m_state.robotHasStableVelocity() && m_Shooter.canShootWithVelocity()
                    && m_VisionShooter.isWithinTurretErr(desiredAngle) && m_state.isResonableDist(m_VisionShooter.getPredictedDistFromTarget())
                    && m_VisionShooter.canSeeTarget()) {
                // System.out.println("true");
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }
            desiredAngle = m_state.turretDegreesDesired();
            m_Shooter.shootingDist(m_state.shooterDistDesired(m_VisionShooter.getMovingAverageDistance()));
            // m_Shooter.shootingDist(m_VisionShooter.limeShooterDistDesired());

            System.out.println("Desired: " + desiredAngle);
            System.out.println("Vision: " + m_VisionShooter.getYaw());
            // System.out.println("Overestimate: " + m_state.turretDegreesOverEstimate(m_VisionShooter.m_offset));
        
        } else if (m_lastButtonWasClimber) {
            m_Shooter.turnOff();
        } else if ((ntShooterPreheatEnable.getBoolean(false)
                || DriverStation.isFMSAttached()) && !m_lastButtonWasClimber) {
            // speed up shooter automatically
            m_Shooter.shootingDist(8.25);
        } else if (OI.getAimTurret()) {
            m_Shooter.shootingDist(8.25);
        } else if (m_Intake.getBothBallsLoaded() && !m_lastButtonWasClimber) {
            // speed up shooter automatically
            m_Shooter.shootingDist(8.25);
            // } else if (OI.driveAndShoot()) {
            // m_Shooter.shootingDist(m_state.getVelocityTurretDegrees());
            // if (m_Shooter.canShootWithVelocity()) {
            // m_Intake.loadShooter();
            // } else {
            // m_Intake.turnOffLoadShooter();
            // }
        } else {
            m_Shooter.turnOff();
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
        
        // System.out.println(m_SwerveDrive.getPose2d());
        // if (m_VisionShooter.isOnTarget() && loopCounter % 25 == 0) {
        //     System.out.println("changing to ");
        //     Transform2d newRobotPose = m_state.resetRobotPoseOdometry(m_VisionShooter.getDistance(), m_Shooter.getTurretAngleDegrees(), m_VisionShooter.getActualYaw(), m_SwerveDrive.getPose2d().getRotation().getDegrees());
        //     m_SwerveDrive.resetOdometry(new Pose2d(newRobotPose.getX(), newRobotPose.getY(), m_SwerveDrive.getPose2d().getRotation()));
        // }

        // // TURNTABLE
        if (OI.getBarf()) {
            m_Shooter.setTurretAngle(180);
        } else if (m_lastButtonWasClimber) {
            if (Math.abs(m_Shooter.getTurretAngleDegrees() - 90) > 5) {
                m_Shooter.setTurretAngle(90);
            }
        } else if (OI.getAimTurret()) {
            // aimTurretAuto();
            if (m_VisionShooter.getTargetValid()) {
                m_Shooter.aimTurret(m_VisionShooter.getYaw());
            }

        } else if (toggleShootAndRun) {
            if (m_VisionShooter.getTargetValid()) {
                // m_Shooter.aimTurret(m_state.turretDegreesDesired());
                // m_VisionShooter.setOffset(desiredAngle);
                // m_Shooter.aimTurret(m_VisionShooter.getYaw() - m_state.turretDegreesOverEstimate());
                m_Shooter.aimTurret(m_VisionShooter.getYaw() + m_state.turretDegreesDesired());

                // System.out.println("Yep: " + (desiredAngle - m_state.turretDegreesOverEstimate()));
                // System.out.println(desiredAngle);
                // System.out.println("Light " + m_VisionShooter.getYaw());
                // System.out.println(m_VisionShooter.m_offset);
            } else {
                aimTurretAuto();
            }
        } else if (OI.ShootAndRun()) {
            toggleShootAndRun = !toggleShootAndRun;
            if (toggleShootAndRun == false) {
                m_VisionShooter.setOffset(m_visionTeleopOffset);
            } else {
                m_VisionShooter.setOffset(0.0);
            }
        } else if (m_autoAimEnabled) {
            aimTurretAuto();
        } else {
            m_Shooter.TurnTable(OI.getRightTurntable(),
                    OI.getLeftTurntable());
        }

        if (toggleShootAndRun == false) {
            m_VisionShooter.setOffset(m_visionTeleopOffset);
        }

        // when shooting released, stop loading
        if (OI.getShootingReleased()) {
            m_Intake.turnOffLoadShooter();
        }

        if (OI.getYforDisableAutoAim()) {
            m_autoAimEnabled = false;
        }

        if (OI.shouldSetFieldRelative() || OI.getResetOdometryLaunchPad() || OI.getResetOdometryRightFender()) {
            m_autoAimEnabled = true;
        }

    }

    public void aimTurretAuto() {
        // if (m_VisionShooter.getTargetValid()) {
        // m_Shooter.aimTurret(m_VisionShooter.getYaw());
        // } else {
        // m_Shooter.setTurretAngle(m_state.getTurretAimingAngle().getDegrees());
        // TODO: try this below
        m_Shooter.setTurretAngle(m_state.getTurretAimingAngle(m_SwerveDrive.getAngVel()).getDegrees());
        // }
    }

    // auto coding

    @Override
    public void autonomousInit() {
        m_autoStep = 0;
        m_autoAimEnabled = true;
        m_auto.autonomousInit();
        m_Intake.autonomousInit();
        m_Climber.hooksReverse();
        m_Climber.eStop();
        m_VisionShooter.LEDon();
        m_VisionShooter.setOffset(3);
        m_SwerveDrive.setFieldRelative();

        m_timestamp = Timer.getFPGATimestamp();

        PathPlannerState state = new PathPlannerState();
        state.poseMeters = new Pose2d();
        state.holonomicRotation = new Rotation2d();

        if (m_chooser.getSelected() == kAuto1b) {
            m_auto.setupAuto1bp1();
        } else if (m_chooser.getSelected() == kAuto2) {
            m_auto.setupAuto2();
        } else if (m_chooser.getSelected() == kAutoMarsRock) {

        }
        state = m_auto.getInitialState();

        m_SwerveDrive.resetOdometry(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));

        m_Timer.reset();
    }

    @Override
    public void autonomousPeriodic() {
        if (m_chooser.getSelected() == kAuto1b) {
            Auto1b();
        } else if (m_chooser.getSelected() == kAuto2) {
            Auto2();
        } else if (m_chooser.getSelected() == kAutoMarsRock) {
            Auto3();
        }
    }

    public void Auto1b() {
        // default turret aiming code here
        aimTurretAuto();
        // end turret code
        double firstShotMeters = 3.0;
        double secondShotMeters = 3.7;
        double thirdShotMeters = 3.45;

        if (m_autoStep == 0) {
            // follow auto1b_part1 trajectory to pick up 1 balls
            // move to next step when traj complete
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 1;
                m_Timer.reset();
                m_Timer.start();
                m_Intake.countinueIntakeMotors();
            }
            // always preheat shooter
            // m_Shooter.warmUp();
            m_Shooter.shootingDist(Units.metersToFeet(firstShotMeters));

            // update with trajectory
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds(m_SwerveDrive.getPose2d()));

            // intake on
            m_Intake.turnOn();

        } else if (m_autoStep == 1) {
            // shoot 2 balls
            m_Shooter.shootingDist(Units.metersToFeet(firstShotMeters));
            if (m_Shooter.ShooterIsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }
            // when no more balls, go to next step
            if (m_Shooter.ShooterIsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 7) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 2;
                    m_auto.setupAuto1bp2();
                    m_Timer.reset();
                    m_Timer.start();
                }
            }
        } else if (m_autoStep == 2) {
            // follow second trajectory to pick up 1 ball
            // move to next step when traj complete
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 3;
                m_Timer.reset();
                m_Timer.start();
            }
            // m_Shooter.warmUp();
            m_Shooter.shootingDist(Units.metersToFeet(secondShotMeters));

            // update with trajectory
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds(m_SwerveDrive.getPose2d()));

            m_Intake.turnOn();

        } else if (m_autoStep == 3) {
            // delay to pick up ball
            if (m_Timer.hasElapsed(0.9) || m_Intake.getNumberOfBallsHolding() > 0) {
                m_autoStep = 4;
                m_Timer.reset();
                m_Timer.start();
                m_Intake.countinueIntakeMotors();
            }
            m_Shooter.shootingDist(Units.metersToFeet(secondShotMeters));
            m_Intake.turnOn();
            m_SwerveDrive.drive(0, 0, 0, false);
        } else if (m_autoStep == 4) {
            // shoot 1 ball
            m_Shooter.shootingDist(Units.metersToFeet(secondShotMeters));
            if (m_Shooter.ShooterIsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }
            // when no more balls, go to next step
            if (m_Shooter.ShooterIsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 7) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 5;
                    m_auto.setupAuto1bp3();
                    m_Timer.reset();
                    m_Timer.start();
                }
            }

        } else if (m_autoStep == 5) {
            // follow third trajectory to pick up 2 balls from loading station
            // move to next step when traj complete
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 6;
                m_Timer.reset();
                m_Timer.start();
            }
            m_Shooter.warmUp();

            // update with trajectory
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds(m_SwerveDrive.getPose2d()));
            m_Intake.turnOn();

        } else if (m_autoStep == 6) {
            // wait at loading station
            if (m_Timer.hasElapsed(1.0) || m_Intake.getNumberOfBallsHolding() == 2) {
                m_autoStep = 7;
                m_Timer.reset();
                m_Timer.start();
                m_auto.setupAuto1bp4();
            }
            m_Shooter.warmUp();
            m_SwerveDrive.drive(0, 0, 0, false);
            m_Intake.turnOn();

        } else if (m_autoStep == 7) {
            // 4th trajectory from load station to hub
            // move to next step when traj complete
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 8;
                m_Timer.reset();
                m_Timer.start();
            }
            // m_Shooter.warmUp();
            m_Shooter.shootingDist(Units.metersToFeet(thirdShotMeters));

            // update with trajectory
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds(m_SwerveDrive.getPose2d()));
            m_Intake.turnOn();

        } else if (m_autoStep == 8) {
            // shoot last 2 balls
            m_Shooter.shootingDist(Units.metersToFeet(thirdShotMeters));
            if (m_Shooter.ShooterIsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }

            // when no more balls, go to next step
            if (m_Shooter.ShooterIsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
                m_zeroBallCounter++;
                if (m_zeroBallCounter > 7) {
                    m_zeroBallCounter = 0;
                    m_autoStep = 9;
                    m_auto.setupAuto1bp3();
                    m_Timer.reset();
                    m_Timer.start();
                }
            }

        } else if (m_autoStep == 9) {
            SmartDashboard.putNumber("auto time", Timer.getFPGATimestamp() - m_timestamp);
            if (m_Timer.advanceIfElapsed(3)) {
                m_autoStep = 10;
            }
            m_Shooter.shootingDist(Units.metersToFeet(thirdShotMeters));
            if (m_Shooter.ShooterIsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }
        } else if (m_autoStep == 10) {
            m_Shooter.warmUp();
            m_Intake.turnOffLoadShooter();
        } else {
            m_Shooter.warmUp();
            m_Intake.turnOffLoadShooter();

        }

        // always update intake state
        m_Intake.changeState(false, false);
    }

    public void Auto2() {
        aimTurretAuto();
        if (m_autoStep == 0) {
            if (m_auto.isTrajectoryCompleted()) {
                m_autoStep = 1;
                m_Timer.reset();
                m_Timer.start();
            }
            m_Intake.turnOn();
            m_SwerveDrive.driveFromChassisSpeeds(m_auto.getNextChassisSpeeds(m_SwerveDrive.getPose2d()));
        } else if (m_autoStep == 1) {
            if (m_Intake.getNumberOfBallsHolding() > 1 || m_Timer.advanceIfElapsed(1)) {
                m_autoStep = 2;
            }
            m_SwerveDrive.stopModules();
            m_Intake.turnOn();
        } else if (m_autoStep == 2) {
            if (m_VisionShooter.getYaw() <= 0.1) {
                m_autoStep = 3;
            }
        } else if (m_autoStep == 3) {
            m_Shooter.shootingRPM(2850, 2800);
            if (m_Shooter.ShooterIsOkToShoot()) {
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
            m_Shooter.shootingRPM(2850, 2800);
            if (m_Shooter.ShooterIsOkToShoot()) {
                m_Intake.loadShooter();
            }
            m_SwerveDrive.stopModules();
        } else if (m_autoStep == 5) {
            m_Shooter.warmUp();
            m_Intake.turnOffLoadShooter();
            m_SwerveDrive.stopModules();
        } else {
            m_Shooter.warmUp();
            m_Intake.turnOffLoadShooter();
            m_SwerveDrive.stopModules();
        }

        // always update intake state
        m_Intake.changeState(false, false);
    }

    public void Auto3() {
        m_Shooter.turnOff();
        m_SwerveDrive.driveFromChassisSpeeds(new ChassisSpeeds(), new Translation2d());
        m_Intake.stopMotors();
        m_Climber.control(false, false, false, false, false, false, 0, 0);
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
        m_autoAimEnabled = true;
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
                OI.getNxtClimb(), OI.getFinClimb(), OI.getClimbArmSpeedDown(), OI.getClimbArmSpeedUp());

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
