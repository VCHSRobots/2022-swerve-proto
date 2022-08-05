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

    Timer m_Timer = new Timer();
    int m_autoStep = 0;

    private Auto m_auto = new Auto();

    double m_shootingDistance = 0;

    private final Compressor m_phCompressor = new Compressor(PneumaticsModuleType.REVPH);
    private final PneumaticHub m_ph = new PneumaticHub(); // even though not used, keep here

    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("super");
    ShuffleboardTab CompTab = Shuffleboard.getTab("computil");
    // shuffleboard: camera, is aimed, is ok to shoot, how many balls are in intake

    NetworkTableEntry ntBotRpm_rb = CompTab.add("RB Bot RPM", 2200).withPosition(3, 5).withSize(1, 1).getEntry();
    NetworkTableEntry ntTopRPM_rb = CompTab.add("RB Top RPM", 1400).withPosition(3, 4).withSize(1, 1).getEntry();

    NetworkTableEntry ntBotRpm_lb = CompTab.add("Y Bot RPM", 2200).withSize(1, 1).getEntry();
    NetworkTableEntry ntTopRPM_lb = CompTab.add("Y Top RPM", 1400).withSize(1, 1).getEntry();

    NetworkTableEntry ntFeetToRPM = CompTab.add("Feet To RPM", 17).withPosition(0, 0).withSize(1, 1)
            .getEntry();
    NetworkTableEntry ntLEDOn = CompTab.add("Limelight LED On", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();
    NetworkTableEntry ntShooterPreheatEnable = CompTab.add("Shooter Preheat", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();

    NetworkTableEntry ntKidMode = CompTab.add("Kid Mode", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    public SuperStructure(SwerveDrive swerveDrive, Intake intake, Shooter shooter, Climber climber) {
        m_SwerveDrive = swerveDrive;
        m_Intake = intake;
        m_Shooter = shooter;
        m_Climber = climber;
    }

    @Override
    public void robotInit() {
        m_phCompressor.disable();
        m_SwerveDrive.robotInit();
        m_Intake.init();
        m_Shooter.robotInit();
        m_Climber.robotInit();
        m_auto.robotInit();
        m_VisionShooter.robotInit();

        m_cameraThread = new Thread(
                () -> {
                    UsbCamera camera = CameraServer.startAutomaticCapture();
                    camera.setResolution(320, 240);
                });
        m_cameraThread.setDaemon(true);
        m_cameraThread.start();

        Shuffleboard.getTab("super").addBoolean("IsOkToShoot", () -> m_Shooter.IsOkToShoot()).withPosition(0, 4);

        Shuffleboard.getTab("super").addNumber("Camera Based Distance", () -> m_VisionShooter.getDistance())
                .withPosition(5, 0).withSize(2, 1);
        Shuffleboard.getTab("computil").add("swervedrive", m_SwerveDrive).withPosition(8, 1).withSize(2, 2);

        Shuffleboard.getTab("debug").addNumber("Camera to Target Yaw", () -> m_VisionShooter.getYaw());
        Shuffleboard.getTab("debug").addNumber("Turret Angle", () -> m_Shooter.getTurretAngleDegrees());
        Shuffleboard.getTab("debug").addNumber("Turret CANcoder", () -> m_Shooter.getTurretAngleCANcoder());
        Shuffleboard.getTab("debug").addNumber("autoStep", () -> m_autoStep);
        Shuffleboard.getTab("debug").addBoolean("last button climber", () -> m_lastButtonWasClimber);

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
        if (OI.getAimTurret() || OI.getRightBumperForLaunchShot() || OI.getTapeShot()
                || OI.getRightTriggerForShooting() || OI.getUnjam() || OI.getBarf() || OI.startIntake()
                || OI.getLeftTurntable() || OI.getRightTurntable()) {
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
        m_VisionShooter.LEDon();
        m_VisionShooter.setOffset(1);
        m_SwerveDrive.setFieldRelative();
        m_autoAimEnabled = true;

    }

    @Override
    public void teleopPeriodic() {
        m_phCompressor.enableAnalog(90, 112);
        m_VisionShooter.LEDon();

        // CLIMBER
        // Send inputs to climber control.
        m_Climber.control(OI.getSolenoidReverse(), OI.getSolenoidForward(), OI.getArmsUp(), OI.getArmsDown(),
                OI.getNxtClimb(), OI.getFinClimb(), OI.getClimbArmSpeedDown(), OI.getClimbArmSpeedUp());

        // DRIVING //

        // XBOX DRIVING CODE
        var driveY = OI.getDriveY();
        var driveX = OI.getDriveX();
        var driveR = OI.getDriveRot();
        if (ntKidMode.getBoolean(false)) {
            driveY *= 0.3;
            driveX *= 0.3;
            driveR *= 0.3;
        }
        m_SwerveDrive.driveWithXbox(driveY, driveX, driveR,
                false, false);

        // INTAKE STATE UPDATE
        m_Intake.changeState(OI.startIntake(), OI.stopIntake(), OI.reverseIntake());

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
            if (m_Shooter.IsOkToShoot() && m_VisionShooter.isOnTarget()) {
                // Load shooter
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }
        } else if (OI.getRightBumperForLaunchShot()) {
            // turn shooter on in rpm mode
            m_Shooter.shootingRPM(ntTopRPM_rb.getDouble(0), ntBotRpm_rb.getDouble(0));

            if (m_Shooter.IsOkToShoot()) {
                // Load shooter
                m_Intake.loadShooter();

            } else {
                m_Intake.countinueIntakeMotors();
            }
        } else if (OI.getTapeShot()) {
            m_Shooter.shootingRPM(ntTopRPM_lb.getDouble(0), ntBotRpm_lb.getDouble(0));

            if (m_Shooter.IsOkToShoot()) {
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }

        }

        else if (m_lastButtonWasClimber) {
            m_Shooter.turnOff();
        } else if ((ntShooterPreheatEnable.getBoolean(false)
                || DriverStation.isFMSAttached()) && !m_lastButtonWasClimber) {
            // speed up shooter automatically
            m_Shooter.shootingDist(9.5);
        } else if (OI.getAimTurret()) {
            m_Shooter.shootingDist(9.5);
        } else if (m_Intake.getBothBallsLoaded() && !m_lastButtonWasClimber) {
            // speed up shooter automatically
            m_Shooter.shootingDist(9.5);
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

        // when shooting released, stop loading
        if (OI.getShootingReleased()) {
            m_Intake.turnOffLoadShooter();
        }

        if (OI.getDisableAutoAim()) {
            m_autoAimEnabled = false;
        }

        if (OI.shouldSetFieldRelative() || OI.getResetOdometryLaunchPad() || OI.getResetOdometryRightFender()) {
            m_autoAimEnabled = true;
        }

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
        } else if (m_autoAimEnabled) {
            aimTurretAuto();
        } else {
            m_Shooter.TurnTable(OI.getRightTurntable(),
                    OI.getLeftTurntable());
        }
    }

    public void aimTurretAuto() {
        if (m_VisionShooter.getTargetValid()) {
            m_Shooter.aimTurret(m_VisionShooter.getYaw());
        } else {
            // m_Shooter.setTurretAngle(m_state.getTurretAimingAngle().getDegrees());
            // TODO: try this below
            m_Shooter.setTurretAngle(m_state.getTurretAimingAngle(m_SwerveDrive.getAngVel()).getDegrees());
        }
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
        double firstShotMeters = 2.88;
        double secondShotMeters = 3.4;
        double thirdShotMeters = 3.15;

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
            if (m_Shooter.IsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
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
            if (m_Shooter.IsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }
            // when no more balls, go to next step
            if (m_Shooter.IsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
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
            if (m_Shooter.IsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
                m_Intake.loadShooter();
            } else {
                m_Intake.countinueIntakeMotors();
            }

            // when no more balls, go to next step
            if (m_Shooter.IsOkToShoot() && m_Intake.getNumberOfBallsHolding() == 0) {
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
            if (m_Shooter.IsOkToShoot() && Math.abs(m_VisionShooter.getYaw()) < 3) {
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
        m_Intake.changeState(false, false, false);
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
            m_Shooter.shootingRPM(2850, 2800);
            if (m_Shooter.IsOkToShoot()) {
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
        m_Intake.changeState(false, false, false);
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
