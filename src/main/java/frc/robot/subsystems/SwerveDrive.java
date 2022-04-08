// Open Source Software; you can modify and/or share it under the terms of
// Copyright (c) FIRST and other WPILib contributors.
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.RobotMap;

/** Add your docs here. */
public class SwerveDrive extends Base {
    public static final double kMaxSpeed = 4.3; // 3 meters per second
    public static final double kMaxAngularSpeed = 3 * Math.PI; // 1 rotation per second

    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(7);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(7);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(7);
    
    static ProfiledPIDController thetaController = new ProfiledPIDController(2.0, 0, 0,
            new Constraints(SwerveDrive.kMaxAngularSpeed, 12 * SwerveDrive.kMaxAngularSpeed));

    private final double inches15toMeters = Units.inchesToMeters(10);
    private final Translation2d m_frontLeftLocation = new Translation2d(inches15toMeters, inches15toMeters);
    private final Translation2d m_frontRightLocation = new Translation2d(inches15toMeters, -inches15toMeters);
    private final Translation2d m_backLeftLocation = new Translation2d(-inches15toMeters, inches15toMeters);
    private final Translation2d m_backRightLocation = new Translation2d(-inches15toMeters, -inches15toMeters);    
    // private final Translation2d m_backRightLocation = new Translation2d(inches15toMeters, inches15toMeters);
    // private final Translation2d m_backLeftLocation = new Translation2d(inches15toMeters, -inches15toMeters);
    // private final Translation2d m_frontRightLocation = new Translation2d(-inches15toMeters, inches15toMeters);
    // private final Translation2d m_frontLeftLocation = new Translation2d(-inches15toMeters, -inches15toMeters);

    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0));

    private boolean m_fieldRelative = true;
    private boolean m_prevResetOdometry = false;

    // tracking vars to output to dashboard
    private ChassisSpeeds m_lastChassisSpeedsDesired = new ChassisSpeeds();

    public SwerveDrive() {
        Thread zeroGyroThread = new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {

            }
        });
        zeroGyroThread.setDaemon(true);
        zeroGyroThread.start();

        double frontLeftOffset = 0;
        double frontRightoffset = 0;
        double backLeftoffset = 0;
        double backRightoffset = 0;
        if (Constants.isPracticeBot) {
            // frontLeftOffset = Constants.SwerveModuleOffsetRadiansPractice.FRONT_LEFT;
            // frontRightoffset = Constants.SwerveModuleOffsetRadiansPractice.FRONT_RIGHT;
            // backLeftoffset = Constants.SwerveModuleOffsetRadiansPractice.BACK_LEFT;
            // backRightoffset = Constants.SwerveModuleOffsetRadiansPractice.BACK_RIGHT;
            frontLeftOffset  = Constants.SwerveModuleOffsetRadiansPractice.kFRONT_LEFT;
            frontRightoffset = Constants.SwerveModuleOffsetRadiansPractice.kFRONT_RIGHT;
            backLeftoffset   = Constants.SwerveModuleOffsetRadiansPractice.kBACK_LEFT;
            backRightoffset  = Constants.SwerveModuleOffsetRadiansPractice.kBACK_RIGHT;
        } else {
            frontLeftOffset = Constants.SwerveModuleOffsetRadiansComp.FRONT_LEFT;
            frontRightoffset = Constants.SwerveModuleOffsetRadiansComp.FRONT_RIGHT;
            backLeftoffset = Constants.SwerveModuleOffsetRadiansComp.BACK_LEFT;
            backRightoffset = Constants.SwerveModuleOffsetRadiansComp.BACK_RIGHT;
        }

        m_frontLeft = new SwerveModule(RobotMap.kDrive_FrontLeftDrive_TalonFX,
            RobotMap.kDrive_FrontLeftTurn_TalonFX, RobotMap.kDrive_FrontLeftEncoder,
            frontLeftOffset, 0);
        m_frontRight = new SwerveModule(RobotMap.kDrive_FrontRightDrive_TalonFX,
            RobotMap.kDrive_FrontRightTurn_TalonFX, RobotMap.kDrive_FrontRightEncoder,
            frontRightoffset, 0);
        m_backLeft = new SwerveModule(RobotMap.kDrive_BackLeftDrive_TalonFX,
            RobotMap.kDrive_BackLeftTurn_TalonFX, RobotMap.kDrive_BackLeftEncoder,
            backLeftoffset, 0);
        m_backRight = new SwerveModule(RobotMap.kDrive_BackRightDrive_TalonFX,
            RobotMap.kDrive_BackRightTurn_TalonFX, RobotMap.kDrive_BackRightEncoder,
            backRightoffset, 0);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    // Robot Init
    public void robotInit() {
        Shuffleboard.getTab("debug").add("front left", m_frontLeft);
        Shuffleboard.getTab("debug").add("front right", m_frontRight);
        Shuffleboard.getTab("debug").add("back left", m_backLeft);
        Shuffleboard.getTab("debug").add("back right", m_backRight);

        Shuffleboard.getTab("debug").addNumber("getAngle", ()->m_gyro.getAngle());
        Shuffleboard.getTab("debug").addNumber("getYaw", ()->m_gyro.getYaw());

    }

    @Override
    public void disabledInit() {
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public Rotation2d getGyroRotation2d() {
        double angle = Math.IEEEremainder(-m_gyro.getAngle(), 360);
        return Rotation2d.fromDegrees(angle);
    }

    // public double getAngularVelocity() {
    //     return m_gyro.
    // }

    public void resetOdometry(Pose2d pose) {
        if (pose == null) {
            System.out.println("cannot reset, pose is " + pose);
            m_odometry.resetPosition(getPose2d(), getGyroRotation2d());
        } else {
            // m_gyro.setAngleAdjustment(pose.getRotation().getDegrees());
            m_gyro.setAngleAdjustment(-pose.getRotation().getDegrees()-m_gyro.getYaw());
            m_odometry.resetPosition(pose, getGyroRotation2d());
        }
    }

    public Pose2d getPose2d() {
        return m_odometry.getPoseMeters();
    }

    public double getAngVel() {
        return Units.degreesToRadians(m_gyro.getRate());
    }

    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    public void driveAndTurnToAngle(double xSpeed, double ySpeed, double angle) {
        double rot = thetaController.calculate(getPose2d().getRotation().getRadians(), angle);
        drive(xSpeed, ySpeed, rot, true);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        drive(xSpeed, ySpeed, rot, fieldRelative, new Translation2d());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed                 Speed of the robot in the x direction
     *                               (forward).
     * @param ySpeed                 Speed of the robot in the y direction
     *                               (sideways).
     * @param rot                    Angular rate of the robot.
     * @param fieldRelative          Whether the provided x and y speeds are
     *                               relative to the
     *                               field.
     * @param centerOfRotationMeters Point relative to center of robot around which
     *                               to rotate. defaults to 0,0
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            Translation2d centerOfRotationMeters) {
        m_lastChassisSpeedsDesired = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        driveFromChassisSpeeds(m_lastChassisSpeedsDesired, centerOfRotationMeters);
    }

    public void driveFromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        driveFromChassisSpeeds(chassisSpeeds, new Translation2d(0, 0));
    }

    public void driveFromChassisSpeeds(ChassisSpeeds chassisSpeed, Translation2d centerOfRotationMeters) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeed, centerOfRotationMeters);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(
                getGyroRotation2d(),
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_backLeft.getState(),
                m_backRight.getState());
    }

    // Teleop Periodic
    public void driveWithXbox(double driveY, double driveX, double xboxRot, boolean frontLeftCOR,
            boolean frontRightCOR) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xSpeedLimiter
                .calculate(MathUtil.applyDeadband(driveY, Constants.xboxDeadband))
                * SwerveDrive.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_ySpeedLimiter
                .calculate(MathUtil.applyDeadband(driveX, Constants.xboxDeadband))
                * SwerveDrive.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(xboxRot, 0.05))
                * SwerveDrive.kMaxAngularSpeed;

        final var centerOfRotationMeters = frontLeftCOR ? m_frontLeftLocation
                : (frontRightCOR ? m_frontRightLocation : new Translation2d());

        drive(xSpeed, ySpeed, rot, m_fieldRelative, centerOfRotationMeters);
    }

    public void setFieldRelative() {
        m_fieldRelative = true;
    }

    // Robot Periodic
    public void changeOdometry(boolean setFieldRelative, boolean setRobotRelative, boolean resetOdometryLaunchPad, boolean resetOdometryFender) {
        if (setFieldRelative) {
            m_fieldRelative = true;
            resetOdometry(new Pose2d(getPose2d().getTranslation(), new Rotation2d()));
        } else if (setRobotRelative) {
            m_fieldRelative = false;
        }
        if (resetOdometryLaunchPad && !m_prevResetOdometry) {
            Translation2d launchPad = new Translation2d(3.86, 5.46);
            resetOdometry(new Pose2d(launchPad, new Rotation2d()));
        }
        if (resetOdometryFender && !m_prevResetOdometry) {
            Translation2d rightFender =  new Translation2d(7.82, 2.95);
            resetOdometry(new Pose2d(rightFender, Rotation2d.fromDegrees(-111)));
        }
        m_prevResetOdometry = resetOdometryLaunchPad;

        updateOdometry();
    }

    // Test Periodic
    public void test(boolean getAButton) {

        // ADD CODE TO TEST MOTORS HERE
        if (getAButton) {
            m_frontLeft.m_turningMotor.set(ControlMode.PercentOutput, 0.03);
            m_frontRight.m_turningMotor.set(ControlMode.PercentOutput, 0.03);
            m_backLeft.m_turningMotor.set(ControlMode.PercentOutput, 0.03);
            m_backRight.m_turningMotor.set(ControlMode.PercentOutput, 0.03);
        } else {
            m_backRight.m_turningMotor.set(ControlMode.PercentOutput, 0);
            m_backLeft.m_turningMotor.set(ControlMode.PercentOutput, 0);
            m_frontRight.m_turningMotor.set(ControlMode.PercentOutput, 0);
            m_frontLeft.m_turningMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Drive");
        builder.addBooleanProperty("Field Oriented", () -> m_fieldRelative, null);
        builder.addDoubleProperty("Rotation deg", () -> getGyroRotation2d().getDegrees(), null);
        builder.addDoubleProperty("Desired Vx m-s", () -> m_lastChassisSpeedsDesired.vxMetersPerSecond, null);
        builder.addDoubleProperty("Desired Vy m-s", () -> m_lastChassisSpeedsDesired.vyMetersPerSecond, null);
        builder.addDoubleProperty("Desired Rot rad-s", () -> m_lastChassisSpeedsDesired.omegaRadiansPerSecond, null);
    }
}
