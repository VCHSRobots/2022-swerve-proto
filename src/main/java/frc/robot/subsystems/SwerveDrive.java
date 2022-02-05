// Open Source Software; you can modify and/or share it under the terms of
// Copyright (c) FIRST and other WPILib contributors.
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Constants.SwerveModuleOffsetRadians;

/** Add your docs here. */
public class SwerveDrive extends Base {
    public static final double kMaxSpeed = 2.0; // 3 meters per second
    public static final double kMaxAngularSpeed = 3 * Math.PI; // 1 rotation per second

    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(8);

    private final double inches15toMeters = Units.inchesToMeters(11);
    private final Translation2d m_frontLeftLocation = new Translation2d(inches15toMeters, inches15toMeters);
    private final Translation2d m_frontRightLocation = new Translation2d(inches15toMeters, -inches15toMeters);
    private final Translation2d m_backLeftLocation = new Translation2d(-inches15toMeters, inches15toMeters);
    private final Translation2d m_backRightLocation = new Translation2d(-inches15toMeters, -inches15toMeters);

    private final SwerveModule m_frontLeft = new SwerveModule(RobotMap.kDrive_FrontLeftDrive_TalonFX,
            RobotMap.kDrive_FrontLeftTurn_TalonFX, RobotMap.kDrive_FrontLeftEncoder,
            Constants.SwerveModuleOffsetRadians.FRONT_LEFT, 0);
    private final SwerveModule m_frontRight = new SwerveModule(RobotMap.kDrive_FrontRightDrive_TalonFX,
            RobotMap.kDrive_FrontRightTurn_TalonFX, RobotMap.kDrive_FrontRightEncoder,
            Constants.SwerveModuleOffsetRadians.FRONT_RIGHT, 0);
    private final SwerveModule m_backLeft = new SwerveModule(RobotMap.kDrive_BackLeftDrive_TalonFX,
            RobotMap.kDrive_BackLeftTurn_TalonFX, RobotMap.kDrive_BackLeftEncoder,
            Constants.SwerveModuleOffsetRadians.BACK_LEFT, 0);
    private final SwerveModule m_backRight = new SwerveModule(RobotMap.kDrive_BackRightDrive_TalonFX,
            RobotMap.kDrive_BackRightTurn_TalonFX, RobotMap.kDrive_BackRightEncoder,
            Constants.SwerveModuleOffsetRadians.BACK_RIGHT, 0);

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroRotation2d());

    private boolean m_fieldRelative = false;

    // tracking vars to output to dashboard
    private ChassisSpeeds m_lastChassisSpeedsDesired = new ChassisSpeeds();

    public SwerveDrive() {
        m_gyro.calibrate();
        m_gyro.reset();
    }

    // Robot Init
    public void robotInit() {
        // SendableRegistry.add(m_frontLeft, "frontleft");
        // SendableRegistry.add(m_frontRight, "frontright");
        Shuffleboard.getTab("main").add("front left", m_frontLeft);
        Shuffleboard.getTab("main").add("front right", m_frontRight);
        Shuffleboard.getTab("main").add("back left", m_backLeft);
        Shuffleboard.getTab("main").add("back right", m_backRight);
    }

    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(-m_gyro.getAngle());
    }

    public void setPose2d(PathPlannerState pose) {
        m_gyro.reset();
        m_gyro.setAngleAdjustment(pose.holonomicRotation.getDegrees());
        m_odometry.resetPosition(pose.poseMeters, getGyroRotation2d());
    }

    public Pose2d getPose2d() {
        return m_odometry.getPoseMeters();
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
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
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
        /* START OF TESTING ROTATION ONLY CODE */
        // for (SwerveModuleState state : swerveModuleStates) {
        // state.speedMetersPerSecond = 0;
        // }
        /* END OF TESTING ROTATION ONLY CODE */
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        // updateOdometry(); // MOVED TO ROBOTPERIODIC()
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
    public void driveWithXbox(double driveY, double driveX, double leftTriggerAxis, double rightTriggerAxis,
            double rightY, double rightX) {
        // set wheels to 1 Rot per sec, else drive normal

        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xSpeedLimiter
                .calculate(MathUtil.applyDeadband(driveY, Constants.xboxDeadband))
                * SwerveDrive.kMaxSpeed * 0.9;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_ySpeedLimiter
                .calculate(MathUtil.applyDeadband(driveX, Constants.xboxDeadband))
                * SwerveDrive.kMaxSpeed * 0.9;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        // final var rot =
        // -m_rotLimiter.calculate(MathUtil.applyDeadband(OI.getDriveRot(),
        // Constants.xboxDeadband))
        // * SwerveDrive.kMaxAngularSpeed;

        // final var centerOfRotationMeters = OI.getCenterOfRotationFrontLeft() ?
        // m_frontLeftLocation
        // : new Translation2d();
        final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(OI.getDriveRot(), Constants.xboxDeadband))
                * SwerveDrive.kMaxAngularSpeed;

        final var centerOfRotationMeters = OI.getCenterOfRotationFrontLeft() ? m_frontLeftLocation
                : (OI.getCenterOfRotationFrontRight() ? m_frontRightLocation : new Translation2d());

        drive(xSpeed, ySpeed, rot, m_fieldRelative, centerOfRotationMeters);
    }

    //Robot Init
    /*public void robotInit() {
     /*   // SendableRegistry.add(m_frontLeft, "frontleft");
        // SendableRegistry.add(m_frontRight, "frontright");
        Shuffleboard.getTab("main").add("front left", m_frontLeft);
        Shuffleboard.getTab("main").add("front right", m_frontRight);
        Shuffleboard.getTab("main").add("back left", m_backLeft);
        Shuffleboard.getTab("main").add("back right", m_backRight); 
    } */

    @Override
    public void disabledInit() {
    }

    // Robot Periodic
    public void changeOdometry(boolean setFieldRelative, boolean setRobotRelative, boolean resetOdometry) {
        if (setFieldRelative) {
            m_fieldRelative = true;
        } else if (setRobotRelative) {
            m_fieldRelative = false;
        }
        if (resetOdometry) {
            m_gyro.reset();
            m_odometry.resetPosition(new Pose2d(), getGyroRotation2d());
        }
        updateOdometry();
    }

    // autonomusInit
    public void resetOdometry() {
        m_odometry.resetPosition(new Pose2d(), getGyroRotation2d());
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
        builder.addDoubleProperty("Heading deg", () -> -m_gyro.getAngle(), null);
        builder.addDoubleProperty("Desired Vx m-s", () -> m_lastChassisSpeedsDesired.vxMetersPerSecond, null);
        builder.addDoubleProperty("Desired Vy m-s", () -> m_lastChassisSpeedsDesired.vyMetersPerSecond, null);
        builder.addDoubleProperty("Desired Rot rad-s", () -> m_lastChassisSpeedsDesired.omegaRadiansPerSecond, null);
    }
}
