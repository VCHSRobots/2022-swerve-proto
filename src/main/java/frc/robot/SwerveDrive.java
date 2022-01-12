// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Add your docs here. */
public class SwerveDrive extends Base {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);

    private final double inches15toMeters = Units.inchesToMeters(15);
    private final Translation2d m_frontLeftLocation = new Translation2d(inches15toMeters, inches15toMeters);
    private final Translation2d m_frontRightLocation = new Translation2d(inches15toMeters, -inches15toMeters);
    private final Translation2d m_backLeftLocation = new Translation2d(-inches15toMeters, inches15toMeters);
    private final Translation2d m_backRightLocation = new Translation2d(-inches15toMeters, -inches15toMeters);

    private final SwerveModule m_frontLeft = new SwerveModule(RobotMap.kDrive_FrontLeftDrive_TalonFX,
            RobotMap.kDrive_FrontLeftTurn_TalonFX, RobotMap.kDrive_FrontLeftEncoder, 0);
    private final SwerveModule m_frontRight = new SwerveModule(RobotMap.kDrive_FrontRightDrive_TalonFX,
            RobotMap.kDrive_FrontRightTurn_TalonFX, RobotMap.kDrive_FrontRightEncoder, 0);
    private final SwerveModule m_backLeft = new SwerveModule(RobotMap.kDrive_BackLeftDrive_TalonFX,
            RobotMap.kDrive_BackLeftTurn_TalonFX, RobotMap.kDrive_BackLeftEncoder, 0);
    private final SwerveModule m_backRight = new SwerveModule(RobotMap.kDrive_BackRightDrive_TalonFX,
            RobotMap.kDrive_BackRightTurn_TalonFX, RobotMap.kDrive_BackRightEncoder, 0);

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

    private boolean m_fieldRelative = false;

    // tracking vars to output to dashboard
    private ChassisSpeeds m_lastChassisSpeedsDesired = new ChassisSpeeds();

    public SwerveDrive() {
        m_gyro.reset();
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
        m_lastChassisSpeedsDesired = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        var swerveModuleStates = m_kinematics.toSwerveModuleStates(m_lastChassisSpeedsDesired);
        // fieldRelative
        // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
        // m_gyro.getRotation2d())
        // : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(
                m_gyro.getRotation2d(),
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_backLeft.getState(),
                m_backRight.getState());
    }

    private void driveWithXbox() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xSpeedLimiter.calculate(MathUtil.applyDeadband(OI.getDriveY(), 0.02))
                * SwerveDrive.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_ySpeedLimiter.calculate(MathUtil.applyDeadband(OI.getDriveX(), 0.02))
                * SwerveDrive.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(OI.getDriveRot(), 0.02))
                * SwerveDrive.kMaxAngularSpeed;

        drive(xSpeed, ySpeed, rot, m_fieldRelative);
    }

    @Override
    public void robotInit() {
        Shuffleboard.getTab("main").add(m_frontLeft);
        Shuffleboard.getTab("main").add(m_frontRight);
        Shuffleboard.getTab("main").add(m_backLeft);
        Shuffleboard.getTab("main").add(m_backRight);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void robotPeriodic() {
        if (OI.shouldSetFieldRelative()) {
            m_fieldRelative = true;
        } else if (OI.shouldSetRobotRelative()) {
            m_fieldRelative = false;
        }
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        driveWithXbox();
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Drive");
        builder.addBooleanProperty("Field Oriented", () -> m_fieldRelative, null);
        builder.addDoubleProperty("Heading deg", () -> m_gyro.getRotation2d().getDegrees(), null);
        builder.addDoubleProperty("Desired Vx m/s", () -> m_lastChassisSpeedsDesired.vxMetersPerSecond, null);
        builder.addDoubleProperty("Desired Vy m/s", () -> m_lastChassisSpeedsDesired.vyMetersPerSecond, null);
        builder.addDoubleProperty("Desired Rot rad/s", () -> m_lastChassisSpeedsDesired.omegaRadiansPerSecond, null);
    }
}
