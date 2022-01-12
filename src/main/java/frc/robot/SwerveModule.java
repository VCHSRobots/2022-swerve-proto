// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/** Add your docs here. */
public class SwerveModule implements Sendable {
    private static final double kWheelRadius = Units.inchesToMeters(4);// 0.0508;
    private static final int kEncoderResolution = 2048;
    private static final double kDriveDistancePerPulse = 2 * Math.PI * kWheelRadius * (1/5) * (1 / kEncoderResolution);

    private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    public final WPI_TalonFX m_driveMotor;
    public final WPI_TalonFX m_turningMotor;

    // private final encoder m_driveEncoder; // use the encode inside talonfx
    private final CANCoder m_turningEncoder;
    private double m_turningEncoderOffset = 0;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            1.75,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.3, 0.4);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.4, 0.5);

    private SwerveModuleState m_desiredState = new SwerveModuleState();

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel     CAN ID for the drive motor.
     * @param turningMotorChannel   CAN ID for the turning motor.
     * @param turningEncoderChannel CAN ID for the turning encoder.
     * @param turningEncoderOffset  Offset from zero point to drive-straight angle
     *                              in radians.
     */
    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderChannel,
            double turningEncoderOffset) {

        m_driveMotor = new WPI_TalonFX(driveMotorChannel);
        m_turningMotor = new WPI_TalonFX(turningMotorChannel);

        m_driveMotor.configFactoryDefault();
        m_turningMotor.configFactoryDefault();

        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_turningMotor.setNeutralMode(NeutralMode.Brake);

        m_driveMotor.setInverted(false);
        m_turningMotor.setInverted(true);

        m_driveMotor.setSensorPhase(false);
        m_turningMotor.setSensorPhase(false);

        TalonFXConfiguration baseConfig = new TalonFXConfiguration();
        baseConfig.closedloopRamp = 0.02;
        baseConfig.neutralDeadband = 0.005;
        baseConfig.nominalOutputForward = 0.0;
        baseConfig.nominalOutputReverse = 0.0;
        baseConfig.openloopRamp = 0.02;
        baseConfig.peakOutputForward = 1.0;
        baseConfig.peakOutputReverse = -1.0;
        baseConfig.statorCurrLimit.enable = true;
        baseConfig.statorCurrLimit.currentLimit = 60;
        baseConfig.statorCurrLimit.triggerThresholdCurrent = 60;
        baseConfig.statorCurrLimit.triggerThresholdTime = 1;
        baseConfig.supplyCurrLimit.enable = true;
        baseConfig.supplyCurrLimit.currentLimit = 60;
        baseConfig.supplyCurrLimit.triggerThresholdCurrent = 60;
        baseConfig.supplyCurrLimit.triggerThresholdTime = 1;
        baseConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_50Ms;
        baseConfig.voltageCompSaturation = 11;

        m_turningMotor.configAllSettings(baseConfig);

        TalonFXConfiguration driveConfig = baseConfig;
        driveConfig.slot0.allowableClosedloopError = 0;
        driveConfig.slot0.closedLoopPeakOutput = 1;
        driveConfig.slot0.closedLoopPeriod = 1;
        driveConfig.slot0.integralZone = 10;
        driveConfig.slot0.kP = 1;
        driveConfig.slot0.kI = 0;
        driveConfig.slot0.kD = 0;
        driveConfig.slot0.kF = 0;

        m_driveMotor.configAllSettings(driveConfig);

        // m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
        m_turningEncoder = new CANCoder(turningEncoderChannel);
        m_turningEncoder.configFactoryDefault();
        m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
        // kEncoderResolution);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_turningEncoderOffset = turningEncoderOffset;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Encoder driveEnc = new Encoder(4,5);
        // return new SwerveModuleState(driveEnc.getRate(), new
        // Rotation2d(m_turningEncoder.get()));
        return new SwerveModuleState(getDriveRatePerSecond(),
                Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));
    }

    private double getDriveRatePerSecond() {
        return m_driveMotor.getSelectedSensorVelocity() * 10 * kDriveDistancePerPulse;
    }

    // private double getTurningRatePerSecond() {
    // return m_turningEncoder.get
    // return m_turningMotor.getSelectedSensorVelocity() * 10 *
    // kTurningRadiansPerPulse;
    // }

    private double getTurningPositionRadians() {
        return Units.degreesToRadians(m_turningEncoder.getAbsolutePosition()) - m_turningEncoderOffset;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(getTurningPositionRadians()));

        m_desiredState = state;

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(getDriveRatePerSecond(), state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController
                .calculate(getTurningPositionRadians(), state.angle.getRadians());

        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");
        builder.addDoubleProperty("Actual Drive m/s", () -> getDriveRatePerSecond(), null);
        builder.addDoubleProperty("Actual Angle deg", () -> m_turningEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Actual Angle with offset rad", () -> getTurningPositionRadians(), null);
        builder.addDoubleProperty("Drive Position", () -> m_driveMotor.getSelectedSensorPosition(), null);
        builder.addDoubleProperty("Desired Drive m/s", () -> m_desiredState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Desired Angle deg", () -> m_desiredState.angle.getDegrees(), null);
    }
}
