// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/** Add your docs here. */
public class SwerveModule implements Sendable {
        private static final double kWheelRadius = Units.inchesToMeters(2.0);
        private static final int kTalonFXEncoderResolution = 2048;
        private static final double kDriveMetersPerIntegratedTick = 2.0 * Math.PI * kWheelRadius * (1.0 / 6.54)
                        * (1.0 / (double) kTalonFXEncoderResolution);

        // TODO: find this.
        // 360 deg * turning rot per motor rot * motor rot per 2048 enc ticks
        private static final double kTurningRotPerMotorRot = 1 / 10.0;

        private static final double kModuleMaxAngularVelocity = 2 * SwerveDrive.kMaxAngularSpeed;
        private static final double kModuleMaxAngularAcceleration = 8 * Math.PI; // radians per second squared

        public final WPI_TalonFX m_driveMotor;
        public final WPI_TalonFX m_turningMotor;

        // private final encoder m_driveEncoder; // use the encode inside talonfx
        private final CANCoder m_turningEncoder;
        private double m_turningEncoderOffset = 0;

        // Gains are for example purposes only - must be determined for your own robot!
        private final PIDController m_drivePIDController = new PIDController(2.0, 0, 0); // 2

        // Gains are for example purposes only - must be determined for your own robot!
        private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
                        2.5,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                        kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

        // Gains are for example purposes only - must be determined for your own robot!
        private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.4, 2.2); // 0.4, 1.93
        private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.34, 0.32); // 0.1,0.25

        private final SimpleMotorFeedforward m_driveFeedforwardIntegrated = new SimpleMotorFeedforward(0, 0);

        private SwerveModuleState m_desiredState = new SwerveModuleState();

        /**
         * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
         * and turning encoder.
         *
         * @param driveMotorChannel              CAN ID for the drive motor.
         * @param turningMotorChannel            CAN ID for the turning motor.
         * @param turningEncoderChannel          CAN ID for the turning encoder.
         * @param turningEncoderOffset           Offset from zero point to
         *                                       drive-straight angle
         *                                       in radians.
         * @param turningIntegratedEncoderOffset Offset for integrated encoder for
         *                                       turning.
         */
        public SwerveModule(
                        int driveMotorChannel,
                        int turningMotorChannel,
                        int turningEncoderChannel,
                        double turningEncoderOffset,
                        double turningIntegratedEncoderOffset) {

                m_driveMotor = new WPI_TalonFX(driveMotorChannel, RobotMap.kCANivore_name);
                m_turningMotor = new WPI_TalonFX(turningMotorChannel, RobotMap.kCANivore_name);

                m_driveMotor.configFactoryDefault(100);
                m_turningMotor.configFactoryDefault(100);

                m_driveMotor.setNeutralMode(NeutralMode.Brake);
                m_turningMotor.setNeutralMode(NeutralMode.Brake);

                m_driveMotor.setInverted(false);
                m_turningMotor.setInverted(true);

                m_driveMotor.setSensorPhase(false);
                m_turningMotor.setSensorPhase(false);

                TalonFXConfiguration baseConfig = new TalonFXConfiguration();
                baseConfig.closedloopRamp = 0.00;
                baseConfig.neutralDeadband = 0.005;
                baseConfig.nominalOutputForward = 0.0;
                baseConfig.nominalOutputReverse = 0.0;
                baseConfig.openloopRamp = 0.00;
                baseConfig.peakOutputForward = 1.0;
                baseConfig.peakOutputReverse = -1.0;
                baseConfig.statorCurrLimit.enable = true;
                baseConfig.statorCurrLimit.currentLimit = 40;
                baseConfig.statorCurrLimit.triggerThresholdCurrent = 40;
                baseConfig.statorCurrLimit.triggerThresholdTime = 0.5;
                baseConfig.supplyCurrLimit.enable = true;
                baseConfig.supplyCurrLimit.currentLimit = 30;
                baseConfig.supplyCurrLimit.triggerThresholdCurrent = 30;
                baseConfig.supplyCurrLimit.triggerThresholdTime = 1;
                baseConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
                baseConfig.voltageCompSaturation = 11;
                baseConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

                TalonFXConfiguration turnConfig = baseConfig;
                turnConfig.remoteFilter0.remoteSensorDeviceID = turningEncoderChannel;
                turnConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
                turnConfig.slot0.allowableClosedloopError = 50;
                turnConfig.slot0.closedLoopPeakOutput = 1.0;
                turnConfig.slot0.closedLoopPeriod = 20;
                turnConfig.slot0.integralZone = 100;
                turnConfig.slot0.kP = 0.0;
                turnConfig.slot0.kI = 0.0;
                turnConfig.slot0.kD = 0.0;
                turnConfig.slot0.kF = 0.0;
                turnConfig.motionAcceleration = kModuleMaxAngularAcceleration;
                turnConfig.motionCruiseVelocity = kModuleMaxAngularVelocity;
                turnConfig.motionCurveStrength = 2;

                m_turningMotor.configAllSettings(turnConfig, 100);
                m_turningMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
                m_turningMotor.selectProfileSlot(0, 0);

                TalonFXConfiguration driveConfig = baseConfig;
                driveConfig.slot0.allowableClosedloopError = 0;
                driveConfig.slot0.closedLoopPeakOutput = 1;
                driveConfig.slot0.closedLoopPeriod = 10;
                driveConfig.slot0.integralZone = 100;
                driveConfig.slot0.kP = 0;
                driveConfig.slot0.kI = 0;
                driveConfig.slot0.kD = 0;
                driveConfig.slot0.kF = 0;

                m_driveMotor.configAllSettings(driveConfig, 100);
                m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_driveMotor.selectProfileSlot(0, 0);
                m_driveMotor.configSelectedFeedbackCoefficient(1);

                m_driveMotor.setSafetyEnabled(false);
                m_turningMotor.setSafetyEnabled(false);

                m_turningEncoder = new CANCoder(turningEncoderChannel, RobotMap.kCANivore_name);
                m_turningEncoder.configFactoryDefault(100);
                m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
                m_turningEncoder.configSensorInitializationStrategy(
                                SensorInitializationStrategy.BootToAbsolutePosition);
                m_turningEncoder.setPositionToAbsolute();
                m_turningEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 50);

                // Limit the PID Controller's input range between -pi and pi and set the input
                // to be continuous.
                m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

                m_turningEncoderOffset = turningEncoderOffset;

                m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
                m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
                m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
                m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
                m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
                m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
                m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);

                m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
                m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
                m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
                m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
                m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
                m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
                m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);
        }

        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        public SwerveModuleState getState() {
                return new SwerveModuleState(getDriveRatePerSecond(),
                                new Rotation2d(getTurningPositionRadians()));
        }

        private double getDriveRatePerSecond() {
                // return m_driveMotor.getSelectedSensorVelocity();
                return m_driveMotor.getSelectedSensorVelocity() * 10 * kDriveMetersPerIntegratedTick;
        }

        private double getTurningPositionRadians() {
                // return Units.degreesToRadians(m_turningEncoder.getAbsolutePosition()) -
                // m_turningEncoderOffset;
                double angle = m_turningEncoder.getPosition() - Units.radiansToDegrees(m_turningEncoderOffset);
                angle = Math.IEEEremainder(angle, 360);
                // if (angle > 180) {
                // angle -= (360);
                // } else if (angle < -180) {
                // angle += (360);
                // }
                return Units.degreesToRadians(angle);
        }

        /**
         * s
         * Sets the desired state for the module.
         *
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
                // Optimize the reference state to avoid spinning further than 90 degrees
                SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);

                m_desiredState = state;

                // if requested state is of ~0 speed, then don't move wheels back to zero,
                // just stop them
                if (Math.abs(state.speedMetersPerSecond) < 0.005) {
                        stop();
                        return;
                }

                // Calculate the drive output from the drive PID controller.
                final double driveOutput = m_drivePIDController.calculate(getDriveRatePerSecond(),
                                state.speedMetersPerSecond);

                final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

                // Calculate the turning motor output from the turning PID controller.
                final double turnOutput = m_turningPIDController
                                .calculate(getTurningPositionRadians(), state.angle.getRadians());

                final double turnFeedforward = m_turnFeedforward
                                .calculate(m_turningPIDController.getSetpoint().velocity);

                m_driveMotor.setVoltage(driveOutput + driveFeedforward);
                m_turningMotor.setVoltage(turnOutput + turnFeedforward);
        }

        /**
         * Sets the desired state for the module using onboard motion magic and Velocity
         * control.
         *
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredStateTalonOnboard(SwerveModuleState desiredState) {
                // Optimize the reference state to avoid spinning further than 90 degrees
                SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                                new Rotation2d(getTurningPositionRadians()));

                m_desiredState = state;

                // Calculate the drive output from the drive PID controller.
                // final double driveOutput =
                // m_drivePIDController.calculate(getDriveRatePerSecond(),
                // state.speedMetersPerSecond);

                final double driveFeedforward = m_driveFeedforwardIntegrated.calculate(state.speedMetersPerSecond);

                // Calculate the turning motor output from the turning PID controller.
                // final double turnOutput = m_turningPIDController
                // .calculate(getTurningPositionRadians(), state.angle.getRadians());

                final double turnFeedforward = m_turnFeedforward
                                .calculate(m_turningPIDController.getSetpoint().velocity);

                // m_driveMotor.setVoltage(driveOutput + driveFeedforward);
                // m_turningMotor.setVoltage(turnOutput + turnFeedforward);
                final double k100msPerSec = 10;
                m_driveMotor.set(ControlMode.Velocity,
                                state.speedMetersPerSecond / kDriveMetersPerIntegratedTick / k100msPerSec,
                                DemandType.ArbitraryFeedForward, driveFeedforward);
                m_turningMotor.set(ControlMode.MotionMagic, state.angle.getDegrees(),
                                DemandType.ArbitraryFeedForward, turnFeedforward);
        }

        public void stop() {
                m_turningMotor.set(ControlMode.PercentOutput, 0);
                m_driveMotor.set(ControlMode.PercentOutput, 0);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Swerve Module");
                builder.addDoubleProperty("Actual Drive m-s", () -> this.getDriveRatePerSecond(), null);
                builder.addDoubleProperty("Drive Position Meters",
                                () -> (kDriveMetersPerIntegratedTick *
                                                m_driveMotor.getSelectedSensorPosition()),
                                null);
                builder.addDoubleProperty("Actual Angle deg", () -> m_turningEncoder.getPosition(), null);
                builder.addDoubleProperty("Actual Angle with offset deg",
                                () -> Units.radiansToDegrees(getTurningPositionRadians()), null);
                builder.addDoubleProperty("Desired Drive m-s", () -> m_desiredState.speedMetersPerSecond, null);
                builder.addDoubleProperty("Desired Angle deg", () -> m_desiredState.angle.getDegrees(), null);
        }
}
