package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.distanceRPMPoint;

public class Shooter extends Base {
    // turntable gear ratio
    // 13 to 62, 56 to 231, GEAR RATIO: 21.19
    // [motor rot / turret rot]*[enc ticks / motor rot]*[turret rot / 360 degrees]
    // private final double kEncoderTicksPerDegree = 21.19 * (2048.0) * (1.0 /
    // 360.0);
    private final double kEncoderTicksPerDegree = (231.0 / 56.0) * (62.0 / 13.0) * 2048 * (1.0 / 360.0);

    private final double kZeroOffsetEncoderTicks = 0;
    private final double kZeroOffsetDegrees = -165;
    private final double kAngleOffset = 180;
    private final double kMinAngle = -135;
    private final double kMaxAngle = 275;
    private final double kMaxAngularVelocity = 600.0; // keep within 560, started at 135
    private final double kMaxAngularAcceleration = 12000.0; // keep within 5700, started at 455
    private double m_targetAngle = 0;
    private boolean m_isTurningAround = false;
    // Shuffleboard Tabs and NetworkTableEntries.
    ShuffleboardTab debugTab = Shuffleboard.getTab("debug");
    // NetworkTableEntry ntVoltage = Shuffleboard.getTab("Shooter").add("voltage", 0.0).getEntry();

    WPI_TalonFX m_shootTalonTop = new WPI_TalonFX(RobotMap.kShoot_TopMotor_TalonFX, RobotMap.kCANivore_name);
    WPI_TalonFX m_shootTalonBot = new WPI_TalonFX(RobotMap.kShoot_BottomMotor_TalonFX, RobotMap.kCANivore_name);
    WPI_TalonFX m_turnTableTalon = new WPI_TalonFX(RobotMap.kTurnTableMotor_TalonFX, RobotMap.kCANivore_name);

    CANCoder m_turntableEncoder = new CANCoder(RobotMap.kShooter_TurretEncoder, RobotMap.kCANivore_name);

    private int m_isOKtoShootCounter = 0;

    private ProfiledPIDController m_turretPIDController = new ProfiledPIDController(0.08, 0, 0,
            new Constraints(kMaxAngularVelocity, kMaxAngularAcceleration));
    // private SimpleMotorFeedforward m_turretFeedForward = new SimpleMotorFeedforward(0.29, 0.007); // 0.007

    // SimpleMotorFeedforward m_ShootFeedForward = new SimpleMotorFeedforward(0.00,
    // 0.00045);

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_interpolatingSpeeds_top = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_interpolatingSpeeds_bot = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

    public Shooter() {
        distanceRPMPoint[] distanceRPMlist = {
                new distanceRPMPoint(8, 2480, 2340),
                new distanceRPMPoint(8.5, 2550, 2360),
                new distanceRPMPoint(9, 2650, 2400),
                new distanceRPMPoint(10, 2770, 2400), 
                new distanceRPMPoint(11, 2850, 2500),
                new distanceRPMPoint(11.5, 2880, 2580),
                new distanceRPMPoint(12, 2920, 2640),
                new distanceRPMPoint(12.5, 2950, 2670),
                new distanceRPMPoint(13, 3050, 2750),
                new distanceRPMPoint(14, 3230, 2920),
                new distanceRPMPoint(17, 3800, 2960)
        };

        for (distanceRPMPoint point : distanceRPMlist) {
            m_interpolatingSpeeds_bot.put(new InterpolatingDouble(point.distance),
                    new InterpolatingDouble(point.botRPM));
            m_interpolatingSpeeds_top.put(new InterpolatingDouble(point.distance),
                    new InterpolatingDouble(point.topRPM));
        }
    }

    // SHUFFLEBOARD HELPERS
    public double getTopMotorRPM() {
        return ticksPer100msToRPM(m_shootTalonTop.getSelectedSensorVelocity());
    }

    public double getBotMotorRPM() {
        return ticksPer100msToRPM(m_shootTalonBot.getSelectedSensorVelocity());
    }

    // END SHUFFLEBOARD HELPERS

    public void robotInit() {
        debugTab.addNumber("Actual Top RPM", () -> getTopMotorRPM()).withPosition(4, 1).withSize(4, 3);
                // .withWidget(BuiltInWidgets.kGraph);
        debugTab.addNumber("Actual Bot RPM", () -> getBotMotorRPM()).withPosition(4, 4).withSize(4, 3);
                // .withWidget(BuiltInWidgets.kGraph);
        debugTab.addNumber("Top Setpoint", () -> ticksPer100msToRPM(m_shootTalonTop.getClosedLoopTarget()));
                // .withWidget(BuiltInWidgets.kGraph);
        debugTab.addNumber("Bot Setpoint", () -> ticksPer100msToRPM(m_shootTalonBot.getClosedLoopTarget()));
                // .withWidget(BuiltInWidgets.kGraph);

        TalonFXConfiguration baseConfig = new TalonFXConfiguration();
        baseConfig.closedloopRamp = 0.0;
        baseConfig.neutralDeadband = 0.005;
        baseConfig.nominalOutputForward = 0.0;
        baseConfig.nominalOutputReverse = 0.0;
        baseConfig.openloopRamp = 0.01;
        baseConfig.peakOutputForward = 1;
        baseConfig.peakOutputReverse = -1;
        baseConfig.statorCurrLimit.enable = true;
        baseConfig.statorCurrLimit.currentLimit = 30;
        baseConfig.statorCurrLimit.triggerThresholdCurrent = 30;
        baseConfig.statorCurrLimit.triggerThresholdTime = 1;
        baseConfig.supplyCurrLimit.enable = true;
        baseConfig.supplyCurrLimit.currentLimit = 30;
        baseConfig.supplyCurrLimit.triggerThresholdCurrent = 30;
        baseConfig.supplyCurrLimit.triggerThresholdTime = 1;
        baseConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
        baseConfig.voltageCompSaturation = 10;
        baseConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        baseConfig.slot0.allowableClosedloopError = 20;
        baseConfig.slot0.closedLoopPeakOutput = 1.0;
        baseConfig.slot0.closedLoopPeriod = 20;
        baseConfig.slot0.integralZone = 100;

        m_shootTalonBot.configFactoryDefault(100);
        m_shootTalonTop.configFactoryDefault(100);
        m_turnTableTalon.configFactoryDefault(100);

        // TalonFXConfiguration topConfig = baseConfig;
        // top settings
        baseConfig.slot0.kI = 0.0;
        baseConfig.slot0.kD = 0.0;
        baseConfig.slot0.kF = 0.0505;
        baseConfig.slot0.kP = 0.0205; // 0.03
        m_shootTalonTop.configAllSettings(baseConfig, 100);

        // TalonFXConfiguration botConfig = baseConfig;
        // bot settings
        baseConfig.slot0.kI = 0.0;
        baseConfig.slot0.kD = 0.0;
        baseConfig.slot0.kF = 0.0535; // after distance tuning, was 0.051
        baseConfig.slot0.kP = 0.205; // after distance tuning, was 0.03
        m_shootTalonBot.configAllSettings(baseConfig, 100);

        m_shootTalonBot.setNeutralMode(NeutralMode.Coast);
        m_shootTalonTop.setNeutralMode(NeutralMode.Coast);
        m_turnTableTalon.setNeutralMode(NeutralMode.Brake);

        m_shootTalonBot.setInverted(false);
        m_shootTalonTop.setInverted(false);
        m_turnTableTalon.setInverted(false);

        m_shootTalonBot.setSensorPhase(false);
        m_shootTalonTop.setSensorPhase(false);

        TalonFXConfiguration turnTableConfig = baseConfig;
        turnTableConfig.peakOutputForward = .33;
        turnTableConfig.peakOutputReverse = -.33;
        // 0.1 @ 10 degree error
        // (0.1 * 1023) / (deg error * ticks / degree)
        turnTableConfig.slot0.kP = (1 * 1023) / (230 * kEncoderTicksPerDegree);
        turnTableConfig.slot0.kI = 0;
        turnTableConfig.slot0.kD = 0;
        turnTableConfig.slot0.kF = 0;
        turnTableConfig.slot0.allowableClosedloopError = 0.2 * kEncoderTicksPerDegree;
        turnTableConfig.supplyCurrLimit.currentLimit = 10;
        turnTableConfig.supplyCurrLimit.enable = true;
        turnTableConfig.supplyCurrLimit.triggerThresholdCurrent = 10;
        turnTableConfig.supplyCurrLimit.triggerThresholdTime = 0.5;
        turnTableConfig.forwardSoftLimitEnable = true;
        turnTableConfig.forwardSoftLimitThreshold = kMaxAngle * kEncoderTicksPerDegree;
        turnTableConfig.reverseSoftLimitEnable = true;
        turnTableConfig.reverseSoftLimitThreshold = kMinAngle * kEncoderTicksPerDegree;
        // // [deg / s*s] * [ticks / deg] * [s / 100ms] = [tick / (100ms * s)]
        // ticks per 100ms per sec
        turnTableConfig.motionAcceleration = kMaxAngularAcceleration * kEncoderTicksPerDegree * (1.0 / 10.0);
        // // [deg / s] * [tick / deg] * [s / 100ms] = [tick / 100ms]
        turnTableConfig.motionCruiseVelocity = kMaxAngularVelocity * kEncoderTicksPerDegree * (1.0 / 10.0);
        turnTableConfig.motionCurveStrength = 5;

        m_turnTableTalon.configAllSettings(turnTableConfig, 100);

        m_turnTableTalon.configOpenloopRamp(0.0);

        // initialize Turn Table CanCoder
        // set units of the CANCoder to radians, with velocity being radians per second

        var config = new CANCoderConfiguration();
        // config.sensorCoefficient = 2 * Math.PI / 4096.0;
        // config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        m_turntableEncoder.configFactoryDefault(100);
        m_turntableEncoder.configAllSettings(config);

        m_turntableEncoder.setPositionToAbsolute(50);
        m_turntableEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 50);

        m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 40);
        m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
        m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);

        m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 40);
        m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
        m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);

        m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
        m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
        m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);

        m_turretPIDController.disableContinuousInput();
        m_turretPIDController.setTolerance(0.5);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                if ((m_turntableEncoder.getPosition() - kZeroOffsetDegrees) < kMinAngle) {
                    m_turntableEncoder.setPosition(m_turntableEncoder.getPosition() + 360, 50);
                } else if ((m_turntableEncoder.getPosition() - kZeroOffsetDegrees) > kMaxAngle) {
                    m_turntableEncoder.setPosition(m_turntableEncoder.getPosition() - 360, 50);
                }
                Thread.sleep(100);
                m_turnTableTalon.setSelectedSensorPosition(getTurretAngleDegrees() * kEncoderTicksPerDegree);
            } catch (Exception e) {

            }
        }).start();

    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        m_isOKtoShootCounter = 0;
    }

    public void warmUp() {
        shootingRPM(2400, 2400);
    }

    // Shooting function with Distance. (NOT READY!!)
    public void shootingDist(double distanceFeet) {
        setSpeedsDist(distanceFeet);
    }

    // Shooting function with RPM.
    public void shootingRPM(double topRPM, double botRPM) {
        setSpeedsRPM(topRPM, botRPM);
    }

    public void turnOff() {
        m_shootTalonBot.setVoltage(0);
        m_shootTalonTop.setVoltage(0);
    }

    // Sets speed for RPM.
    public void setSpeedsRPM(double topRPM, double botRPM) {
        setShootSpeeds(rpmToTicksPer100ms(topRPM),
                rpmToTicksPer100ms(botRPM));
    }

    // Sets Speeds for Distance.
    public void setSpeedsDist(double distanceFeet) {
        setShootSpeeds(rpmToTicksPer100ms(topFeetToRPM(distanceFeet)),
                rpmToTicksPer100ms(botFeetToRPM(distanceFeet)));
    }

    // Sets doubles to Talons.
    public void setShootSpeeds(double shootTopSpeed, double shootBotSpeed) {
        m_shootTalonTop.set(ControlMode.Velocity, shootTopSpeed);
        m_shootTalonBot.set(ControlMode.Velocity, shootBotSpeed);
    }

    // Converts RPM to Ticks/100MS.
    public double rpmToTicksPer100ms(double rpm) {
        double minutesPerSecond = 1.0 / 60.0;
        double secondsPer100ms = 1.0 / 10.0;
        double ticksPerRotation = 2048;
        double ticksPer100ms = rpm * minutesPerSecond * secondsPer100ms * ticksPerRotation;
        return ticksPer100ms;
    }

    // Converts Ticks/100MS to RPM.
    public double ticksPer100msToRPM(double ticksPer100ms) {
        double secondsPerMinute = 60.0;
        double oneHundredMSPerSecond = 10.0;
        double rotationsPerTick = 1.0 / 2048;
        double RPM = ticksPer100ms * secondsPerMinute * oneHundredMSPerSecond * rotationsPerTick;
        return RPM;
    }

    // Equation for Top Motor.
    public double topFeetToRPM(double topfeet) {
        // double RPMquadtop = -1419.522 + 396.7329 * topfeet + -3.353022 * (topfeet *
        // topfeet);
        return m_interpolatingSpeeds_top.getInterpolated(new InterpolatingDouble(topfeet)).value;

    }

    // Equation for Bot Motor.
    public double botFeetToRPM(double botfeet) {
        return m_interpolatingSpeeds_bot.getInterpolated(new InterpolatingDouble(botfeet)).value;
    }

    // Boolean that checks if shooter is reading to shoot at a good speed.
    public boolean IsOkToShoot() {
        if (m_shootTalonBot.getControlMode() == ControlMode.PercentOutput) {
            return false;
        }
        double errorTopRPM = ticksPer100msToRPM(m_shootTalonTop.getClosedLoopTarget()
                - m_shootTalonTop.getSelectedSensorVelocity());
        double errorBotRPM = ticksPer100msToRPM(m_shootTalonBot.getClosedLoopTarget()
                - m_shootTalonBot.getSelectedSensorVelocity());
        boolean isBotFast = ticksPer100msToRPM(m_shootTalonBot.getSelectedSensorVelocity()) > 1300;

        if (Math.abs(errorBotRPM) < 80 && Math.abs(errorTopRPM) < 80 && isBotFast) {
            m_isOKtoShootCounter++;
        } else {
            m_isOKtoShootCounter = 0;
        }
        return m_isOKtoShootCounter > 3;
    }

    public boolean isSpinningFastEnoughForBarf() {
        boolean isBotFast = ticksPer100msToRPM(m_shootTalonBot.getSelectedSensorVelocity()) > 1000;
        boolean isTopFast = ticksPer100msToRPM(m_shootTalonTop.getSelectedSensorVelocity()) > 1000;
        return isBotFast && isTopFast;
    }

    public void setBarfVoltage() {
        m_shootTalonBot.setVoltage(2.75);
        m_shootTalonTop.setVoltage(2.75);

    }

    public void setVoltage(double voltagetop, double voltagebot) {
        m_shootTalonBot.setVoltage(voltagebot);
        m_shootTalonTop.setVoltage(voltagetop);
    }

    public void turnMotorsOff() {
        setShootSpeeds(0, 0);
    }

    // TurnTable Funtions.
    public void TurnTable(boolean rightSideTurnTable,
            boolean leftSideTurnTable) {
        double turntableSpeed = 0;
        if (rightSideTurnTable) {
            turntableSpeed = 0.07;
        }
        if (leftSideTurnTable) {
            turntableSpeed = -0.07;
        }
        m_turnTableTalon.set(ControlMode.PercentOutput, turntableSpeed);
    }

    public void configZeroSettings() {
        m_turnTableTalon.setSelectedSensorPosition(0);
        m_turnTableTalon.configReverseSoftLimitThreshold(-16100, 50);
        m_turnTableTalon.configForwardSoftLimitThreshold(8100, 50);
        m_turnTableTalon.configForwardSoftLimitEnable(true, 50);
        m_turnTableTalon.configReverseSoftLimitEnable(true, 50);
    }

    public void aimTurret(double angleYawDegreesOffset) {
        if (Math.abs(angleYawDegreesOffset) < 0.5) {
            m_turnTableTalon.setVoltage(0);
            return;
        }

        double targetAngle = getTurretAngleDegrees() + angleYawDegreesOffset;
        // if (m_isTurningAround) {
        // if (Math.abs(targetAngle - m_targetAngle) < 17) {
        // m_isTurningAround = false;
        // } else {
        // targetAngle = m_targetAngle;
        // }
        // }
        setTurretAngle(targetAngle);
    }

    public void setTurretAngle(double angleTargetDegrees) {
        double newAngle = angleTargetDegrees % 360;
        if (newAngle < kMinAngle) {
            newAngle += 360;
            // m_targetAngle = newAngle;
            // m_isTurningAround = true;
        } else if (newAngle > kMaxAngle) {
            newAngle -= 360;
            // m_targetAngle = newAngle;
            // m_isTurningAround = true;
        }
        double targetAngleTicks = angleDegreesToEncoderTicks(newAngle);
        // kS to overcome friction
        double kS = Math.copySign(0.020, targetAngleTicks -
                m_turnTableTalon.getSelectedSensorPosition());
        m_turnTableTalon.set(ControlMode.MotionMagic, targetAngleTicks,
                DemandType.ArbitraryFeedForward, kS);

        // Calculate the turning motor output from the turning PID controller.
        // double newAngle = angleTargetDegrees % 360;
        // if (newAngle < kMinAngle) {
        // newAngle += 360;
        // } else if (newAngle > kMaxAngle) {
        // newAngle -= 360;
        // }
        // SmartDashboard.putNumber("newangle", newAngle);

        // final double turnOutput = m_turretPIDController
        // .calculate(getTurretAngleDegrees(), newAngle);
        // final double turnFeedforward = m_turretFeedForward
        // .calculate(m_turretPIDController.getSetpoint().velocity);
        // SmartDashboard.putNumber("turret PID vel",
        // m_turretPIDController.getSetpoint().velocity);
        // SmartDashboard.putNumber("turret ff", turnFeedforward);
        // SmartDashboard.putNumber("turret PID pos err",
        // m_turretPIDController.getPositionError());
        // // m_turnTableTalon.setVoltage(turnOutput + turnFeedforward);
        // m_turnTableTalon.setVoltage(turnFeedforward + turnOutput);
    }

    public double angleDegreesToEncoderTicks(double degrees) {
        // double setAngle = Math.IEEEremainder(degrees, 360);
        return (degrees) * kEncoderTicksPerDegree;
    }

    // return
    public double getTurretAngleDegrees() {
        double angleDegrees = (m_turntableEncoder.getPosition() - kZeroOffsetDegrees);
        // return Math.IEEEremainder(angleDegrees, 360);
        return angleDegrees;
    }

    public double getTurretAngleCANcoder() {
        return m_turntableEncoder.getPosition();
    }

    public void setTurnTableAngleFortFive() {
        setTurretAngle(0);
    }

    public void setTurnTableAngleHundred() {
        setTurretAngle(-90);
    }

    public void setTurnTableAngleNegFortFive() {
        setTurretAngle(180);
    }

    public void setTurnTableAngleNegHundred() {
        setTurretAngle(90);
    }

    public void setTurntableToNTValue() {
        // m_turnTableTalon.setVoltage(ntVoltage.getDouble(0.0));
        // setTurretAngle(ntVoltage.getDouble(0.0));
        System.out.println("*** SETTING POSITION ***");
        SmartDashboard.putBoolean("Enter New Code", true);
    }

}

/*
 * SHOOT TESTING DISTANCES: 2/19/2022
 * 8 feet:
 * top:2050
 * bot:2200
 * -------
 * 9 feet
 * top:2100
 * bot:2250
 * ------
 * 10 feet
 * top:2300
 * bot:2400
 * ------
 * 11 feet
 * top:2350
 * bot:2450
 * ------
 * 12 feet
 * top:2550
 * bot:2550
 * ------
 * 13 feet
 * top:2600
 * bot:2650
 * ------
 * 14 feet
 * top:2670
 * bot:2720
 * -------
 * 15 feet
 * top:3300
 * bot:2300
 * -------
 * 16 feet
 * top:3400
 * bot:2400
 * -------
 * 17 feet
 * top:3480
 * bot:2480
 * -------
 * 18 feet
 * top:3565
 * bot:2565
 * -------
 * 19 feet
 * top:3900
 * bot:2550
 * -------
 * 20 feet
 * top: 4100
 * bot: 2350
 * -------
 * 21 feet
 * top: 4300
 * bot: 2400
 * ------
 * 22 feet
 * top: 5200
 * bot: 2200
 * ------
 * 23 feet
 * top: 5500
 * bot: 2400
 * ------
 * 24 feet
 * top:
 * bot:
 * ------
 * 25 feet
 * top:
 * bot:
 * ------
 * 26 feet
 * top:
 * bot:
 * ------
 */
