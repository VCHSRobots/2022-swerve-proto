package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.Map;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;
import frc.robot.util.Interpolable;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.distanceRPMPoint;

public class Shooter extends Base {
    // turntable gear ratio
    // 13 to 62, 56 to 231, GEAR RATIO: 21.19
    // [motor rot / turret rot]*[enc ticks / motor rot]*[turret rot / 360 degrees]
    // private final double kEncoderTicksPerDegree = 21.19 * (2048.0) * (1.0 /
    // 360.0);
    private final double kEncoderTicksPerDegree = (231.0 / 56.0) * (62.0 / 13.0) * 2048 * (1.0 / 360.0); // 20351 / 180
                                                                                                         // measured cuz
                                                                                                         // math didn't
                                                                                                         // work

    private final double kZeroOffsetEncoderTicks = 0;
    private final double kZeroOffsetDegrees = 123;
    private final double kAngleOffset = 180;
    private final double kMinAngle = -135;
    private final double kMaxAngle = 300;
    private final double kMaxAngularVelocity = 525.0; // keep within 300, started at 135
    private final double kMaxAngularAcceleration = 1800.0; // keep within 1000, started at 455

    // Shuffleboard Tabs and NetworkTableEntries.
    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("Shooter");

    WPI_TalonFX m_shootTalonTop = new WPI_TalonFX(RobotMap.kShoot_TopMotor_TalonFX, RobotMap.kCANivore_name);
    WPI_TalonFX m_shootTalonBot = new WPI_TalonFX(RobotMap.kShoot_BottomMotor_TalonFX, RobotMap.kCANivore_name);
    WPI_TalonFX m_turnTableTalon = new WPI_TalonFX(RobotMap.kTurnTableMotor_TalonFX, RobotMap.kCANivore_name);

    CANCoder m_turntableEncoder = new CANCoder(RobotMap.kShooter_TurretEncoder, RobotMap.kCANivore_name);

    private int m_isOKtoShootCounter = 0;

    private ProfiledPIDController m_turretPIDController = new ProfiledPIDController(0.1, 0, 0,
            new Constraints(kMaxAngularVelocity, kMaxAngularAcceleration));
    private SimpleMotorFeedforward m_turretFeedForward = new SimpleMotorFeedforward(0, 0);

    // SimpleMotorFeedforward m_ShootFeedForward = new SimpleMotorFeedforward(0.00,
    // 0.00045);
    enum STATE {
        NotShooting, SpinningUp, ShootingDistance, ShootingRPM
    };

    STATE m_state = STATE.NotShooting;

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_interpolatingSpeeds_top = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_interpolatingSpeeds_bot = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

    public Shooter() {
        distanceRPMPoint[] distanceRPMlist = {
                new distanceRPMPoint(8, 2050, 2200),
                new distanceRPMPoint(9, 2100, 2250),
                new distanceRPMPoint(10, 2300, 2400),
                new distanceRPMPoint(11, 2350, 2450),
                new distanceRPMPoint(12, 2550, 2550),
                new distanceRPMPoint(13, 2600, 2650),
                new distanceRPMPoint(14, 2670, 2720),
                new distanceRPMPoint(15, 3300, 2300),
                new distanceRPMPoint(16, 3400, 2400),
                new distanceRPMPoint(17, 3480, 2480),
                new distanceRPMPoint(18, 3565, 2565),
                new distanceRPMPoint(19, 3900, 2550),
                new distanceRPMPoint(20, 4100, 2350),
                new distanceRPMPoint(21, 4300, 2400),
                new distanceRPMPoint(22, 5200, 2200),
                new distanceRPMPoint(23, 5500, 2400),

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
        ShootMotorTab.addNumber("Actual Top RPM", () -> getTopMotorRPM()).withPosition(5, 2);
        ShootMotorTab.addNumber("Actual Bot RPM", () -> getBotMotorRPM()).withPosition(5, 3);
        // ShootMotorTab.addBoolean("Is Ok to Shoot", () ->
        // IsOkToShoot()).withPosition(4, 1);
        ShootMotorTab.addNumber("Turn Table Position", () -> m_turnTableTalon.getSelectedSensorPosition());
        // ShootMotorTab.addBoolean("Has Been Zero'ed", () -> m_hasBeenCalibrated);

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
        baseConfig.slot0.allowableClosedloopError = 50;
        baseConfig.slot0.closedLoopPeakOutput = 1.0;
        baseConfig.slot0.closedLoopPeriod = 20;
        baseConfig.slot0.integralZone = 100;

        TalonFXConfiguration topConfig = baseConfig;
        topConfig.slot0.kI = 0.0;
        topConfig.slot0.kD = 0.0;
        topConfig.slot0.kF = 0.158;
        topConfig.slot0.kP = 0.042; // 0.03

        TalonFXConfiguration botConfig = baseConfig;
        botConfig.slot0.kI = 0.0;
        botConfig.slot0.kD = 0.0;
        botConfig.slot0.kF = 0.054;
        botConfig.slot0.kP = 0.045; // 0.03

        m_shootTalonBot.configFactoryDefault(100);
        m_shootTalonTop.configFactoryDefault(100);
        m_turnTableTalon.configFactoryDefault(100);

        m_shootTalonBot.configAllSettings(botConfig, 100);
        m_shootTalonTop.configAllSettings(topConfig, 100);

        m_shootTalonBot.setNeutralMode(NeutralMode.Coast);
        m_shootTalonTop.setNeutralMode(NeutralMode.Coast);
        m_turnTableTalon.setNeutralMode(NeutralMode.Brake);

        m_shootTalonBot.setInverted(false);
        m_shootTalonTop.setInverted(false);
        m_turnTableTalon.setInverted(false);

        m_shootTalonBot.setSensorPhase(false);
        m_shootTalonTop.setSensorPhase(false);

        TalonFXConfiguration turnTableConfig = baseConfig;
        turnTableConfig.peakOutputForward = .3;
        turnTableConfig.peakOutputReverse = -.3;
        // 0.1 @ 10 degree error
        // (0.1 * 1023) / (deg error * ticks / degree)
        turnTableConfig.slot0.kP = (1 * 1023) / (58 * kEncoderTicksPerDegree);
        turnTableConfig.slot0.kI = 0;
        turnTableConfig.slot0.kD = 0;
        turnTableConfig.slot0.kF = 0;
        turnTableConfig.slot0.allowableClosedloopError = 0.2 * kEncoderTicksPerDegree;
        turnTableConfig.supplyCurrLimit.currentLimit = 10;
        turnTableConfig.supplyCurrLimit.enable = true;
        turnTableConfig.supplyCurrLimit.triggerThresholdCurrent = 10;
        turnTableConfig.supplyCurrLimit.triggerThresholdTime = 0.5;
        // // [deg / s*s] * [ticks / deg] * [s / 100ms] = [tick / (100ms * s)]
        turnTableConfig.motionAcceleration = kMaxAngularAcceleration * kEncoderTicksPerDegree * (1.0 / 10.0); // ticks
                                                                                                              // per
                                                                                                              // 100ms
                                                                                                              // per sec
        // // [deg / s] * [tick / deg] * [s / 100ms] = [tick / 100ms]
        turnTableConfig.motionCruiseVelocity = kMaxAngularVelocity * kEncoderTicksPerDegree * (1.0 / 10.0);
        turnTableConfig.motionCurveStrength = 7;

        m_turnTableTalon.configAllSettings(turnTableConfig, 100);

        m_turnTableTalon.configOpenloopRamp(0.1);

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

        m_turntableEncoder.setPositionToAbsolute();
        m_turntableEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 50);

        // new Thread(() -> {
        // try {
        // Thread.sleep(1000);
        // SlotConfiguration con = new SlotConfiguration();
        // m_shootTalonBot.getSlotConfigs(con);
        // System.out.println("bot kp: ");
        // System.out.println(con.kP);
        // } catch (Exception e) {

        // }
        // }).start();
    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void autonomousInit() {
        m_state = STATE.NotShooting;
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        m_state = STATE.NotShooting;
        m_isOKtoShootCounter = 0;
    }

    public void warmUp() {
        shootingRPM(2400, 2400);
    }

    // Shooting function with Distance. (NOT READY!!)
    public void shootingDist(double distanceMeters) {
        m_state = STATE.ShootingDistance;
        setSpeedsDist(distanceMeters);

    }

    // Shooting function with RPM.
    public void shootingRPM(double topRPM, double botRPM) {
        m_state = STATE.ShootingRPM;
        setSpeedsRPM(topRPM, botRPM);
    }

    public void turnOff() {
        m_shootTalonBot.setVoltage(0);
        m_shootTalonTop.setVoltage(0);
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

        if (errorBotRPM < 80 && errorTopRPM < 80 && isBotFast) {
            m_isOKtoShootCounter++;
        } else {
            m_isOKtoShootCounter = 0;
        }
        return m_isOKtoShootCounter > 7;
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

    public void turnMotorsOff() {
        setShootSpeeds(0, 0);
    }

    public void configZeroSettings() {
        m_turnTableTalon.setSelectedSensorPosition(0);
        m_turnTableTalon.configReverseSoftLimitThreshold(-16100, 50);
        m_turnTableTalon.configForwardSoftLimitThreshold(8100, 50);
        m_turnTableTalon.configForwardSoftLimitEnable(true, 50);
        m_turnTableTalon.configReverseSoftLimitEnable(true, 50);
    }

    public void aimTurretTalonOnboard(double angleYawDegreesOffset) {
        if (Math.abs(angleYawDegreesOffset) < 0.5) {
            m_turnTableTalon.setVoltage(0);
            return;
        }
        double targetAngle = getTurretAngleDegrees() + angleYawDegreesOffset;
        setTurretAngle(targetAngle);
    }

    public void setTurretAngle(double angleTargetDegrees) {
        // double targetAngleTicks = angleDegreesToEncoderTicks(angleTargetDegrees);
        // // double ff = Math.copySign(0.038, -angleTargetDegrees); // kS to overcome
        // // friction
        // double ff = Math.copySign(0.033, targetAngleTicks -
        // m_turnTableTalon.getSelectedSensorPosition()); // kS to
        // // overcome
        // // friction
        // m_turnTableTalon.set(ControlMode.MotionMagic, targetAngleTicks,
        // DemandType.ArbitraryFeedForward, ff);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turretPIDController
                .calculate(getTurretAngleDegrees(), angleTargetDegrees);

        final double turnFeedforward = m_turretFeedForward
                .calculate(m_turretPIDController.getSetpoint().velocity);

        m_turnTableTalon.setVoltage(turnOutput + turnFeedforward);
    }

    public double angleDegreesToEncoderTicks(double degrees) {
        double newAngle = degrees - kAngleOffset;
        double setAngle = Math.IEEEremainder(newAngle, 360);

        return (setAngle) * kEncoderTicksPerDegree + kZeroOffsetEncoderTicks;
    }

    // returns [-180, 180]
    public double getTurretAngleDegrees() {
        double angleDegrees = (m_turntableEncoder.getPosition() - kZeroOffsetDegrees);
        // return Math.IEEEremainder(angleDegrees, 360);
        return angleDegrees;
    }

    public double getTurretAngleCANcoder() {
        return m_turntableEncoder.getPosition();
    }

    public void setTurnTableAngleFortFive() {
        m_turnTableTalon.setSelectedSensorPosition(angleDegreesToEncoderTicks(45), 0, 50);
    }

    public void setTurnTableAngleHundred() {
        m_turnTableTalon.setSelectedSensorPosition(angleDegreesToEncoderTicks(100), 0, 50);
    }

    public void setTurnTableAngleNegFortFive() {
        m_turnTableTalon.setSelectedSensorPosition(angleDegreesToEncoderTicks(-45), 0, 50);
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
