package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;

public class Shooter extends Base {
    // Shuffleboard Tabs and NetworkTableEntries.
    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("Shooter");

    WPI_TalonFX m_shootTalonTop = new WPI_TalonFX(RobotMap.kShoot_TopMotor_TalonFX);
    WPI_TalonFX m_shootTalonBot = new WPI_TalonFX(RobotMap.kShoot_BottomMotor_TalonFX);
    WPI_TalonFX m_turnTableTalon = new WPI_TalonFX(RobotMap.kTurnTableMotor_TalonFX);
    DigitalInput m_TurnTableZero = new DigitalInput(RobotMap.kShooter_TurretZeroDIO);
    public boolean m_hasBeenCalibrated = false;

    // SimpleMotorFeedforward m_ShootFeedForward = new SimpleMotorFeedforward(0.00,
    // 0.00045);
    enum STATE {
        NotShooting, SpinningUp, ShootingDistance, ShootingRPM
    };

    STATE m_state = STATE.NotShooting;

    // SHUFFLEBOARD HELPERS
    private double getTopMotorRPM() {
        return ticksPer100msToRPM(m_shootTalonTop.getSelectedSensorVelocity());
    }

    private double getBotMotorRPM() {
        return ticksPer100msToRPM(m_shootTalonBot.getSelectedSensorVelocity());
    }

    // END SHUFFLEBOARD HELPERS

    public void robotInit() {
        ShootMotorTab.addNumber("Actual Top RPM", () -> getTopMotorRPM()).withPosition(5, 2);
        ShootMotorTab.addNumber("Actual Bot RPM", () -> getBotMotorRPM()).withPosition(5, 3);
        ShootMotorTab.addBoolean("Is Ok to Shoot", () -> IsOkToShoot()).withPosition(4, 1);
        ShootMotorTab.addNumber("Turn Table Position", () -> m_turnTableTalon.getSelectedSensorPosition());
        ShootMotorTab.addBoolean("Has Been Zero'ed", () -> m_hasBeenCalibrated);

        TalonFXConfiguration baseConfig = new TalonFXConfiguration();
        baseConfig.closedloopRamp = 0.02;
        baseConfig.neutralDeadband = 0.005;
        baseConfig.nominalOutputForward = 0.0;
        baseConfig.nominalOutputReverse = 0.0;
        baseConfig.openloopRamp = 0.01;
        baseConfig.peakOutputForward = 1.0;
        baseConfig.peakOutputReverse = -1.0;
        baseConfig.statorCurrLimit.enable = true;
        baseConfig.statorCurrLimit.currentLimit = 30;
        baseConfig.statorCurrLimit.triggerThresholdCurrent = 30;
        baseConfig.statorCurrLimit.triggerThresholdTime = 1;
        baseConfig.supplyCurrLimit.enable = true;
        baseConfig.supplyCurrLimit.currentLimit = 30;
        baseConfig.supplyCurrLimit.triggerThresholdCurrent = 30;
        baseConfig.supplyCurrLimit.triggerThresholdTime = 1;
        baseConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
        baseConfig.voltageCompSaturation = 11;
        baseConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        baseConfig.slot0.allowableClosedloopError = 50;
        baseConfig.slot0.closedLoopPeakOutput = 1.0;
        baseConfig.slot0.closedLoopPeriod = 20;
        baseConfig.slot0.integralZone = 100;
        baseConfig.slot0.kI = 0.0;
        baseConfig.slot0.kD = 0.0;
        baseConfig.slot0.kF = 0.05;
        baseConfig.slot0.kP = 0.038; // 0.03

        m_shootTalonBot.configFactoryDefault();
        m_shootTalonTop.configFactoryDefault();
        m_turnTableTalon.configFactoryDefault();

        m_shootTalonBot.configAllSettings(baseConfig);
        m_shootTalonTop.configAllSettings(baseConfig);

        m_shootTalonBot.setNeutralMode(NeutralMode.Coast);
        m_shootTalonTop.setNeutralMode(NeutralMode.Coast);
        m_turnTableTalon.setNeutralMode(NeutralMode.Brake);

        m_shootTalonBot.setInverted(false);
        m_shootTalonTop.setInverted(false);
        m_turnTableTalon.setInverted(false);

        m_shootTalonBot.setSensorPhase(false);
        m_shootTalonTop.setSensorPhase(false);

        m_turnTableTalon.configOpenloopRamp(0.1);

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
        setShootSpeeds(topFeetToRPM(distanceFeet),
                botFeetToRPM(distanceFeet));

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
        double RPMquadtop = -1419.522 + 396.7329 * topfeet + -3.353022 * (topfeet * topfeet);
        return RPMquadtop;
    }

    // Equation for Bot Motor.
    public double botFeetToRPM(double botfeet) {
        double RPMquadbot = 8225 - 784.9 * botfeet + 32.04 * (botfeet * botfeet);
        return RPMquadbot;
    }

    // Boolean that checks if shooter is reading to shoot at a good speed.
    public boolean IsOkToShoot() {
        double errorTopRPM = rpmToTicksPer100ms(m_shootTalonTop.getClosedLoopError());
        double errorBotRPM = rpmToTicksPer100ms(m_shootTalonBot.getClosedLoopError());

        return errorBotRPM < 75 && errorTopRPM < 75;
    }

    public void turnMotorsOff() {
        setShootSpeeds(0, 0);
    }

    public boolean setTurnTableToZero() {
        if (getTurnTableZero()) {
            // 13 to 62, 52 to 231, GEAR RATIO: 21.19
            m_hasBeenCalibrated = true;
            m_turnTableTalon.setSelectedSensorPosition(0);
            m_turnTableTalon.configReverseSoftLimitThreshold(-10000);
            m_turnTableTalon.configForwardSoftLimitThreshold(12000);
            m_turnTableTalon.configForwardSoftLimitEnable(true);
            m_turnTableTalon.configReverseSoftLimitEnable(true);

            m_turnTableTalon.set(ControlMode.PercentOutput, 0);
            return true;
        } else {
            m_turnTableTalon.set(ControlMode.PercentOutput, -0.07);
            return false;
        }

    }

    public boolean getTurnTableZero() {
        return !m_TurnTableZero.get();
    }

    public boolean isAtZero() {
        return !m_TurnTableZero.get();
    }
}
