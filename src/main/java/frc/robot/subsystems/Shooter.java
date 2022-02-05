package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Shooter extends Base {
    // Shuffleboard Tabs and NetworkTableEntries.
    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("Shooter");

    NetworkTableEntry ntBotRPM = ShootMotorTab.add("Bot RPM", 1000).withPosition(3, 3).withSize(1, 1).getEntry();
    NetworkTableEntry ntTopRPM = ShootMotorTab.add("Top RPM", 1000).withPosition(3, 2).withSize(1, 1).getEntry();
    NetworkTableEntry ntFeetToRPM = ShootMotorTab.add("Feet To Top RPM", 17).withPosition(4, 2).withSize(1, 1)
            .getEntry();

    WPI_TalonFX m_shootTalonTop = new WPI_TalonFX(RobotMap.kShoot_TopMotor_TalonFX);
    WPI_TalonFX m_shootTalonBot = new WPI_TalonFX(RobotMap.kShoot_BottomMotor_TalonFX);
    WPI_TalonFX m_turnTableTalon = new WPI_TalonFX(RobotMap.kTurnTableMotor_TalonFX);
    

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

        m_shootTalonBot.setInverted(false);
        m_shootTalonTop.setInverted(false);

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
    // Original Function for shooting RPM and distance.
    public void shooting(boolean distanceMode, boolean RPMMode) {
        // default all set outputs to 0
        double shootTopSpeed = 0;
        double shootBotSpeed = 0;

        if (distanceMode) {
            m_state = STATE.ShootingDistance;
        } else if (RPMMode) {
            m_state = STATE.ShootingRPM;
        } else {
            m_state = STATE.NotShooting;
        }

        if (m_state == STATE.NotShooting) {
            shootTopSpeed = 0;
            shootTopSpeed = 0;
        } else if (m_state == STATE.ShootingDistance) {
            setSpeedsDist(ntFeetToRPM.getNumber(0).doubleValue());
            

        } else if (m_state == STATE.ShootingRPM) {
            setSpeedsRPM(ntTopRPM.getNumber(0).doubleValue(), ntBotRPM.getNumber(0).doubleValue());
            
        }

    }
    // Shooting function with Distance. (NOT READY!!)
    public void shootingDist(double distanceMeters) {
        m_state = STATE.ShootingDistance;
        if (m_state == STATE.ShootingDistance) {
            setSpeedsDist(distanceMeters);
        }

    }
    // Shooting function with RPM.
    public void shootingRPM(double topRPM, double botRPM) {
        m_state = STATE.ShootingRPM;
        if (m_state == STATE.ShootingRPM) {
            setSpeedsRPM(topRPM, botRPM);
            
        }

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

        return errorBotRPM > 150 && errorTopRPM > 150;
    }
    public void turnMotorsOff() {
        setShootSpeeds(0, 0);
    }
}
