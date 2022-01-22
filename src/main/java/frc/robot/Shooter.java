package frc.robot;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Base {

    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("Shooter");

    NetworkTableEntry ntTopRPM = ShootMotorTab.add("Top RPM", 1000).getEntry();
    NetworkTableEntry ntBotRPM = ShootMotorTab.add("BOT RPM", 1000).getEntry();

    WPI_TalonFX ShootTalonTop = new WPI_TalonFX(RobotMap.kShoot_BottomMotor_TalonFX);
    WPI_TalonFX ShootTalonBot = new WPI_TalonFX(RobotMap.kShoot_TopMotor_TalonFX);
    WPI_TalonFX TurnTableTalon = new WPI_TalonFX(RobotMap.kTurnTableMotor_TalonFX);

    double shootTopSpeed = 0;
    double shootBotSpeed = 0;
    double turntableSpeed = 0;

    SimpleMotorFeedforward m_ShootFeedForward = new SimpleMotorFeedforward(0, 0);

    @Override
    public void robotInit() {
        ShootMotorTab.addNumber("Actual Top RPM", () -> tickesPer100msToRPM(ShootTalonTop.getSelectedSensorVelocity()));
        ShootMotorTab.addNumber("Actual Bot RPM", () -> tickesPer100msToRPM(ShootTalonTop.getSelectedSensorVelocity()));

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
        baseConfig.slot0.kP = 0.0;
        baseConfig.slot0.kI = 0.0;
        baseConfig.slot0.kD = 0.0;
        baseConfig.slot0.kF = 0.0;
        baseConfig.slot0.kP = 0.03;

        ShootTalonBot.configFactoryDefault();
        ShootTalonTop.configFactoryDefault();
        TurnTableTalon.configFactoryDefault();

        ShootTalonBot.configAllSettings(baseConfig);
        ShootTalonTop.configAllSettings(baseConfig);

        TurnTableTalon.configOpenloopRamp(0.1);

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
    }

    @Override
    public void teleopPeriodic() {
        shootBotSpeed = 0;
        shootTopSpeed = 0;

        if (OI.getRightTriggerAxisForShoot() > 0.5) {
            shootTopSpeed = rpmToTicksper100ms(ntTopRPM.getNumber(0).doubleValue());
            shootBotSpeed = rpmToTicksper100ms(ntBotRPM.getNumber(0).doubleValue());
        } else {
            shootBotSpeed = 0;
            shootTopSpeed = 0;
        }
        if (OI.getRightBumperForTurntable()) {
            turntableSpeed = 0.2;
        }
        if (OI.getLeftBumperForTurntable()) {
            turntableSpeed = -0.2;
        }

        ShootTalonTop.set(ControlMode.Velocity, shootTopSpeed);
        ShootTalonBot.set(ControlMode.Velocity, shootBotSpeed);
        TurnTableTalon.set(ControlMode.PercentOutput, turntableSpeed);

    }

    public double rpmToTicksper100ms(double rpm) {
        double secondsinmin = 60;
        double secto100ms = 10;
        double tick = 2048;
        double rpmtoticks100 = rpm / secondsinmin / secto100ms * tick;
        return rpmtoticks100;

    }

    public double tickesPer100msToRPM(double ticksPer100ms) {
        return ticksPer100ms * 60 * 10 / 2048;
    }

}
