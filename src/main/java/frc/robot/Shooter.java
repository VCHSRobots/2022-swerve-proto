package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Shooter extends Base {

    ShuffleboardTab ShootMotorTab = Shuffleboard.getTab("Shooter");

    NetworkTableEntry ntTopRPM = ShootMotorTab.add("Top RPM", 1000).getEntry();
    NetworkTableEntry ntBotRPM = ShootMotorTab.add("Bot RPM", 1000).getEntry();
    // NetworkTableEntry ntFeetToBotRPM = ShootMotorTab.add("FeetToBotRPM",
    // 13).getEntry();
    // NetworkTableEntry ntFeetToTopRPM = ShootMotorTab.add("FeetToTopRPM",
    // 13).getEntry();

    WPI_TalonFX ShootTalonTop = new WPI_TalonFX(RobotMap.kShoot_TopMotor_TalonFX);
    WPI_TalonFX ShootTalonBot = new WPI_TalonFX(RobotMap.kShoot_BottomMotor_TalonFX);
    WPI_TalonFX TurnTableTalon = new WPI_TalonFX(RobotMap.kTurnTableMotor_TalonFX);

    // SimpleMotorFeedforward m_ShootFeedForward = new SimpleMotorFeedforward(0.00,
    // 0.00045);

    // SHUFFLEBOARD HELPERS
    private double getTopMotorRPM() {
        return ticksPer100msToRPM(ShootTalonTop.getSelectedSensorVelocity());
    }

    private double getBotMotorRPM() {
        return ticksPer100msToRPM(ShootTalonBot.getSelectedSensorVelocity());
    }

    // private double DistToFeetTopRPM() {
    // return topfeettoRPM(ShootTalonTop.getSelectedSensorVelocity());
    // }

    // private double DistToFeetBotRPM() {
    // return botfeettoRPM(ShootTalonBot.getSelectedSensorVelocity());
    // }

    // END SHUFFLEBOARD HELPERS

    @Override
    public void robotInit() {
        ShootMotorTab.addNumber("Actual Top RPM", () -> getTopMotorRPM());
        ShootMotorTab.addNumber("Actual Bot RPM", () -> getBotMotorRPM());

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
        baseConfig.slot0.kP = 0.035; // 0.03

        ShootTalonBot.configFactoryDefault();
        ShootTalonTop.configFactoryDefault();
        TurnTableTalon.configFactoryDefault();

        ShootTalonBot.configAllSettings(baseConfig);
        ShootTalonTop.configAllSettings(baseConfig);

        ShootTalonBot.setNeutralMode(NeutralMode.Coast);
        ShootTalonTop.setNeutralMode(NeutralMode.Coast);

        ShootTalonBot.setInverted(false);
        ShootTalonTop.setInverted(false);

        ShootTalonBot.setSensorPhase(false);
        ShootTalonTop.setSensorPhase(false);

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
        // default all set outputs to 0
        double shootTopSpeed = 0;
        double shootBotSpeed = 0;
        double turntableSpeed = 0;
    
        
        if (OI.getRightTriggerAxisForShoot() > 0.5) {
            shootTopSpeed = rpmToTicksPer100ms(ntTopRPM.getNumber(0).doubleValue());
            shootBotSpeed = rpmToTicksPer100ms(ntBotRPM.getNumber(0).doubleValue());

        }
                // if(OI.getXButtonForToggleFeetToDist()){
                //     if(OI.getRightTriggerAxisForShoot() > 0.5){
                //     shootTopSpeed = rpmToTicksPer100ms(ntFeetToBotRPM.getNumber(0).doubleValue());
                //     shootBotSpeed = rpmToTicksPer100ms(ntFeetToTopRPM.getNumber(0).doubleValue());
                //     }
                //     elseif{
                // if (OI.getRightTriggerAxisForShoot() > 0.5) {
                //      shootTopSpeed = rpmToTicksPer100ms(ntTopRPM.getNumber(0).doubleValue());
                //      shootBotSpeed = rpmToTicksPer100ms(ntBotRPM.getNumber(0).doubleValue());
                //   }

       // }
        if (OI.getRightBumperForTurntable()) {
            turntableSpeed = 0.07;
        }
        if (OI.getLeftBumperForTurntable()) {
            turntableSpeed = -0.07;
        }

        ShootTalonTop.set(ControlMode.Velocity, shootTopSpeed);
        ShootTalonBot.set(ControlMode.Velocity, shootBotSpeed);
        TurnTableTalon.set(ControlMode.PercentOutput, turntableSpeed);

    }

    public double rpmToTicksPer100ms(double rpm) {
        double minutesPerSecond = 1.0 / 60.0;
        double secondsPer100ms = 1.0 / 10.0;
        double ticksPerRotation = 2048;
        double ticksPer100ms = rpm * minutesPerSecond * secondsPer100ms * ticksPerRotation;
        return ticksPer100ms;

    }

    public double ticksPer100msToRPM(double ticksPer100ms) {
        double secondsPerMinute = 60.0;
        double oneHundredMSPerSecond = 10.0;
        double rotationsPerTick = 1.0 / 2048;
        double RPM = ticksPer100ms * secondsPerMinute * oneHundredMSPerSecond * rotationsPerTick;
        return RPM;
    }

    // public double topfeettoRPM(double topfeet) {
    // double RPMquadtop = -1419.522 + 396.7329 * topfeet + -3.353022 * (topfeet *
    // topfeet);
    // return RPMquadtop;

    //}

    //public double botfeettoRPM(double botfeet) {
    //    double RPMquadbot = 8225 - 784.9 * botfeet + 32.04 * (botfeet * botfeet);
    //    return RPMquadbot;
    //}

    /*
     * NOTE: NOT ACTUAL RPM, INPUTTED RPM
     * 20 feet 5300 top 5600 bot
     * 17 feet 9 inch 4300
     * 15 feet 3900
     * 13 feet 3130 top 3200 bot
     * 11 feet 2800 top 3250 bot
     * 9 feet 1600 top 4000 bot
     * 7 feet 1270 top 4270 bot
     * top eq. y=-1,419.522+396.7329x+-3.353022x^2
     * bot eq. y=8225-784.9x+32.04x^2
     * 
     * 26feet 10inch
     */

}



