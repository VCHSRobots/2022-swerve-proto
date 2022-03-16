// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.RobotMap;

/** Add your docs here. */
public class Climber extends Base {

    private DoubleSolenoid m_solenoid;

    private WPI_TalonFX m_master;
    private WPI_TalonFX m_follower_1;
    private WPI_TalonFX m_follower_2;

    private DigitalInput m_bottomLeftLimit = new DigitalInput(RobotMap.kClimber_LeftBottomLimit);
    private DigitalInput m_bottomRightLimit = new DigitalInput(RobotMap.kClimber_RightBottomLimit);
    public double m_encoderValue;

    private double kInchesPerEncoderTick = 0.00014573;
    private boolean m_hasBeenCalibrated = false;

    // init
    public void robotInit() {
        // sensors send
        Shuffleboard.getTab("computil").addBoolean("Left Limit", () -> !m_bottomLeftLimit.get());
        Shuffleboard.getTab("computil").addBoolean("Right Limit", () -> !m_bottomRightLimit.get());
        Shuffleboard.getTab("super").addNumber("climbenc", () -> m_master.getSelectedSensorPosition()).withPosition(12, 3);
        // init motors
        m_master = new WPI_TalonFX(RobotMap.kClimb_master_TalonFX, RobotMap.kCANivore_name);
        m_follower_1 = new WPI_TalonFX(RobotMap.kClimb_follower_TalonFX, RobotMap.kCANivore_name);
        m_follower_2 = new WPI_TalonFX(RobotMap.kClimb_follower2_TalonFX, RobotMap.kCANivore_name);

        // motor configs
        m_master.configFactoryDefault(50);
        m_follower_1.configFactoryDefault(50);
        m_follower_2.configFactoryDefault(50);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = 0.11;
        config.slot0.kI = 0.0;
        config.slot0.kD = 0.5;
        config.slot0.kF = 0.0;
        config.motionAcceleration = 24 / kInchesPerEncoderTick; // 6 in/ s*s
        config.motionCruiseVelocity = 18 / kInchesPerEncoderTick; // 12in per sec
        config.motionCurveStrength = 6;
        config.supplyCurrLimit.currentLimit = 30;
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.triggerThresholdCurrent = 30;
        config.supplyCurrLimit.triggerThresholdTime = 0.5;

        m_master.configAllSettings(config);

        m_master.setNeutralMode(NeutralMode.Brake);
        m_follower_1.setNeutralMode(NeutralMode.Brake);
        m_follower_2.setNeutralMode(NeutralMode.Brake);

        m_follower_1.follow(m_master);
        m_follower_2.follow(m_master);

        m_master.setInverted(false);
        m_follower_1.setInverted(InvertType.FollowMaster);
        m_follower_2.setInverted(InvertType.OpposeMaster);

        m_follower_1.follow(m_master);
        m_follower_2.follow(m_master);

        m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
        m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
        m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

        m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 50);
        m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 50);
        m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
        m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
        m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

        m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 50);
        m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 50);
        m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
        m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
        m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);


        // motor configs
        m_master.configFactoryDefault(100);
        m_follower_1.configFactoryDefault(100);
        m_follower_2.configFactoryDefault(100);

        m_master.setNeutralMode(NeutralMode.Brake);
        m_follower_1.setNeutralMode(NeutralMode.Brake);
        m_follower_2.setNeutralMode(NeutralMode.Brake);

        // m_master.setInverted(false);
        // m_follower_1.setInverted(false);
        // m_follower_2.setInverted(true);

        m_master.setSelectedSensorPosition(0);
        m_master.configForwardSoftLimitEnable(true);
        m_master.configForwardSoftLimitThreshold(300000);

        // init solenoids
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.kClimb_SolenoidForward,
                RobotMap.kClimb_SolenoidReverse);
        m_solenoid.set(Value.kReverse);

        // encoder Value
        // encoderValue = m_follower_1.getSelectedSensorPosition();
    }

    // Teleop Periodic
    public void climberMove() {
        // limit switch
        // if (bottomLimit.get()) {
        m_follower_1.setSelectedSensorPosition(0);
        m_follower_2.setSelectedSensorPosition(0);
        // }
    }

    public void hooksForward() {
        m_solenoid.set(Value.kForward);
    }

    public void hooksReverse() {
        m_solenoid.set(Value.kReverse);

    }

    public void armsUp() {
        m_master.set(ControlMode.PercentOutput, 0.9);

        m_follower_1.follow(m_master);
        m_follower_2.follow(m_master);
        // m_follower_1.setVoltage(0);
        // m_follower_2.setVoltage(0);
    }

    public void armsDown() {
        if (getClimberZero()) {
            m_master.set(ControlMode.PercentOutput, 0);
        } else if (m_master.getSelectedSensorPosition() < 50000) {
            m_master.set(ControlMode.PercentOutput, -0.3);
        } else {
            m_master.set(ControlMode.PercentOutput, -0.75);
        }
        m_follower_1.follow(m_master);
        m_follower_2.follow(m_master);
        // m_follower_1.setVoltage(0);
        // m_follower_2.setVoltage(0);
    }

    public void armsStop() {
        m_master.set(ControlMode.PercentOutput, 0);
        m_follower_1.follow(m_master);
        m_follower_2.follow(m_master);
        // m_follower_1.setVoltage(0);
        // m_follower_2.setVoltage(0);
    }

    public void resetPosition() {
        m_master.setSelectedSensorPosition(0);
    }

    public void goToHalf() {
        m_master.set(ControlMode.MotionMagic, 12 / kInchesPerEncoderTick);
    }

    public void goToFull() {
        m_master.set(ControlMode.MotionMagic, 27 / kInchesPerEncoderTick);
    }

    public void goToZero() {
        m_master.set(ControlMode.MotionMagic, 0);
    }

    public boolean setClimberToZero() {
        if (getClimberZero()) {
            // 13 to 62, 52 to 231, GEAR RATIO: 21.19
            m_hasBeenCalibrated = true;
            m_master.set(ControlMode.PercentOutput, 0);
            m_master.setSelectedSensorPosition(0);
            return true;
        } else {
            m_hasBeenCalibrated = false;
            m_master.set(ControlMode.PercentOutput, -0.07);
            return false;
        }
    }

    public void checkZero() {
        if (getClimberZero()) {
            m_master.setSelectedSensorPosition(0);
            // m_master.configReverseSoftLimitEnable(false);
            // m_master.configReverseSoftLimitThreshold(0);
            m_hasBeenCalibrated = true;
        }
    }

    public boolean isCalibrated() {
        return m_hasBeenCalibrated;
    }

    public boolean getClimberZero() {
        // add other limit too?
        return !m_bottomRightLimit.get() || !m_bottomLeftLimit.get();
    }

}
