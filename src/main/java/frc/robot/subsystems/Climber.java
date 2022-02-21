// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;

/** Add your docs here. */
public class Climber extends Base {

    private DoubleSolenoid m_solenoid;

    private WPI_TalonFX m_master;
    private WPI_TalonFX m_follower;
    private WPI_TalonFX m_follower2;

    private DigitalInput bottomLimit;
    private DigitalInput topLimit;
    public double encoderValue;

    ShuffleboardTab ClimberTab = Shuffleboard.getTab("Climber Encoder");
    NetworkTableEntry ntClimberEncoderValue = ClimberTab.add("Climber Encoder Value", encoderValue).withPosition(2, 2)
            .withSize(1, 1).getEntry();

    // init
    public void robotInit() {
        // init motors
        m_master = new WPI_TalonFX(RobotMap.kClimb_master_TalonFX);
        m_follower = new WPI_TalonFX(RobotMap.kClimb_follower_TalonFX);
        m_follower2 = new WPI_TalonFX(RobotMap.kClimb_follower2_TalonFX);

        // motor configs
        m_master.configFactoryDefault(50);
        m_follower.configFactoryDefault(50);
        m_follower2.configFactoryDefault(50);

        m_master.setNeutralMode(NeutralMode.Brake);
        m_follower.setNeutralMode(NeutralMode.Brake);
        m_follower2.setNeutralMode(NeutralMode.Brake);

        m_master.setInverted(false);
        m_follower.setInverted(InvertType.FollowMaster);
        m_follower2.setInverted(InvertType.OpposeMaster);

        m_follower.follow(m_master);
        m_follower2.follow(m_master);

        m_follower.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 50);
        m_follower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 50);
        m_follower2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 50);
        m_follower2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 50);

        m_master.setSelectedSensorPosition(0);
        m_master.configForwardSoftLimitEnable(true);
        m_master.configForwardSoftLimitThreshold(236981);

        // init solenoids
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.kClimb_SolenoidForward,
                RobotMap.kClimb_SolenoidReverse);
        m_solenoid.set(Value.kReverse);

        // init limit switches
        bottomLimit = new DigitalInput(RobotMap.kClimber_BottomLimitSwitch);
        topLimit = new DigitalInput(RobotMap.kClimber_TopLimitSwitch);

        // encoder Value
        encoderValue = m_master.getSelectedSensorPosition();

        // shuffleboard send
        Shuffleboard.getTab("climber").addNumber("climber pos", ()-> m_master.getSelectedSensorPosition());
    }

    // Teleop Periodic
    public void climberMove() {
        // limit switch
        if (bottomLimit.get()) {
            m_master.setSelectedSensorPosition(0);
        }
    }

    public void hooksForward() {
        m_solenoid.set(Value.kForward);
    }

    public void hooksReverse() {
        m_solenoid.set(Value.kReverse);

    }

    public void armsUp() {
        m_master.set(ControlMode.PercentOutput, 0.6);
    }

    public void armsDown() {

        m_master.set(ControlMode.PercentOutput, -0.6);
    }

    public void armsStop() {
        m_master.set(ControlMode.PercentOutput, 0);
    }

    public void resetPosition() {
        m_master.setSelectedSensorPosition(0);
    }
}
