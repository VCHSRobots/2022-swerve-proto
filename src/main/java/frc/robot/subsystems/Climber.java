// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
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
    private WPI_TalonFX m_follower_1;
    private WPI_TalonFX m_follower_2;
    

    private DigitalInput bottomLeftLimit = new DigitalInput(RobotMap.kClimber_LeftBottomLimit);
    private DigitalInput bottomRightLimit = new DigitalInput(RobotMap.kClimber_RightBottomLimit);
    public double encoderValue;

    private double kInchesPerEncoderTick = 0.00014573;
    private boolean m_hasBeenCalibrated = false;

    ShuffleboardTab ClimberTab = Shuffleboard.getTab("Climber Encoder");
    NetworkTableEntry ntClimberEncoderValue = ClimberTab.add("Climber Encoder Value", encoderValue).withPosition(2, 2)
            .withSize(1, 1).getEntry();

    // init
    public void robotInit() {
        // init motors
        m_master = new WPI_TalonFX(RobotMap.kClimb_master_TalonFX, RobotMap.kCANivore_name);
        m_follower_1 = new WPI_TalonFX(RobotMap.kClimb_follower_TalonFX, RobotMap.kCANivore_name);
        m_follower_2 = new WPI_TalonFX(RobotMap.kClimb_follower2_TalonFX, RobotMap.kCANivore_name);

        // motor configs
        m_master.configFactoryDefault(100);
        m_follower_1.configFactoryDefault(100);
        m_follower_2.configFactoryDefault(100);

        m_master.setNeutralMode(NeutralMode.Brake);
        m_follower_1.setNeutralMode(NeutralMode.Brake);
        m_follower_2.setNeutralMode(NeutralMode.Brake);

        m_master.setInverted(false);
        m_follower_1.setInverted(false);
        m_follower_2.setInverted(true);

        m_follower_1.follow(m_master);
        m_follower_2.follow(m_master);

        m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 50);
        // m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 50);


        m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 50);
        // m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 50);

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
        if (getClimberToZero()) {
            // 13 to 62, 52 to 231, GEAR RATIO: 21.19
            m_hasBeenCalibrated = true;
            m_master.set(ControlMode.PercentOutput, 0);

            m_master.setSelectedSensorPosition(0);
            return true;
        } else {
            m_master.set(ControlMode.PercentOutput, -0.07);
            return false;
        }
    }

    public boolean getClimberToZero() {
        // add other limit too?
        return !bottomRightLimit.get() || !bottomLeftLimit.get();
    }

}
