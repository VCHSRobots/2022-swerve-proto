// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Base;
import frc.robot.RobotMap;

/** Add your docs here. */
public class Climber extends Base {

    private DoubleSolenoid m_solenoid;

    private WPI_TalonFX m_master;
    private WPI_TalonFX m_follower;

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

        // motor configs
        m_master.configFactoryDefault();
        m_follower.configFactoryDefault();

        m_master.setNeutralMode(NeutralMode.Brake);
        m_follower.setNeutralMode(NeutralMode.Brake);

        m_master.setInverted(false);
        m_follower.setInverted(false);

        // init solenoids
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.kClimb_SolenoidForward,
                RobotMap.kClimb_SolenoidReverse);

        // init limit switches
        bottomLimit = new DigitalInput(RobotMap.kClimber_BottomLimitSwitch);
        topLimit = new DigitalInput(RobotMap.kClimber_TopLimitSwitch);

        // motors
        m_follower.follow(m_master);

        // solenoids
        m_solenoid.set(Value.kReverse);

        // encoder Value
        encoderValue = m_follower.getSelectedSensorPosition();
    }

    // Teleop Periodic
    public void climberMove() {
        // limit switch
        if (bottomLimit.get()) {
            m_follower.setSelectedSensorPosition(0);
        }
    }

    public void hooksForward() {
        m_solenoid.set(Value.kForward);


    }

    public void hooksReverse() {
        m_solenoid.set(Value.kReverse);

    }

    public void armsUp() {
        m_master.set(ControlMode.PercentOutput, 0.4);
    }

    public void armsDown() {

        m_master.set(ControlMode.PercentOutput, -0.4);
    }

    public void armsStop() {
        m_master.set(ControlMode.PercentOutput, 0);
    }

}
