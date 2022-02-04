// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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

/** Add your docs here. */
public class Climber extends Base{

    private DoubleSolenoid m_solenoid;

    private WPI_TalonFX leftArm;
    private WPI_TalonFX rightArm;

    private DigitalInput bottomLimit;
    private DigitalInput topLimit;
    public double encoderValue;

    ShuffleboardTab ClimberTab = Shuffleboard.getTab("Climber Encoder");
    NetworkTableEntry ntClimberEncoderValue = ClimberTab.add("Climber Encoder Value", encoderValue).withPosition(2, 2).withSize(1, 1).getEntry();

    //init
    public void robotInit() {

        //init motors
        leftArm = new WPI_TalonFX(RobotMap.kClimb_LeftArm_TalonFX);
        rightArm = new WPI_TalonFX(RobotMap.kClimb_RightArm_TalonFX);

        // motor configs
        leftArm.configFactoryDefault();
        rightArm.configFactoryDefault();

        leftArm.setNeutralMode(NeutralMode.Brake);
        rightArm.setNeutralMode(NeutralMode.Brake);

        leftArm.setInverted(false);
        rightArm.setInverted(false);

        //init solenoids
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.kClimb_SolenoidForward, RobotMap.kClimb_SolenoidReverse);
        
        //init limit switches
        bottomLimit = new DigitalInput(RobotMap.kClimber_BottomLimitSwitch);
        topLimit = new DigitalInput(RobotMap.kClimber_TopLimitSwitch);

        //motors
        leftArm.follow(rightArm);

        //solenoids
        m_solenoid.set(Value.kReverse);

        //encoder Value
        encoderValue = rightArm.getSelectedSensorPosition();        
    }

    //Teleop Periodic
    public void climberMove(boolean shortHookBack, boolean armsUp, boolean armsDown) {
        //solenoids
        if(shortHookBack) {
            m_solenoid.set(Value.kReverse);
        } else {
            m_solenoid.set(Value.kForward);
        }
        //motors
        if(armsUp) {
            rightArm.set(ControlMode.PercentOutput, 0.15);
        } else {
            rightArm.set(ControlMode.PercentOutput, -0.15);
        }
        //limit switch
        if(bottomLimit.get()) {
            rightArm.setSelectedSensorPosition(0);
        }
    }
}
