// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class Intake extends Base{

    public final WPI_TalonFX m_roller = new WPI_TalonFX(RobotMap.kIntake_roller_TalonFX);
    public final DoubleSolenoid doublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.kIntake_Pnuematic1, RobotMap.kIntake_Pnuematic2);

    ShuffleboardTab rollerMotorTab = Shuffleboard.getTab("Intake");
    NetworkTableEntry ntIntakeSpeed = rollerMotorTab.add("speed", 0.1).getEntry();

    @Override
    public void robotInit() {
        m_roller.configFactoryDefault();
        m_roller.setNeutralMode(NeutralMode.Brake);
        m_roller.setInverted(false);
        m_roller.setSensorPhase(false);
        
        doublePCM.set(Value.kReverse);
    }

    @Override
    public void robotPeriodic() {
        ntIntakeSpeed.setDouble(m_roller.getMotorOutputPercent());
    }

    @Override
    public void teleopPeriodic() {
        if(OI.getIntake()) {
            m_roller.set(ControlMode.PercentOutput, .1);
        } else {
            m_roller.set(ControlMode.PercentOutput, .1);
        }

        if(OI.togglePneumatic()) {
            doublePCM.toggle();
        }

    }
    

}
