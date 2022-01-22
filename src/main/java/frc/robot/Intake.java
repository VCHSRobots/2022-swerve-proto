// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.TimerTask;
import java.util.ResourceBundle.Control;
import java.util.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class Intake extends Base {

    private Timer m_timer = new Timer();
    private TimerTask m_timerTask = new TimerTask() {
        @Override
        public void run() {
            m_state = STATE.E;
        };
    };

    public final WPI_TalonFX m_roller = new WPI_TalonFX(RobotMap.kIntake_roller_TalonFX);
    public final WPI_TalonFX m_mover = new WPI_TalonFX(RobotMap.kIntake_mover_TalonFX);
    public final WPI_TalonFX m_shooterIntake = new WPI_TalonFX(RobotMap.kIntake_shooterInput_TalonFX);
    public final DoubleSolenoid m_doublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            RobotMap.kIntake_Pnuematic1, RobotMap.kIntake_Pnuematic2);

    DigitalInput m_digitalInput = new DigitalInput(0);

    ShuffleboardTab intakeMotortab = Shuffleboard.getTab("Intake");
    NetworkTableEntry ntIntakeSpeed = intakeMotortab.add("speed", 0.1).getEntry();

    enum STATE {
        A, B, C, D, E;
    };

    STATE m_state = STATE.A;

    @Override
    public void robotInit() {
        m_roller.configFactoryDefault();
        m_roller.setNeutralMode(NeutralMode.Brake);
        m_roller.setInverted(false);
        m_roller.setSensorPhase(false);

        m_doublePCM.set(Value.kReverse);
    }

    @Override
    public void robotPeriodic() {
        ntIntakeSpeed.setDouble(m_roller.getMotorOutputPercent());
    }

    @Override
    public void teleopPeriodic() {
        switch (m_state) {
            case A:
                if (OI.startIntake()) {
                    m_state = STATE.B;
                }

                m_roller.set(ControlMode.PercentOutput, 0);
                m_mover.set(ControlMode.PercentOutput, 0);
                m_shooterIntake.set(ControlMode.PercentOutput, 0);

                m_doublePCM.set(Value.kReverse);

                break;
            case B:
                if (m_digitalInput.get()) {
                    m_state = STATE.C;
                }

                m_roller.set(ControlMode.PercentOutput, .1);
                m_mover.set(ControlMode.PercentOutput, .1);
                m_shooterIntake.set(ControlMode.PercentOutput, .1);

                m_doublePCM.set(Value.kForward);

                break;
            case C:
                if (ColorSensor.ballDetected) {
                    m_state = STATE.D;
                }
                m_roller.set(ControlMode.PercentOutput, .1);
                m_mover.set(ControlMode.PercentOutput, .1);
                m_shooterIntake.set(ControlMode.PercentOutput, 0);

                m_doublePCM.set(Value.kForward);

                break;
            case D:
                // state changes to E after timer
                m_timer.schedule(m_timerTask, 500);
                m_roller.set(ControlMode.PercentOutput, .1);
                m_mover.set(ControlMode.PercentOutput, .1);
                m_shooterIntake.set(ControlMode.PercentOutput, 0);

                m_doublePCM.set(Value.kForward);

                break;
            case E:
                m_doublePCM.set(Value.kReverse);

                m_roller.set(ControlMode.PercentOutput, 0);
                m_mover.set(ControlMode.PercentOutput, 0);
                m_shooterIntake.set(ControlMode.PercentOutput, 0);
                break;
        }

    }

}
