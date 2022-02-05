// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.TimerTask;
import java.util.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.ColorSensor;

/** Add your docs here. */
public class Intake extends Base {

    private double m_defaultMotorSpeed = .1;

    private double m_motorSpeeds = 0;
    private boolean isChanging = false;

    private Timer m_timer = new Timer();
    private TimerTask m_changeToE = new TimerTask() {
        @Override
        public void run() {
            m_state = STATE.A;
        };
    };
    private TimerTask m_change = new TimerTask() {
        @Override
        public void run() {
            isChanging = false;
        };
    };

    public final WPI_TalonFX m_intake = new WPI_TalonFX(RobotMap.kIntake_roller_TalonFX);
    public final WPI_TalonFX m_mover = new WPI_TalonFX(RobotMap.kIntake_mover_TalonFX);
    public final WPI_TalonFX m_shooterLoader = new WPI_TalonFX(RobotMap.kIntake_shooterInput_TalonFX);
    public final DoubleSolenoid m_doublePCM = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            RobotMap.kIntake_Pnuematic1, RobotMap.kIntake_Pnuematic2);

    DigitalInput m_digitalInput = new DigitalInput(RobotMap.kIntake_LoaderDIO);

    ShuffleboardTab intakeMotortab = Shuffleboard.getTab("Intake Motors");
    NetworkTableEntry ntIntakeSpeed = intakeMotortab.add("intake", 0.1).getEntry();
    NetworkTableEntry ntMoverSpeed = intakeMotortab.add("mover", 0.1).getEntry();
    NetworkTableEntry ntShooterLoaderSpeed = intakeMotortab.add("shooter loader", 0.1).getEntry();

    enum STATE {
        A, B, C, D, E;
    };

    STATE m_state = STATE.A;

    // Robot Init
    public void init() {
        // motors
        m_intake.configFactoryDefault();
        m_intake.setNeutralMode(NeutralMode.Brake);
        m_intake.setInverted(false);
        m_intake.setSensorPhase(false);
        // solenoids
        m_doublePCM.set(Value.kReverse);
    }

    // Teleop Periodic
    public void changeState(boolean startIntake) {
        switch (m_state) {
            case A:

                // intake and load off, intake up

                if (startIntake) {
                    m_state = STATE.B;
                }

                m_intake.set(ControlMode.PercentOutput, 0);
                m_mover.set(ControlMode.PercentOutput, 0);
                m_shooterLoader.set(ControlMode.PercentOutput, 0);

                m_doublePCM.set(Value.kReverse);

                break;
            case B:
                // intake, bt, and load ON
                // intake down

                // ball detected right before shooter, go to next state
                if (m_digitalInput.get()) {
                    m_state = STATE.C;
                }

                m_intake.set(ControlMode.PercentOutput, m_defaultMotorSpeed);
                m_mover.set(ControlMode.PercentOutput, m_defaultMotorSpeed);
                m_shooterLoader.set(ControlMode.PercentOutput, m_defaultMotorSpeed);

                m_doublePCM.set(Value.kForward);

                break;
            case C:
                // intake, mover, ON, loader OFF
                // intake down

                m_motorSpeeds = .1;

                // SPIT BALL OUT IF BAD (WRONG COLOR) :))))))
                if (isChanging) {

                    m_motorSpeeds = -0.5;

                } else {
                    switch (DriverStation.getAlliance()) {
                        case Blue:
                            if (ColorSensor.redBallDetected) {
                                m_motorSpeeds = -.3;
                                isChanging = true;
                                // turns back to normal (isChanging = False) after 2.5 seconds
                                m_timer.schedule(m_change, 2500);
                            } else if (ColorSensor.blueBallDetected) {
                                m_state = STATE.D;
                            }
                            break;
                        case Red:
                            if (ColorSensor.blueBallDetected) {
                                m_motorSpeeds = -.3;
                                isChanging = true;
                                // turns back to normal (isChanging = False) after 2.5 seconds
                                m_timer.schedule(m_change, 2500);
                            } else if (ColorSensor.redBallDetected) {
                                m_state = STATE.D;
                            }
                            break;
                        case Invalid:
                            m_state = STATE.D;
                            break;
                    }

                }

                m_intake.set(ControlMode.PercentOutput, m_motorSpeeds);
                m_mover.set(ControlMode.PercentOutput, m_motorSpeeds);
                m_shooterLoader.set(ControlMode.PercentOutput, 0);

                m_doublePCM.set(Value.kForward);

                break;
            case D:
                // state changes to E after timer (inbetween state)
                m_timer.schedule(m_changeToE, 500);
                m_intake.set(ControlMode.PercentOutput, m_defaultMotorSpeed);
                m_mover.set(ControlMode.PercentOutput, m_defaultMotorSpeed);
                m_shooterLoader.set(ControlMode.PercentOutput, 0);

                m_doublePCM.set(Value.kForward);

                break;
            case E:
                // start loading balls into shooter (loadShooter)
                // stops when no more shooter buttons are pressed
                m_intake.set(ControlMode.PercentOutput, 0);
                m_mover.set(ControlMode.PercentOutput, .1);
                m_shooterLoader.set(ControlMode.PercentOutput, .1);

                break;
        }

        // sets nt values for motors
        setNTValues();

    }

    // Turns on shooter intake and mover to put balls in shooter.
    // Used in OI to coordinate shooting.
    public void loadShooter() {
        m_state = STATE.E;
    }

    // Goes back to the first state
    public void turnOffLoadShooter() {
        if(m_state == STATE.E) {
            m_state = STATE.A;
        }
    }

    // spit out ball
    public void reverseIntake() {
        m_mover.set(ControlMode.PercentOutput, -0.3);
        m_intake.set(ControlMode.PercentOutput, -0.3);
    }
    
    // sends motor values to shuffleboard
    public void setNTValues() {
        ntIntakeSpeed.setDouble(m_intake.getMotorOutputPercent());
        ntMoverSpeed.setDouble(m_mover.getMotorOutputPercent());
        ntShooterLoaderSpeed.setDouble(m_shooterLoader.getMotorOutputPercent());
    }

}
