// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.intake.input.*;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Intake extends Base {

    private boolean isChanging = false;

    private Timer m_timer = new Timer();

    public final WPI_TalonFX m_intake = new WPI_TalonFX(RobotMap.kIntake_roller_TalonFX, RobotMap.kCANivore_name);
    public final WPI_TalonFX m_mover = new WPI_TalonFX(RobotMap.kIntake_mover_TalonFX, RobotMap.kCANivore_name);
    public final WPI_TalonFX m_shooterLoader = new WPI_TalonFX(RobotMap.kIntake_shooterInput_TalonFX,
            RobotMap.kCANivore_name);
    public final DoubleSolenoid m_doublePCM = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            RobotMap.kIntake_Pnuematic1, RobotMap.kIntake_Pnuematic2);

    private final ColorSensor m_colorSensor = new ColorSensor();

    private final DigitalInput m_loadDIO = new DigitalInput(RobotMap.kIntake_LoadDIO);
    private final DigitalInput m_middleDIO = new DigitalInput(RobotMap.kIntake_MiddleDIO);

    ShuffleboardTab debugTab = Shuffleboard.getTab("computil");

    NetworkTableEntry ntColorFilterEnable = debugTab.add("Color Filter Enable", true)
            .withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 1).getEntry();

    // NetworkTableEntry ntIntakeSpeed = debugTab.add("intake actual",
    // 0.15).getEntry();
    // NetworkTableEntry ntMoverSpeed = debugTab.add("mover actual",
    // 0.15).getEntry();
    // NetworkTableEntry ntShooterLoaderSpeed = debugTab.add("loader actual",
    // 0.15).getEntry();

    // NetworkTableEntry ntMotorSpeed = debugTab.add("Intake percent out",
    // 0.3).getEntry();
    // NetworkTableEntry ntBTPercentOut = debugTab.add("BT percent out",
    // 0.5).getEntry();
    // NetworkTableEntry ntLoaderPercentOut = debugTab.add("loader percent out",
    // 0.5).getEntry();

    public IntakeState m_state = IntakeState.A;
    private InputManager inputManager;

    public Intake() {
        m_colorSensor.init();

        m_intake.configFactoryDefault(100);
        m_mover.configFactoryDefault(100);
        m_shooterLoader.configFactoryDefault(100);

        m_intake.setInverted(false);
        m_mover.setInverted(false);
        m_shooterLoader.setInverted(false);

        m_intake.setNeutralMode(NeutralMode.Brake);
        m_mover.setNeutralMode(NeutralMode.Brake);
        m_shooterLoader.setNeutralMode(NeutralMode.Brake);

        // only states B and C can have their motors reversed
        IntakeState.B.setReversable();
        IntakeState.C.setReversable();

        // add to shuffleboard
        // debugTab.addBoolean("Ball at Middle", () -> !m_middleDIO.get());
        // debugTab.addBoolean("Ball at Load", () -> !m_loadDIO.get());
        // debugTab.addNumber("Intake Vel", () -> m_intake.getSelectedSensorVelocity());
        // debugTab.addNumber("Middle Vel", () -> m_mover.getSelectedSensorVelocity());
        // debugTab.addNumber("Loader Vel", () ->
        // m_shooterLoader.getSelectedSensorVelocity());
        // debugTab.addNumber("Intake Cur", () -> m_intake.getSupplyCurrent());
        // debugTab.addNumber("Middle Cur", () -> m_mover.getSupplyCurrent());
        // debugTab.addNumber("Loader Cur", () -> m_shooterLoader.getSupplyCurrent());

        InputSetUpActions inputSetUpActions = new InputSetUpActions(this);
        inputManager = inputSetUpActions.getNewInputManager();
    }

    // Robot Init
    public void init() {
        // solenoids
        m_doublePCM.set(Value.kReverse);

    }

    public void turnOn() {
        if (getNumberOfBallsHolding() < 2) {
            m_state = IntakeState.B;
        }
    }

    @Override
    public void robotPeriodic() {
        m_colorSensor.checkColor();
        // m_colorSensor.updateNT();
    }

    public void autonomousInit() {
        m_state = IntakeState.C;
    }

    // Teleop Periodic
    public void changeState(boolean startIntake, boolean stopIntake) {

        // manage inputs
        inputManager.manageInputs(startIntake, stopIntake, isBallAtMiddle(), isBallAtLoad(), m_colorSensor.isBallDetected());

        // control motors and pnuematics
        m_intake.set(ControlMode.PercentOutput, m_state.intakeSpeed);
        m_mover.set(ControlMode.PercentOutput, m_state.ballTransportSpeed);

        if(m_state.isReversable) {
            spitWrongColorBallOut();
            if(isChanging) {
                reverseIntake();
                if (m_timer.get() >= .4) {
                    isChanging = false;
                    m_timer.stop();
                }
            }
        }

        m_shooterLoader.set(ControlMode.PercentOutput, m_state.loaderSpeed);

        setIntakePnuematic(m_state.pneumaticOut);

    }

    // toggles intake pnuematic, for testing purposes
    public void setIntakePnuematic(boolean forward) {
        m_doublePCM.set(forward ? Value.kForward : Value.kReverse);
    }

    // Turns on shooter intake and mover to put balls in shooter.
    // Used in OI to coordinate shooting.
    public void loadShooter() {
        if(isBallAtLoad() && !isBallAtMiddle()) {
            // intake out
            m_state = IntakeState.D;
        }
        // intake in
        m_state = IntakeState.E;
    }

    // Goes back to the first state
    public void turnOffLoadShooter() {
        m_state = IntakeState.A;
    }

    public void stopMotors() {
        m_state = IntakeState.A;
        m_intake.stopMotor();
        m_mover.stopMotor();
        m_shooterLoader.stopMotor();
    }

    // continues spinning intake motors if ball is there while shooting
    public void countinueIntakeMotors() {
        if (isBallAtLoad()) {
            m_state = IntakeState.A;
        } else {
            m_state = IntakeState.F;
        }
    }

    // spit out ball
    public void reverseIntake() {
        m_mover.set(ControlMode.PercentOutput, -0.45);
        m_intake.set(ControlMode.PercentOutput, -0.45);
    }

    public boolean unjamShooter() {
        m_shooterLoader.set(ControlMode.PercentOutput, -0.2);
        m_shooterLoader.set(ControlMode.PercentOutput, -0.2);
        return isBallAtLoad();
    }

    // sends motor values to shuffleboard
    public void setNTValues() {
        // ntIntakeSpeed.setDouble(m_intake.getMotorOutputPercent());
        // ntMoverSpeed.setDouble(m_mover.getMotorOutputPercent());
        // ntShooterLoaderSpeed.setDouble(m_shooterLoader.getMotorOutputPercent());
    }

    public void setState(IntakeState state) {
        m_state = state;
    }

    public boolean getBothBallsLoaded() {
        if (isBallAtLoad() && isBallAtMiddle()) {
            return true;
        }
        return false;
    }

    // helper functions so don't have to remember to invert DIO
    public boolean isBallAtLoad() {
        return !m_loadDIO.get();
    }

    public boolean isBallAtMiddle() {
        return !m_middleDIO.get();
    }

    public int getNumberOfBallsHolding() {
        int count = 0;
        count += isBallAtLoad() ? 1 : 0;
        count += (isBallAtMiddle()) ? 1 : 0;
        return count;
    }

    private void spitWrongColorBallOut() {
        switch (ntColorFilterEnable.getBoolean(true) ? DriverStation.getAlliance() : Alliance.Invalid) {
            case Blue:
                if (m_colorSensor.isRedBallDetected()) {
                    startSpitOutTimer();
                }
                break;
            case Red:
                if (m_colorSensor.isBlueBallDetected()) {
                    startSpitOutTimer();
                }
                break;
            case Invalid:
                break;
        }
    }

    // turns back to normal (isChanging = False) after 1 seconds
    private void startSpitOutTimer() {
        isChanging = true;

        m_timer.reset();
        m_timer.start();
    }

}
