// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// A new auto-climber tools have been added to this file.  Also, overall control of the
// climber has been moved to this file. DLB, 3/23/22.

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
import edu.wpi.first.wpilibj.Timer;
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
    private double kInchesPerEncoderTick_Auto = 0.0000989; // Found by experiment
    private double kLastDisttoDownInches = 2.0;  // The limit at which the arms will go down with speed.
    private double kHighBarInches = 29.0;
    private boolean m_hasBeenCalibrated = false;

    // These variables used for the auto climbing functions...
    private int     m_state = 0;          // Possible states: 0=manual, 1=FIN, 2=NXT
    private double  m_nxtseq_t0 = 0.0;    // Time at which minro seq starts.
    private int     m_nxtseq = 0;         // Minor sequence number for NEXT auto.
    private double  kTopDelayForSwing = 0.75;   // Delay after arms extend and reach the next bar, for robot to swing in.
    private double  kTentionDistance = 21.50;   // Distance at which TE and SW hooks fully engaged on different bars.
    private double  kSwingBackDistance = 10.00; // DIstance at which we should be fully on next bar with TE hooks.
    private double  kWaitForTension = 1.0;     // TIme to wait for full tension after SW hooks in reverse.
    private double  m_lastArmSpeed = 0.0;      // Speed at which the arms are moving down during auto Next

    // init
    public void robotInit() {
        // sensors send
        Shuffleboard.getTab("computil").addBoolean("Left Limit", () -> !m_bottomLeftLimit.get());
        Shuffleboard.getTab("computil").addBoolean("Right Limit", () -> !m_bottomRightLimit.get());
        Shuffleboard.getTab("computil").addNumber("tePosInches", () -> getArmPositionInches());
        Shuffleboard.getTab("computil").addNumber("teCurrent", () -> m_master.getStatorCurrent());
        Shuffleboard.getTab("computil").addBoolean("AutoClimbReady", () -> getAutoClimbReady());
        Shuffleboard.getTab("computil").addNumber("AutoSequece", () -> getAutoSequence());
        Shuffleboard.getTab("super").addNumber("climbenc", () -> m_master.getSelectedSensorPosition()).withPosition(3, 0);
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

    // This is to allow adjustments to the auto climb during development.
    public void robotPeriodic() {
        // Update to update values here... ??
    }

    // THis is the control function that is called from tele_periodic and test_periodic. 
    // NOTE: For the new climber auto functions to work without breaking the robot,
    // This routine must be called on every TelePeriodic cycle!!  
    public void control(boolean shortHookBack, boolean shortHookForward, boolean armsUp, boolean armsDown,
        boolean climbNext, boolean climbFinish, double climbArmSpeed, boolean climbEStop) {

        // IF we are in an auto sequence, and ANY non-auto button is pressed, the auto
        // sequence is stopped, and we return to manual.
        if (shortHookBack || shortHookForward || armsUp || armsDown || climbEStop) {
            eStop();
        }

        if (m_state == 1) {
            continueClimbFinish();
            return;
        }
        if (m_state == 2) {
            continueClimbNext();
            return;
        }

        // If we fall through to here, we are in manual mode.  
        // Cleck to see if an auto fuction takes over.

        if (climbFinish) {
            startClimbFinish();
            return;
        }
        if (climbNext) {
            startClimbNext();
            return;
        }

        // No auto function active, resume manual mode...

        // solenoids
        if (shortHookBack) {
            hooksReverse();
        } else if (shortHookForward) {
            hooksForward();
        }

        if (armsUp && armsDown) {
            setClimberToZero();
        } else {
            // motors
            if (armsUp) {
            armsUp();
            } else if (armsDown) {
                armsDown();
            } else if (climbArmSpeed > 0.0) {
                armsDownAtSpeed(climbArmSpeed);
            } else {
                armsStop();
            }
        }
    }

    // Causes the auto funtions in the climber to stop.
    public void eStop() {
        if (m_state != 0) {
            armsStop();
        }
        m_state = 0;
    }

    // Teleop Periodic  (Note: I don't think this is ever called?)
    public void climberMove() {
        // limit switch
        // if (bottomLimit.get()) {
        m_follower_1.setSelectedSensorPosition(0);
        m_follower_2.setSelectedSensorPosition(0);
        // }
    }

    // Returns the Arm position in inches.  The arms are at zero
    // when the magnetic senser is actived.  As they extend up,
    // the position increases.  This number is ONLY valid
    // if the arms are calibrated.
    public double getArmPositionInches() {
        return m_master.getSelectedSensorPosition() * kInchesPerEncoderTick_Auto;
    }

    public void hooksForward() {
        m_solenoid.set(Value.kForward);
    }

    public void hooksReverse() {
        m_solenoid.set(Value.kReverse);

    }

    // Returns true if the hooks are forward.
    public boolean getHooksInForward() {
        if (m_solenoid.get() == Value.kForward) {
            return true;
        }
        return false;
    }

    public int getAutoSequence() {
        return m_nxtseq;
    }

    public void armsUp() {
        m_master.set(ControlMode.PercentOutput,0.95); // was 0.9

        m_follower_1.follow(m_master);
        m_follower_2.follow(m_master);
        // m_follower_1.setVoltage(0);
        // m_follower_2.setVoltage(0);
    }

    public void armsDown() {
        if (getClimberZero()) {
            m_master.set(ControlMode.PercentOutput, 0);
        } else if ( getArmPositionInches() < kLastDisttoDownInches) {
            m_master.set(ControlMode.PercentOutput, -0.3);
        } else {
            m_master.set(ControlMode.PercentOutput, -0.95); // was -0.85
        }
        m_follower_1.follow(m_master);
        m_follower_2.follow(m_master);
        // m_follower_1.setVoltage(0);
        // m_follower_2.setVoltage(0);
    }

    public void armsDownAtSpeed(double percent) {
        double x = -percent;
        if (getClimberZero()) {
            m_master.set(ControlMode.PercentOutput, 0);
        } else if ( getArmPositionInches() < kLastDisttoDownInches) {
            m_master.set(ControlMode.PercentOutput, -0.3);
        } else {
            m_master.set(ControlMode.PercentOutput, x); 
        }
        m_follower_1.follow(m_master);
        m_follower_2.follow(m_master);
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

    // public void goToHalf() {  // Not called?
    //     m_master.set(ControlMode.MotionMagic, 12 / kInchesPerEncoderTick);
    // }

    // public void goToFull() {  // Not called?
    //     m_master.set(ControlMode.MotionMagic, 27 / kInchesPerEncoderTick);
    // }

    // public void goToZero() {  // Not called?
    //     m_master.set(ControlMode.MotionMagic, 0);
    // }

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

    // This is called by RobotPeriodic.  If ever the climber arms are down,
    // the climber will be calibrated from that point on.
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
    
    public boolean getAutoClimbReady() {
        if (!isCalibrated()) return false;
        if (m_state == 0) return true;
        return false;
    }

    // Returns TRUE if either arm is all the way down.
    public boolean getClimberZero() {
        // add other limit too?
        return !m_bottomRightLimit.get() || !m_bottomLeftLimit.get();
    }

    // *******  Auto Sequence Functions ***********



    // Starts the auto operation to complete a climb to a bar.
    // It is assumed that the robot is engaged on the bar with
    // the TE hooks, and that the SW hooks can swing freely
    // in either direction.
    private void startClimbFinish() {
        if(!isCalibrated()) {
            // Must be calibrated to do this auto sequence.
            m_state = 0;
            return;
        }
        m_state = 1;
        m_nxtseq = 1;   // Start of sequence
        m_nxtseq_t0 = Timer.getFPGATimestamp();
        if (getHooksInForward()) {
            m_nxtseq = 1;
            m_nxtseq_t0 = Timer.getFPGATimestamp();
            hooksReverse();
            armsStop();
        } else {
            m_nxtseq = 2;
            m_nxtseq_t0 = Timer.getFPGATimestamp();
            armsUp();
        }
        return;
    }

    private void continueClimbFinish() {
        if (m_nxtseq == 1) {
            // Here, we are waiting for the SW hooks to change position.
            if (Timer.getFPGATimestamp() - m_nxtseq_t0 > 0.75) {
                armsDown();
                m_nxtseq = 2;
                m_nxtseq_t0 = Timer.getFPGATimestamp();
            }
            return;
        }
        if (m_nxtseq == 2) {
            armsDown();
            // Here, we are waiting for the TE arms to retract all the way.
            if (getClimberZero()) {
                armsStop();
                m_nxtseq = 3;
                m_nxtseq_t0 = Timer.getFPGATimestamp();
                hooksForward();
            }
            return;
        }
        if (m_nxtseq == 3) {
            armsStop();
            // Now we are waiting for the swing arms to engage.
            if (Timer.getFPGATimestamp() - m_nxtseq_t0 > 0.75) {
                m_nxtseq = 4;
                m_nxtseq_t0 = Timer.getFPGATimestamp();
            }
            return;
        }
        m_state = 0;
    }

    // Starts the auto sequence to climb to the next bar.
    // It is assumed that the robot is hanging by the SW hooks,
    // and that the TE hooks are mostly down and near the
    // SW hooks.  The TE hooks can be fully engaged.
    private void startClimbNext() {
        if(!isCalibrated()) {
            // Must be calibrated to do this auto sequence.
            m_state = 0;
            return;
        }
        m_state = 2;
        m_nxtseq = 1;   // Start of sequence
        m_nxtseq_t0 = Timer.getFPGATimestamp();
        // Start arms going up.
        armsUp();
    }

    private void continueClimbNext() {
        if (m_nxtseq == 1) {
            armsUp();
            // we are at the start... The arms should be extending up.  After 4 inches,
            // we need to swing the bot back.
            if (getArmPositionInches() > 4.0) {
                hooksForward();
                m_nxtseq = 2;
                m_nxtseq_t0 = Timer.getFPGATimestamp();
            }
            return;
        }
        if(m_nxtseq == 2) {
            armsUp();
            // We are extending to grab the next bar. Keep extending till the arms 
            // are far enough to grab the bar.  Then swing back to enage the hooks.
            if (getArmPositionInches() > kHighBarInches) {
                armsStop();
                m_nxtseq_t0 = Timer.getFPGATimestamp();
                m_nxtseq = 3;
                hooksReverse();
            }
            return;
        }
        if(m_nxtseq == 3) {
            // Here we are delaying for a bit for the robot to swing and have the
            // TE bars engage the upper bar.  Then we start pulling down, slowly to fully engage.
            if (Timer.getFPGATimestamp() - m_nxtseq_t0 > kTopDelayForSwing) {          
                m_nxtseq = 4;
                m_nxtseq_t0 = Timer.getFPGATimestamp();
                m_lastArmSpeed = 0.95;
                armsDownAtSpeed(m_lastArmSpeed); 
            }
            return;
        }
        if(m_nxtseq == 4) {
            // This is the crutial step!!
            // As we pull down, we must stop almost exactly at the tention distance.
            // As we get closer, slow down so that we do not overshoot too much.
            double distToGo = getArmPositionInches() - kTentionDistance;
            if (distToGo > 4.0) {
                m_lastArmSpeed = 0.95;
            } else if (distToGo < 0.0) {
                m_lastArmSpeed = 0.3;
            } else {
                m_lastArmSpeed = 0.3 + 0.65*(distToGo / 4.0);
            }
            armsDownAtSpeed(m_lastArmSpeed);
            if (getArmPositionInches() < kTentionDistance) {
                armsStop();
                m_nxtseq = 5;
                m_nxtseq_t0 = Timer.getFPGATimestamp();
                hooksForward();
            }
            return;
        }
        if(m_nxtseq == 5) {
            armsStop();
            // Here we wait to make sure the TE hooks are fully engaged.
            if (Timer.getFPGATimestamp() - m_nxtseq_t0 > kWaitForTension) {
                m_nxtseq = 6;  
                m_nxtseq_t0 = Timer.getFPGATimestamp();
                armsDown();
            }
            return;
        }
        if(m_nxtseq == 6) {
            armsDown();
            // At this point, it should be safe to climb on the next
            // bar up for a few inches. Do that, then let the operator
            // take over.
            if (getArmPositionInches() < kSwingBackDistance) {
                m_nxtseq = 7;
                m_nxtseq_t0 = Timer.getFPGATimestamp();
                armsStop();
                hooksReverse();
            }
            return;
        }
        armsStop();
        m_state = 0;
    }
}
