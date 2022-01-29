// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Climber extends Base{

    private static DoubleSolenoid leftSolenoid;
    private static DoubleSolenoid rightSolenoid;

    private static WPI_TalonFX leftArm;
    private static WPI_TalonFX rightArm;
    private static WPI_TalonFX neutralExtra;

    //init
    static void init() {

        //init motors
        leftArm = new WPI_TalonFX(RobotMap.kClimb_LeftArm_TalonFX);
        rightArm = new WPI_TalonFX(RobotMap.kClimb_RightArm_TalonFX);
        neutralExtra = new WPI_TalonFX(RobotMap.kClimb_NeutralExtra_TalonFX);
        //init solenoids
        leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.kClimb_LeftSolenoidForward, RobotMap.kClimb_LeftSolenoidReverse);
        rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.kClimb_RightSolenoidForward, RobotMap.kClimb_RightSolenoidReverse);

        //motors
        leftArm.follow(rightArm);
        neutralExtra.follow(rightArm);
        //solenoids
        leftSolenoid.set(Value.kReverse);
        rightSolenoid.set(Value.kReverse);
    }

    //teleop periodic
    static void tPeriodic(boolean solenoidToggle, boolean armsUp, boolean armsDown) {
        //solenoids
        if(solenoidToggle) {
            leftSolenoid.toggle();
            rightSolenoid.toggle();
        }
        //motors
        if(armsUp) {
            rightArm.set(ControlMode.PercentOutput, 0.25);
        }
        if(armsDown) {
            rightArm.set(ControlMode.PercentOutput, -0.25);
        }

    }


  @Override
  public void robotInit() {    

    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);

    leftArm.follow(rightArm);
    neutralExtra.follow(rightArm);
    
    


  }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

        if (OI.getSolenoidToggle()) {

            leftSolenoid.toggle();
            rightSolenoid.toggle();

        }

        if (OI.getArmsUp()) {

            rightArm.set(ControlMode.PercentOutput, 0.25);

        } else if (OI.getArmsDown()) {

            rightArm.set(ControlMode.PercentOutput, -0.25);

        }

    }

}
