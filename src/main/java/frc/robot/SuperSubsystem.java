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
public class SuperSubsystem {

    private SwerveDrive m_SwerveDrive;
    private OI m_OI;
    private Shooter m_shooter;
    private Climber m_climber;
    private Intake m_intake;
    private RobotMap m_robotMap;


    public SuperSubsystem(SwerveDrive swerveDrive, OI oi, Shooter shooter, Climber climber, Intake intake, RobotMap robotMap) {

        m_SwerveDrive = swerveDrive;
        m_OI = oi;
        m_shooter = shooter;
        m_climber = climber;
        m_intake = intake;
        m_robotMap = robotMap;

    }
    public void teleopPeriodic() {
        if (m_OI.getSolenoidToggle()) {

            
            m_climber.leftSolenoid.toggle();
            m_climber.rightSolenoid.toggle();

        }

        if(m_OI.getArmsUp()) {

            m_climber.rightArm.set(ControlMode.PercentOutput, 0.25);
        } 
        if(m_OI.getArmsDown()) {

        }

    }


   /* public class IntakeE extends Base {

        public void robotInit() {
        }
      
        public void robotPeriodic() {
        }

        public void autonomousInit() {
        }

        public void autonomousPeriodic() {
        }

        public void teleopInit() {
        }

        public void teleopPeriodic() {
        }
      
        public void disabledInit() {
        }
      
        public void disabledPeriodic() {
        }
      
        public void testInit() {
        }
      
        public void testPeriodic() {
        }
*/
    

}
