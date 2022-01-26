// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Find a blue or red ball and drive toward it.  
 * for now, only work in teleop mode. 
 * 1. Read the most likely target detected by the PhotonVision RPi device from the Network table
 * 2. display the target on shuffleboard
 * 3. drive toward it.
*/
public class VisionBall extends Base {

    ShuffleboardTab visionBalltab = Shuffleboard.getTab("VisionBall");
    NetworkTableEntry ntVisionBallColor= visionBalltab.add("Ball Color","red").getEntry();

    public VisionBall() {
    }

    @Override
    public void robotInit() {
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    public void autonomousInit() {
    }

    public void autonomousPeriodic() {
    }

}
