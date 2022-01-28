// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Find a blue or red ball and drive toward it.  
 * for now, only work in teleop mode. 
 * 1. Read the most likely target detected by the PhotonVision RPi device from the Network table and display the target on shuffleboard
 * 2. drive toward it.
 *   2.1 what if the ball is moving
 * 3. if no ball found, turn around to find the ball.
*/
public class VisionBall extends Base {

    ShuffleboardTab visionBalltab = Shuffleboard.getTab("VisionBall");
    NetworkTableEntry ntTargetYaw= visionBalltab.add("Yaw",0).getEntry();
    NetworkTableEntry ntTargetPitch= visionBalltab.add("Pitch",0).getEntry();
    NetworkTableEntry ntTargetArea= visionBalltab.add("Area",0).getEntry();
    private PhotonCamera camera;
    private int icount=0;

    public VisionBall() {
    }

    @Override
    public void robotInit() {
        // Change this to match the name of your camera
        camera = new PhotonCamera("photonvision");

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
        icount++;
        var result = camera.getLatestResult();

        //update shuffle board
        if (icount % 30 == 0) {
            if (result.hasTargets()) {
                ntTargetYaw.setDouble(result.getBestTarget().getYaw());
                ntTargetPitch.setDouble(result.getBestTarget().getPitch());
                ntTargetArea.setDouble(result.getBestTarget().getArea());
            } else {
                ntTargetYaw.setDouble(-1);
                ntTargetPitch.setDouble(-1);
                ntTargetArea.setDouble(-1);
            }
        }
        if (!result.hasTargets()) {
            return;
        }

        /*
        double targetYaw=result.getBestTarget().getYaw();
        double targetPitch=result.getBestTarget().getPitch();
        double targetArea=result.getBestTarget().getArea();

        //drive toward the ball
        double xSpeed=()
        Robot.m_swerve.drive(xSpeed, double ySpeed, double rot, boolean fieldRelative,
            Translation2d centerOfRotationMeters) {
                */
    }

}
