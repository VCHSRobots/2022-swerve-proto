// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Find a blue or red ball and drive toward it.
 * for now, only work in teleop mode.
 * 1. Read the most likely target detected by the PhotonVision RPi device
 * from the Network table and display the target on shuffleboard
 * 2. drive toward it.
 * - what if the ball is moving
 * 3. if no ball found, turn around to find the ball.
 */
public class VisionBall extends Base {

    ShuffleboardTab visionBalltab = Shuffleboard.getTab("VisionBall");
    NetworkTableEntry ntTargetYaw = visionBalltab.add("Yaw", 0).getEntry();
    NetworkTableEntry ntTargetPitch = visionBalltab.add("Pitch", 0).getEntry();
    NetworkTableEntry ntTargetArea = visionBalltab.add("Area", 0).getEntry();
    private PhotonTrackedTarget m_lastTarget;
    private boolean hasLastTarget = false;
    private PhotonCamera camera;
    private int m_lostCount = 0;
    private int m_icount = 0;
    private int m_MAXLOSTCOUNT = 10;
    // private SwerveDrive m_SwerveDrive;

    // public VisionBall(SwerveDrive sd) {
    public VisionBall() {
        // m_SwerveDrive=sd;
    }

    @Override
    public void robotInit() {
        // var cameraName="Microsoft_LifeCam_HD-3000";
        var cameraName = "mmal_ballcam";
        camera = new PhotonCamera(cameraName);
    }

    // public boolean followBall() {
    public ChassisSpeeds followBall() {
        var result = camera.getLatestResult();

        // update shuffle board
        m_icount++;
        if (m_icount % 50 == 0) {
            m_icount = 0;
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

        /*
         * vision filter:
         * 1. prevent jumping between two different objects
         * check the difference between the current target and last target.
         * if too different, consider it as losing the original target. (see next filter
         * case)
         * 2. work through losing target for 1 or 2 times.
         * if there is no targets for 3 consecutive times, stop;
         * if there is no target less than 3 times, keep going toward the orginal target
         */
        PhotonTrackedTarget target;
        var hasTarget = false;
        double stopArea = 25;
        if (!result.hasTargets()) {
            if (hasLastTarget) {
                target = m_lastTarget;
            } else {
                // m_SwerveDrive.drive(0, 0, 0, false);
                // return false;
                return new ChassisSpeeds();
            }
        } else {
            hasTarget = true;
            target = result.getBestTarget();
            if (target.getArea() > stopArea) {
                hasTarget = false;
            }
            if (hasLastTarget) {
                if (Math.abs((target.getYaw() - m_lastTarget.getYaw()) / target.getYaw()) > 0.5
                        || Math.abs(target.getArea() - m_lastTarget.getArea()) / m_lastTarget.getArea() > 0.3) {
                    hasTarget = false;
                }
            }
        }
        if (!hasTarget) {
            m_lostCount++;
            if (m_lostCount >= m_MAXLOSTCOUNT) {
                hasLastTarget = false;
                // m_SwerveDrive.drive(0, 0, 0, false);
                // return false;
                return new ChassisSpeeds();
            }
        } else {
            m_lostCount = 0;
        }

        double targetYaw = target.getYaw();
        // double targetPitch=target.getPitch();
        double targetArea = target.getArea();

        double kpx = 0.1;
        double kpy = -1.0 / 60;
        double kpr = -1.0 / 60;

        // drive toward the ball
        double xSpeed = 0;
        double deltaArea = stopArea - targetArea;
        if (deltaArea > 0) {
            xSpeed = kpx * Math.sqrt(deltaArea);
        }
        double ySpeed = kpy * targetYaw;
        double rot = kpr * targetYaw;

        // m_SwerveDrive.drive(xSpeed, ySpeed, rot, false);

        m_lastTarget = target;
        hasLastTarget = true;
        // return true;
        return new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

}
