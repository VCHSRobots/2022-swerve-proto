// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
public class VisionShooter extends Base {

    ShuffleboardTab visionBalltab = Shuffleboard.getTab("VisionShooter");

    // rename these in network tables if camera is rotated
    // NetworkTableEntry ntTargetYaw = visionBalltab.add("Yaw", 0).getEntry();
    // NetworkTableEntry ntTargetPitch = visionBalltab.add("Pitch", 0).getEntry();
    // NetworkTableEntry ntTargetArea = visionBalltab.add("Area", 0).getEntry();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ntTargetYaw = table.getEntry("tx");
    NetworkTableEntry ntTargetPitch = table.getEntry("ty");
    NetworkTableEntry ntTargetArea = table.getEntry("ta");
    NetworkTableEntry ntTargetValid = table.getEntry("tv");

    private PhotonTrackedTarget m_lastTarget;
    private double m_lastTx = 0;
    private double m_lastTy = 0;
    private boolean hasLastTarget = false;
    // private PhotonCamera camera = new PhotonCamera("mmal_service_16.1");
    private int m_lostCount = 0;
    private int m_icount = 0;
    private int m_MAXLOSTCOUNT = 10;
    private double m_yawOffset = 0;

    // public VisionBall(SwerveDrive sd) {
    public VisionShooter() {
        // m_SwerveDrive=sd;
        // connectCamera();
    }

    @Override
    public void robotInit() {
        // var cameraName="Microsoft_LifeCam_HD-3000";
        // connectCamera();
        Shuffleboard.getTab("shooter debug").addNumber("Vision Yaw", ()-> getYaw());
    }

    public void connectCamera() {
        var cameraName = "mmal_service_16.1";
        // if (camera == null)
        //     camera = new PhotonCamera(cameraName);
    }

    public double calculateAngleError() {
        // connectCamera();
        // if (camera == null)
        //     return -1;
        // var result = camera.getLatestResult();
        // System.out.println("calc angle");

        // update shuffle board
        m_icount++;
        if (m_icount % 50 == 0) {
            m_icount = 0;
            // if (result.hasTargets()) {
            //     ntTargetYaw.setDouble(result.getBestTarget().getYaw());
            //     ntTargetPitch.setDouble(result.getBestTarget().getPitch());
            //     ntTargetArea.setDouble(result.getBestTarget().getArea());
            // } else {
            //     ntTargetYaw.setDouble(-1);
            //     ntTargetPitch.setDouble(-1);
            //     ntTargetArea.setDouble(-1);
            // }
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
        // PhotonTrackedTarget target;
        double newTx;
        double newTy;
        var hasTarget = false;
        // if (!result.hasTargets()) {
        if (!(ntTargetValid.getDouble(0) == 1)) {
            if (hasLastTarget) {
                // target = m_lastTarget;
                newTx = m_lastTx;
                newTy = m_lastTy;
            } else {
                // m_SwerveDrive.drive(0, 0, 0, false);
                // return false;
                return 0;
            }
        } else {
            hasTarget = true;
            // target = result.getBestTarget();
            newTx = ntTargetYaw.getDouble(0);
            newTy = ntTargetPitch.getDouble(0);
            if (hasLastTarget) {
                if (Math.abs((newTx - m_lastTx) / m_lastTx) > 0.7) {
                        // || Math.abs(target.getArea() - m_lastTarget.getArea()) / m_lastTarget.getArea() > 0.5) {
                    hasTarget = false;
                }
            }
        }

        if (!hasTarget) {
            m_lostCount++;
            if (m_lostCount >= m_MAXLOSTCOUNT) {
                hasLastTarget = false;
                return 0;
            }
        } else {
            m_lostCount = 0;
        }

        // m_lastTarget = target;
        m_lastTx = newTx;
        m_lastTy = newTy;
        hasLastTarget = true;
        return newTx;
    }

    public double getYaw() {
        // if (m_lastTarget == null)
        //     return 0.0;
        return -m_lastTx;
    }

    public double getPitch() {
        // change this is camera is rotated
        // if (m_lastTarget == null)
        //     return 15.0;
        return m_lastTy;
    }

    public double getDistance() {
        double visionTargetOffsetFromCenter = 2;
        double distanceFeet = 14;
        double cameraAngle = Units.degreesToRadians(25.0); // angle from horizontal to axis of camera view
        double cameraHeight = 2; // height of camera on robot
        double targetHeight = 8.46833; // height of retroreflective tape in feet
        /*
         * from internet
         * distance = delta_height / (tan(cameraAngle + Pitch) * cos(Yaw))
         * 
         * d = (targetHeight - cameraHeight) / tan(cameraAngle + Pitch)
         */
        distanceFeet = visionTargetOffsetFromCenter + (targetHeight - cameraHeight)
                / (Math.tan(cameraAngle + Units.degreesToRadians(getPitch())) * Math.cos(getYaw()));
        return distanceFeet;
    }

}
