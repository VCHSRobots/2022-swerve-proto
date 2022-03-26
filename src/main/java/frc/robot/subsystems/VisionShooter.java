// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.util.MovingAverage;

public class VisionShooter extends Base {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ntTargetYaw = table.getEntry("tx");
    NetworkTableEntry ntTargetPitch = table.getEntry("ty");
    NetworkTableEntry ntTargetArea = table.getEntry("ta");
    NetworkTableEntry ntTargetValid = table.getEntry("tv");

    private double m_lastTx = 0;
    private double m_lastTy = 0;
    private boolean hasLastTarget = false;
    private int m_lostCount = 0;
    private int m_MAXLOSTCOUNT = 7;
    private double m_max_angle_change_in_20ms = 10.0;

    private MovingAverage m_movingAverage = new MovingAverage(10);

    @Override
    public void robotInit() {
        Shuffleboard.getTab("debug").addNumber("Vision Yaw", () -> getYaw());
    }

    public boolean getTargetValid() {
        return ntTargetValid.getDouble(0) == 1.0;
        // return ntTargetValid.getBoolean(false);
    }

    public double calculateAngleError() {
        /*
         * vision filter:
         * 1. prevent jumping between two different objects
         * check the difference between the current target and last target.
         * if too different, consider it as losing the original target. (see next filter
         * case)
         * 2. work through losing target for a few times.
         * if there is no targets for N consecutive times, stop;
         * if there is no target less than N times, keep going toward the orginal target
         */
        double newTx;
        double newTy;
        var hasTarget = false;

        if (ntTargetValid.getDouble(0) == 0) {
            if (hasLastTarget) {
                newTx = m_lastTx;
                newTy = m_lastTy;
            } else {
                return 0;
            }
        } else {
            hasTarget = true;
            newTx = ntTargetYaw.getDouble(0);
            newTy = ntTargetPitch.getDouble(0);
            if (hasLastTarget) {
                if (Math.abs((newTx - m_lastTx)) > m_max_angle_change_in_20ms) {
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

        m_lastTx = newTx;
        m_lastTy = newTy;
        hasLastTarget = true;
        return newTx;
    }

    public double getYaw() {
        return -ntTargetYaw.getDouble(0) + 0.0;
    }

    public double getPitch() {
        return m_lastTy;
    }

    public double getDistance() {
        double visionTargetOffsetFromCenter = 2;
        double distanceFeet = 0;
        double cameraAngle = 20.0; // angle from horizontal to axis of camera view, in degress
        double cameraHeight = 2.6667; // height of camera on robot
        double targetHeight = 8.46833; // height of retroreflective tape in feet

        double pitch = getPitch();
        /*
         * from internet
         * distance = delta_height / (tan(cameraAngle + Pitch) * cos(Yaw))
         * 
         * d = (targetHeight - cameraHeight) / tan(cameraAngle + Pitch)
         */
        distanceFeet = visionTargetOffsetFromCenter + (targetHeight - cameraHeight)
                / Math.tan(Units.degreesToRadians(cameraAngle + pitch));
        return distanceFeet;
    }

    public double getMovingAverageDistance() {
        return m_movingAverage.getAverage();
    }

    public void addDistanceToMovingAverage() {
        m_movingAverage.add(getDistance());
    }

    public void LEDoff() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void LEDon() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void LEDBlink() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    }

}
