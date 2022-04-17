// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

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
    public double m_offset = 2.0;
    private double turretErrAllowance = 0.5;
    private int loopIterations = 0;

    private MovingAverage m_movingAverage = new MovingAverage(12);

    @Override
    public void robotInit() {
        Shuffleboard.getTab("debug").addNumber("Vision Yaw", () -> getYaw());
    }

    public boolean getTargetValid() {
        return ntTargetValid.getDouble(0) == 1.0;
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
        return -ntTargetYaw.getDouble(0) + m_offset;
    }

    public double getActualYaw() {
        return -ntTargetYaw.getDouble(0);
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
         * distance = delta_height / (tan(cameraAngle + Pitch) * cos(ntYaw))
         * 
         * d = (targetHeight - cameraHeight) / tan(cameraAngle + Pitch)
         */
        distanceFeet = visionTargetOffsetFromCenter + (targetHeight - cameraHeight)
                / (Math.tan(Units.degreesToRadians(cameraAngle + pitch)));
        return distanceFeet;
    }

    public double getMovingAverageDistance() {
        return m_movingAverage.getAverage();
    }

    public void addDistanceToMovingAverage() {
        m_movingAverage.add(getDistance());
    }

    public void setOffset(double offset) {
        m_offset = offset;
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

    public boolean canSeeTarget() {
        return hasLastTarget;
    }

    public boolean isOnTarget() {
        // TODO: should the be Math.abs(getYaw())?
        return getTargetValid() && Math.abs(getYaw()) < 6;
    }

    public boolean isWithinTurretErr(double desiredAngle) {
        if (desiredAngle == Double.NaN) {
            return false;
        } else if (getYaw() < m_offset + turretErrAllowance && getYaw() > m_offset - turretErrAllowance) {
            return true;
        }
        return false;
    }

    public boolean withinErrWithTime() {
        if (isWithinTurretErr(m_offset)) {
            loopIterations ++;
        } else {
            loopIterations = 0;
        }
        return loopIterations > 2;
    }

    public double limeShooterDistDesired() {
        double loopsAhead = 5.0;
        ArrayList<Double> averageDistPerSecDiff = new ArrayList<Double>();
        ArrayList<Double> limeLightDistances = m_movingAverage.getNumbers();
        double prevDist = 0;

        for (int i=0; i<m_movingAverage.getSize(); i++) {
            if (i==0) {
                prevDist = limeLightDistances.get(i);
            } else {
                averageDistPerSecDiff.add(limeLightDistances.get(i) - prevDist);
                prevDist = limeLightDistances.get(i);
            }
        }

        double averageChangeInDist = 0;
        for (int i = 0; i < averageDistPerSecDiff.size(); i++) {
            averageChangeInDist += averageDistPerSecDiff.get(i);
        }
        averageChangeInDist /= averageDistPerSecDiff.size();

        double turretDistDesired = getDistance() + (averageChangeInDist * loopsAhead);

        return turretDistDesired;
    }
}
