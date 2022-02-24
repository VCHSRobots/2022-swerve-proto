package frc.robot.state;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

public class State {
    // SwerveDriveOdometry m_odometry;
    Pose2d_4415 m_pose;
    Rotation2d_4415 m_turretAngle;
    double m_lastTimestamp;

    final Translation2d kRobotToTurretTranslation = new Translation2d(); // estimate turret in center for now
    final Translation2d kTurretToCamera = new Translation2d(Units.inchesToMeters(12), new Rotation2d());

    final int kMapSize = 50; // 50 20 ms loops per second
    InterpolatingTreeMap<InterpolatingDouble, Pose2d_4415> m_fieldToRobot = new InterpolatingTreeMap<>(
            kMapSize);
    InterpolatingTreeMap<InterpolatingDouble, Rotation2d_4415> m_robotToTurret = new InterpolatingTreeMap<>(
            kMapSize);
    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_targetYawMap = new InterpolatingTreeMap<>(
            kMapSize);
    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_targetPitchMap = new InterpolatingTreeMap<>(
            kMapSize);

    public State(Pose2d pose, Rotation2d turretAngleRadians) {
        update(pose, turretAngleRadians);
    }

    public void update(Pose2d pose, Rotation2d turretAngleRadians) {
        InterpolatingDouble timestamp = new InterpolatingDouble(Timer.getFPGATimestamp());
        m_pose = new Pose2d_4415(pose);
        m_turretAngle = new Rotation2d_4415(turretAngleRadians);

        m_fieldToRobot.put(timestamp, m_pose);
        m_robotToTurret.put(timestamp, new Rotation2d_4415(turretAngleRadians));
    }

    public Pose2d getRobotToTurretPose() {
        return new Pose2d(kRobotToTurretTranslation, m_turretAngle.getRotation2d());
    }

    public Pose2d getFieldToRobotPose() {
        return m_pose.getPose2d();
    }

    public Translation2d getCameraToTarget(Rotation2d yaw, Rotation2d pitch) {
        double visionTargetOffsetFromCenter = 2;
        double distance = 14;
        double cameraAngle = Units.degreesToRadians(25.0); // angle from horizontal to axis of camera view
        double cameraHeight = Units.feetToMeters(2); // height of camera on robot
        double targetHeight = Units.inchesToMeters(8*12.0+8.0-1); // height of retroreflective tape
        /*
         * from internet
         * distance = delta_height / (tan(cameraAngle + Pitch) * cos(Yaw))
         * 
         * d = (targetHeight - cameraHeight) / tan(cameraAngle + Pitch)
         */
        distance = visionTargetOffsetFromCenter + (targetHeight - cameraHeight)
                / (Math.tan(cameraAngle + pitch.getRadians()) * Math.cos(yaw.getRadians()));
        return PhotonUtils.estimateCameraToTargetTranslation(distance, yaw);
    }
}
