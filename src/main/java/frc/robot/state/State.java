package frc.robot.state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

public class State {
    // SwerveDriveOdometry m_odometry;
    Pose2d_4415 m_pose; 
    double m_turretAngleRadians;
    double m_lastTimestamp;
    final int kMapSize = 50; // 50 20 ms loops per second
    InterpolatingTreeMap<InterpolatingDouble, Pose2d_4415> m_poseMap = new InterpolatingTreeMap<InterpolatingDouble, Pose2d_4415>(kMapSize);
    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_turretAngleMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(kMapSize);

    public State(Pose2d pose, double turretAngleRadians) {
        update(pose, turretAngleRadians);
    }

    public void update(Pose2d pose, double turretAngleRadians) {
        InterpolatingDouble timestamp = new InterpolatingDouble(Timer.getFPGATimestamp());
        m_pose = new Pose2d_4415(pose);
        m_turretAngleRadians = turretAngleRadians;

        m_poseMap.put(timestamp, m_pose);
        m_turretAngleMap.put(timestamp, new InterpolatingDouble(m_turretAngleRadians));
    }
}
