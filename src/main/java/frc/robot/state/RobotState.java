package frc.robot.state;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

public class RobotState {
    // SwerveDriveOdometry m_odometry;
    Pose2d_4415 m_pose;
    Rotation2d_4415 m_turretAngle;
    double m_lastTimestamp;

    final Pose2d kFieldToCenterHub = new Pose2d(new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(162)),
            new Rotation2d());
    final Translation2d kRobotToTurretTranslation = new Translation2d(); // estimate turret in center for now
    final Transform2d kTurretToCamera = new Transform2d(new Translation2d(Units.inchesToMeters(12), new Rotation2d()),
            new Rotation2d());

    TimeInterpolatableBuffer<Pose2d> m_fieldToRobot = TimeInterpolatableBuffer.createBuffer(1);
    TimeInterpolatableBuffer<Pose2d> m_robotToTurret = TimeInterpolatableBuffer.createBuffer(1);

    // Translation X = Yaw, Y = Pitch.
    TimeInterpolatableBuffer<Translation2d> m_visionTargetYawPitch = TimeInterpolatableBuffer.createBuffer(1);

    public RobotState(Pose2d pose, Rotation2d turretAngleRadians) {
        update(pose, turretAngleRadians);
    }

    public void update(Pose2d pose, Rotation2d turretRotation) {
        double timestamp = Timer.getFPGATimestamp();
        m_fieldToRobot.addSample(timestamp, pose);
        m_robotToTurret.addSample(timestamp, new Pose2d(kRobotToTurretTranslation, turretRotation));
    }

    public Pose2d getFieldToRobotPose(double sampleTimeSeconds) {
        return m_fieldToRobot.getSample(sampleTimeSeconds);
    }

    public Pose2d getFieldToTarget() {
        return kFieldToCenterHub;
    }

    public Pose2d getRobotToCenterHub(double sampleTimeSeconds) {
        Pose2d robot = m_fieldToRobot.getSample(sampleTimeSeconds);
        return kFieldToCenterHub.relativeTo(robot);
    }

    public Pose2d getTurretToCenterHub(double sampleTimeSeconds) {
        Pose2d robotToCenter = getRobotToCenterHub(sampleTimeSeconds);
        return robotToCenter.relativeTo(m_robotToTurret.getSample(sampleTimeSeconds));            
    }

    public Rotation2d getTurretAimingAngle() {
        Pose2d robotToCenter = getRobotToCenterHub(Timer.getFPGATimestamp());
        Translation2d tr = robotToCenter.getTranslation();
        Rotation2d rot = new Rotation2d(tr.getX(), tr.getY());
        return rot;
    }


    // not tested
    public Translation2d getCameraToTargetPhoton(Rotation2d yaw, Rotation2d pitch) {
        double distance = getCameraToTargetDistance(yaw, pitch);
        double visionTargetOffsetFromCenter = Units.feetToMeters(2);
        double cameraToCenterHub = distance + visionTargetOffsetFromCenter;
        return PhotonUtils.estimateCameraToTargetTranslation(distance, yaw);
    }

    public double getCameraToTargetDistance(Rotation2d yaw, Rotation2d pitch) {
        double distance = 3;
        double horizontalToCameraAngle = Units.degreesToRadians(53.0); // angle from horizontal to axis of camera view
        double cameraHeight = Units.inchesToMeters(23.75); // height of camera on robot
        double targetHeight = Units.inchesToMeters(8 * 12.0 + 8.0 - 1); // height of retroreflective tape
        /*
         * from internet
         * distance = delta_height / (tan(cameraAngle + Pitch) * cos(Yaw))
         * 
         * d = (targetHeight - cameraHeight) / tan(cameraAngle + Pitch)
         */
        distance = (targetHeight - cameraHeight)
                / (Math.tan(horizontalToCameraAngle + pitch.getRadians()) * Math.cos(yaw.getRadians()));
        return distance;
    }
}
