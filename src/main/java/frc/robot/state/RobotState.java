package frc.robot.state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.MovingAverageTwist2d;

public class RobotState {
    // SwerveDriveOdometry m_odometry;
    Pose2d m_pose;
    Rotation2d m_turretAngle;
    double m_lastTimestamp;
    double msTillShoot = 115;

    MovingAverageTwist2d m_movingAverageTwist2d = new MovingAverageTwist2d(10);

    final public Pose2d kFieldToCenterHub = new Pose2d(new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(162)),
            new Rotation2d());
    final Translation2d kRobotToTurretTranslation = new Translation2d(); // estimate turret in center for now
    final Transform2d kTurretToCamera = new Transform2d(new Translation2d(Units.inchesToMeters(12), new Rotation2d()),
            new Rotation2d());

    double predictedBallAirTime = 0;


    TimeInterpolatableBuffer<Pose2d> m_fieldToRobot = TimeInterpolatableBuffer.createBuffer(1);
    TimeInterpolatableBuffer<Pose2d> m_robotToTurret = TimeInterpolatableBuffer.createBuffer(1);

    // Translation X = Yaw, Y = Pitch.
    TimeInterpolatableBuffer<Translation2d> m_visionTargetYawPitch = TimeInterpolatableBuffer.createBuffer(1);

    public RobotState(Pose2d pose, Rotation2d turretAngleRadians) {
        update(pose, turretAngleRadians);
    }

    public void update(Pose2d pose, Rotation2d turretRotation) {
        double timestamp = Timer.getFPGATimestamp();
        Pose2d prevPose = m_fieldToRobot.getSample(timestamp);

        m_fieldToRobot.addSample(timestamp, pose);
        m_robotToTurret.addSample(timestamp, new Pose2d(kRobotToTurretTranslation, turretRotation));

        try {
            
            Transform2d poseDifference = pose.minus(prevPose);
            Twist2d currentTwist2d = new Twist2d(poseDifference.getX(), poseDifference.getY(), poseDifference.getRotation().getRadians());
    
            m_movingAverageTwist2d.add(currentTwist2d);
            predictedBallAirTime = 0.14388*getPredictedPoseToTarget() - 0.35283;
        
        } catch (NullPointerException e) {
            System.out.println(e);
        }
        
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

    // public Pose2d getTurretToCenterHub(double sampleTimeSeconds) {
    //     Pose2d robotToCenter = getRobotToCenterHub(sampleTimeSeconds);
    //     return robotToCenter.relativeTo(m_robotToTurret.getSample(sampleTimeSeconds));            
    // }

    public Rotation2d getTurretAimingAngle() {
        Pose2d robotToCenter = getRobotToCenterHub(Timer.getFPGATimestamp());
        Translation2d tr = robotToCenter.getTranslation();
        Rotation2d rot = new Rotation2d(tr.getX(), tr.getY());
        return rot;
    }

    public Rotation2d getTurretAimingAngle(double drivetrainAngularVelRadians) {
        Rotation2d preAngVelAngle = getTurretAimingAngle();
        double dt = 0.020;
        return preAngVelAngle.plus(new Rotation2d(drivetrainAngularVelRadians*dt));
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

    
    public double getRotationVelocity() {
        return m_movingAverageTwist2d.getAverage().dtheta;
    }

    /**
     * 
     * Gets the degrees the turret needs to be for the shooter to shoot at the target
     * 
     * @return degrees the turret should be at to shoot
     */
    public double getVelocityTurretDegrees() {
        
        Twist2d predictedTwist2d = m_movingAverageTwist2d.getAverage();
        double radiansToDegrees = 180 / Math.PI;
        double loopsUntilShoot = (msTillShoot / 20.0);

        Pose2d robotPose = getRobotToCenterHub(Timer.getFPGATimestamp());

        double predictedY = robotPose.getY() - (predictedTwist2d.dy * loopsUntilShoot) - (predictedTwist2d.dy * predictedBallAirTime);
        double predictedX = robotPose.getX() - (predictedTwist2d.dx * loopsUntilShoot) - (predictedTwist2d.dx * predictedBallAirTime);

        double predictedRadiansOfTurret = Math.atan2(predictedY, predictedX);

        return (predictedRadiansOfTurret * radiansToDegrees);

    }

    /**
     * 
     * Gets the degrees offset for the turntable that the shooter needs to be to shoot at the target
     * 
     * @return degrees offset the shooter should be to shoot
     */
    public double getVelocityTurretDegreesOffset(double turretAngleDegrees) {

        return (getVelocityTurretDegrees() - turretAngleDegrees);

    }

    /**
     * 
     * Gets predicted distance from robot to target based on odometry
     * 
     * @return predicted distance with velocity based on odometry
     */
    public double getPredictedPoseToTarget() {

        Twist2d predictedTwist2d = m_movingAverageTwist2d.getAverage();
        double loopsUntilShoot = (msTillShoot / 20.0);

        double predictedY = kFieldToCenterHub.getY() - (predictedTwist2d.dy * loopsUntilShoot);
        double predictedX = kFieldToCenterHub.getX() - (predictedTwist2d.dx * loopsUntilShoot);

        return (Math.sqrt((Math.pow(predictedY, 2) + Math.pow(predictedX, 2))));
    }

    public double getPredictedShotToTarget() {

        Twist2d predictedTwist2d = m_movingAverageTwist2d.getAverage();
        double loopsUntilShoot = (msTillShoot / 20.0);

        double predictedY = kFieldToCenterHub.getY() - (predictedTwist2d.dy * loopsUntilShoot) - (predictedTwist2d.dy * predictedBallAirTime * 50);
        double predictedX = kFieldToCenterHub.getX() - (predictedTwist2d.dx * loopsUntilShoot) - (predictedTwist2d.dy * predictedBallAirTime * 50);

        return (Math.sqrt((Math.pow(predictedY, 2) + Math.pow(predictedX, 2))));
    }

    /**
     * 
     * Creates an offset for the target distance to the robot by utilizing the limelight
     * 
     * @param distanceFromTarget distance from the target
     * @return predicted distance offset to target with aid from limelight
     */
    public double getPredictedDistanceToTargetOffset(double distanceFromTarget) {
        
        return getPredictedShotToTarget() - distanceFromTarget;

    }

    public boolean robotHasStableVelocity() {
        return m_movingAverageTwist2d.isAcceptablePrediction();
    }

    /**
    * resets the turntables odometry based on the turret and hub
    * @param limelightDist Current limelight distance in FEET to center hub
    * @param turretDegrees Current DEGREES Of turret
    * @return new pose 2d of the robot based PURELY on the shooter
    */
    public Transform2d resetRobotPoseOdometry(double limelightDist, double turretDegrees) {

        double x = Math.cos(Units.degreesToRadians(turretDegrees)) * Units.feetToMeters(limelightDist);
        double acSquared = (Math.pow(limelightDist, 2) - Math.pow(x, 2));
        double y = Math.sqrt(acSquared) * (acSquared / Math.abs(acSquared));
        
        if (x < -0.5 || x > 17.0 || y < -0.5 || y > 9.0) {
            return null;
        }

        return new Pose2d(x, y, new Rotation2d(0.0)).minus(kFieldToCenterHub);
    }

}
