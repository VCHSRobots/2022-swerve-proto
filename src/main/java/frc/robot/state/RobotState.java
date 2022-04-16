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
    int msTillShoot = 60;

    MovingAverageTwist2d m_movingAverageTwist2d = new MovingAverageTwist2d(15);

    final public Pose2d kFieldToCenterHub = new Pose2d(new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(162)),
            new Rotation2d());
    final Translation2d kRobotToTurretTranslation = new Translation2d(); // estimate turret in center for now
    final Transform2d kTurretToCamera = new Transform2d(new Translation2d(Units.inchesToMeters(12), new Rotation2d()),
            new Rotation2d());

    double predictedBallAirTime = 0;
    static double predictedRobotDist = 0.0;

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

    public Twist2d movingAvg2d() {
        return m_movingAverageTwist2d.getAverage();
    }

    public double shooterDistDesired(double ftDist) {
        
        Twist2d predictedTwist2d = m_movingAverageTwist2d.getAverage();
        double loopsUntilShoot = (msTillShoot / 20.0);
        double predictedXDiff = Units.metersToFeet(predictedTwist2d.dx) * loopsUntilShoot;
        double predictedYDiff = Units.metersToFeet(predictedTwist2d.dy) * loopsUntilShoot;

        double turretDistDesired = Math.hypot(predictedXDiff, predictedYDiff) + ftDist;
        
        predictedRobotDist = Math.max(Math.min(turretDistDesired, 8.5), 14);

        return turretDistDesired;
    }

    public double shooterDistOffset() {
        return shooterDistDesired(0.0);
    }

    public double turretDegreesDesired() {

        Twist2d predictedTwist2d = m_movingAverageTwist2d.getAverage();
        int teleopCyclesInASecond = 50;

        double predictedXBallAirVelocity = Units.metersToFeet(predictedTwist2d.dx) * teleopCyclesInASecond;
        double predictedYBallAirVelocity = Units.metersToFeet(predictedTwist2d.dy) * teleopCyclesInASecond;
        double robotVelocityVector = Math.hypot(predictedXBallAirVelocity, predictedYBallAirVelocity);

        if (predictedXBallAirVelocity < 0 && predictedYBallAirVelocity < 0) {
            robotVelocityVector *= -1;
        } else {
            double xMult = predictedXBallAirVelocity == 0.0 ? 1 : predictedXBallAirVelocity / Math.abs(predictedXBallAirVelocity);
            double yMult = predictedYBallAirVelocity == 0.0 ? 1 : predictedYBallAirVelocity / Math.abs(predictedYBallAirVelocity);

            robotVelocityVector *= (xMult / yMult);
            // robotVelocityVector *= (xMult / yMult) * teleopCyclesInASecond;
        }

        // predictedBallAirTime = 0.14388*getPredictedPoseToTarget() - 0.35283;
        predictedBallAirTime = 0.1253239322 * Math.pow(1.220558191, predictedRobotDist);
        predictedBallAirTime = Math.min(predictedBallAirTime, 0.5);

        return -Units.radiansToDegrees(Math.atan((robotVelocityVector * predictedBallAirTime) / predictedRobotDist));
        // return Units.radiansToDegrees(Math.atan(predictedRobotDist / (robotVelocityVector * predictedBallAirTime)));

        // double innerAngle = Units.radiansToDegrees(Math.atan2(robotVelocityVector, predictedRobotDist));
        // double horizontalVelocity = robotVelocityVector * Math.sin(innerAngle);
        // double newPredictedDist = Math.sqrt(Math.pow(robotVelocityVector, 2) - Math.pow(horizontalVelocity, 2));

        // return Math.atan2(newPredictedDist * predictedBallAirTime, horizontalVelocity);

    }

    public double turretDegreesOverEstimate(double marginOfErr) {
        
        Twist2d predictedTwist2d = m_movingAverageTwist2d.getAverage();
        double loopsUntilShoot = (msTillShoot / 20.0) * 2.5;

        // System.out.println(marginOfErr);
        
        return marginOfErr - (turretDegreesDesired() * Math.hypot(predictedTwist2d.dx, predictedTwist2d.dy * loopsUntilShoot));
    }

    public boolean robotHasStableVelocity() {
        return m_movingAverageTwist2d.isAcceptablePrediction();
    }

    public boolean isResonableDist(double robotDist) {
        double minDist = 8.5;
        double maxDist = 14;
        double predictedDist = shooterDistDesired(robotDist);

        if (predictedDist > minDist && predictedDist < maxDist) {
            return true;
        }

        return false;
    }

    // public double turntableOverestimator() {
    //     double maxDegreeOverestimate = 6.0;
    //     double maxVelocityMetersASec = 3.0;
    //     double maxVelocityMetersPer20MS = maxVelocityMetersASec / 50;
    //     double robotVelocityVector = Math.sqrt((Math.pow(movingAvg2d().dx, 2) + Math.pow(movingAvg2d().dy, 2)));
    //     double xQuadrantAccounter = movingAvg2d().dx / Math.abs(movingAvg2d().dx);
    //     double yQuadrantAccounter = movingAvg2d().dy / Math.abs(movingAvg2d().dy);
        
    //     if (xQuadrantAccounter < 0 && yQuadrantAccounter < 0) {
    //         robotVelocityVector *= xQuadrantAccounter;
    //     } else {
    //         robotVelocityVector *= xQuadrantAccounter * yQuadrantAccounter;
    //     }

    //     return (robotVelocityVector / maxVelocityMetersPer20MS) * maxDegreeOverestimate;
    // }

    /**
    * resets the turntables odometry based on the turret and hub
    * @param limelightDist Current limelight distance in FEET to center hub
    * @param turretDegrees Current DEGREES Of turret
    * @return new pose 2d of the robot based PURELY on the shooter
    */
    // public Transform2d resetRobotPoseOdometry(double limelightDist, double turretDegrees, double turretYawDegreesOffset, double robotDegrees) {

    //     double limelightMetersDist = Units.feetToMeters(limelightDist);
    //     double x = Math.cos(Units.degreesToRadians(turretDegrees - turretYawDegreesOffset - robotDegrees)) * limelightMetersDist;
    //     double acSquared = (Math.pow(limelightMetersDist, 2) - Math.pow(x, 2));
    //     double y = Math.sqrt(acSquared);

    //     System.out.println("x: " + x);
    //     System.out.println("y: " + y);

    //     if (x < -0.5 || x > 17.0 || y < -0.5 || y > 9.0) {
    //         return new Transform2d(new Pose2d(), new Pose2d(-500, -500, new Rotation2d(0.0)));
    //     }

    //     return new Pose2d(x, y, new Rotation2d(0.0)).minus(kFieldToCenterHub);
    // }

}
