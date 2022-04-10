package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.state.RobotState;
import frc.robot.subsystems.SwerveDrive;

public class DegreeErrEstimator_4415 {
    
    double degreeMarginOfErr = 0.0;
    RobotState robotState;
    SwerveDrive robotDrive;

    public DegreeErrEstimator_4415(RobotState m_state, SwerveDrive m_drive) {
        robotState = m_state;
        robotDrive = m_drive;
    }

    public double maxMarginOfErr(double airTimeOfBall, double ftDistToTarget, double avgRobotVelocity) {
        double maxDistOfTarget = 2.640625 + ftDistToTarget;
        double ballTravelDist = avgRobotVelocity * airTimeOfBall;
        Pose2d robotPose = robotState.getRobotToCenterHub(Timer.getFPGATimestamp());

        double worstCaseY = robotPose.getY();

        double maxMargin = 0.0;

        return robotState.getVelocityTurretDegrees() - maxMargin;
    }

    public double getDegreeMarginOfErr() {
        return degreeMarginOfErr;
    }

}
