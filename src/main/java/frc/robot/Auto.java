package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto {

    PathPlannerTrajectory pathPlannerTrajectory;

    PIDController xController = new PIDController(0, 0, 0);
    PIDController yController = new PIDController(0, 0, 0);
    static ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0,
            new Constraints(0, 0));
    // new Constraints(SwerveDrive.kMaxAngularSpeed, SwerveDrive.kMaxAngularSpeed));
    HolonomicDriveController controller = new HolonomicDriveController(xController, yController, thetaController);

    Timer timer = new Timer();

    public Auto() {
        pathPlannerTrajectory = PathPlanner.loadPath("line2meter", 2, 3);
    }

    public void robotInit() {

        Shuffleboard.getTab("main").add("xController", xController);
        Shuffleboard.getTab("main").add("yController", yController);
        Shuffleboard.getTab("main").add("theta controller", thetaController);


    }

    public void startTimer() {
        timer.start();
    }

    public void resetTimer() {
        timer.reset();
    }

    public void autonomousInit() {
        resetTimer();
        startTimer();
    }

    public void autonomousPeriodic() {
        if (pathPlannerTrajectory == null)
            return;
        ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
        var goal = (PathPlannerState) pathPlannerTrajectory.sample(timer.get());
        if (timer.get() < pathPlannerTrajectory.getTotalTimeSeconds()) {

            adjustedSpeeds = controller.calculate(Robot.m_swerve.getPose2d(), goal,
                    goal.holonomicRotation);

        } else {
            // adjustedSpeeds is initialized to zero, don't need to re assign
        }

        // System.out.println(Robot.m_swerve.getPose2d());

        SmartDashboard.putNumber("goal/x", goal.poseMeters.getX());
        SmartDashboard.putNumber("goal/y", goal.poseMeters.getY());
        SmartDashboard.putNumber("goal/theta", goal.poseMeters.getRotation().getDegrees());

        SmartDashboard.putNumber("adjustedSpeeds/x", adjustedSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("adjustedSpeeds/y", adjustedSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("adjustedSpeeds/theta", adjustedSpeeds.omegaRadiansPerSecond);

        Robot.m_swerve.driveFromChassisSpeeds(adjustedSpeeds);
    }

public void robotPeriodic() {


}

}
