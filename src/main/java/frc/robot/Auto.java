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

    PathPlannerTrajectory trajectory1;
    PathPlannerTrajectory trajectory2;

    int m_currentAutoStep = 0;
    PIDController xController = new PIDController(0.4, 0, 0);
    PIDController yController = new PIDController(0.4, 0, 0);
    static ProfiledPIDController thetaController = new ProfiledPIDController(2.5, 0, 0,
            // new Constraints(0, 0));
            new Constraints(SwerveDrive.kMaxAngularSpeed, SwerveDrive.kMaxAngularSpeed));
    HolonomicDriveController controller;
    Timer timer = new Timer();

    public Auto() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        controller = new HolonomicDriveController(xController, yController, thetaController);

        trajectory1 = PathPlanner.loadPath("traj1", 2.5, 4);
        trajectory2 = PathPlanner.loadPath("traj2", 2.5, 4);
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
        if (trajectory1 == null || trajectory2 == null) {
            System.out.println("could not load trajectory1 or trajectory2");
            return;
        }

        ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
        PathPlannerState goal = new PathPlannerState();

        // step 0 trajectory 1
        if (m_currentAutoStep == 0) {
            // if time is still less that total planned trajectory time
            if (timer.get() < trajectory1.getTotalTimeSeconds()) {
                // calculate speeds for trajectory
                goal = (PathPlannerState) trajectory1.sample(timer.get());
                adjustedSpeeds = controller.calculate(Robot.m_swerve.getPose2d(), goal, goal.holonomicRotation);

            }
            // if the trajectory is over
            else {
                // reset timer and go on to next step
                timer.reset();
                timer.start();
                m_currentAutoStep++;
            }
        }
        // step 1 pause in place
        else if (m_currentAutoStep == 1) {
            double time_to_wait = 1.5;
            if (timer.get() < time_to_wait) {
                // pause for shooting
            }
            // else, pause is over
            else {
                timer.reset();
                timer.start();
                m_currentAutoStep++;
            }
        }
        // step 2 trajectory 2
        else if (m_currentAutoStep == 2) {
            // if time is still left in trajectory, then run it
            if (timer.get() < trajectory2.getTotalTimeSeconds()) {
                // calc for trajectory
                goal = (PathPlannerState) trajectory2.sample(timer.get());
                adjustedSpeeds = controller.calculate(Robot.m_swerve.getPose2d(), goal, goal.holonomicRotation);
            }
        }
        // end of auto...
        else {
            adjustedSpeeds = new ChassisSpeeds();
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
