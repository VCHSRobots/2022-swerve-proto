package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDrive;

public class Auto {

    PathPlannerTrajectory trajectory1;
    PathPlannerTrajectory trajectory2;
    PathPlannerTrajectory trajectory3;
    PathPlannerTrajectory m_chosenTrajectory;

    int m_currentAutoStep = 0;
    PIDController xController = new PIDController(0.5, 0, 0);
    PIDController yController = new PIDController(0.5, 0, 0);
    static ProfiledPIDController thetaController = new ProfiledPIDController(2.7, 0, 0,
            new Constraints(SwerveDrive.kMaxAngularSpeed, 2 * SwerveDrive.kMaxAngularSpeed));
    HolonomicDriveController controller;

    Timer timer = new Timer();
    
    NetworkTableEntry ntGoalX = Shuffleboard.getTab("super").add("Goal/X", 0).getEntry(); 
    NetworkTableEntry ntGoalY = Shuffleboard.getTab("super").add("Goal/Y", 0).getEntry();
    NetworkTableEntry ntGoalRot = Shuffleboard.getTab("super").add("Goal/Rot", 0).getEntry();
    NetworkTableEntry ntGoalHolRot = Shuffleboard.getTab("super").add("Goal/Hol Rot", 0).getEntry();

    public Auto() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        controller = new HolonomicDriveController(xController, yController, thetaController);

        trajectory1 = PathPlanner.loadPath("auto1", 3, 3);
        trajectory2 = PathPlanner.loadPath("auto2", 1, 1);
        trajectory3 = PathPlanner.loadPath("strafeleft", 1, 1);
        

        Shuffleboard.getTab("super").addNumber("timer", ()->timer.get());

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
        m_currentAutoStep = 0;
    }

    private PathPlannerState getInitialState(PathPlannerTrajectory traj) {
        return traj.getInitialState();
    }
    
    public PathPlannerState getInitialState_auto1() {
        return getInitialState(trajectory1);
    }

    public PathPlannerState getInitialState_auto2() {
        return getInitialState(trajectory2);
    }

    public PathPlannerState getInitialState_auto3() {
        return getInitialState(trajectory3);
    }

    public ChassisSpeeds getNextChassisSpeeds_Auto1(Pose2d currentPose) {
        return getNextChassisSpeeds(currentPose, trajectory1);
    }

    public ChassisSpeeds getNextChassisSpeeds_Auto2(Pose2d currentPose) {
        return getNextChassisSpeeds(currentPose, trajectory2);
    }

    public ChassisSpeeds getNextChassisSpeeds_Auto3(Pose2d currentPose) {
        return getNextChassisSpeeds(currentPose, trajectory3);
    }

    public ChassisSpeeds getNextChassisSpeeds(Pose2d currentPose, PathPlannerTrajectory trajectory) {
        // around table to cadatorium path
        if (trajectory == null) {
            return new ChassisSpeeds();
        }
        ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
        PathPlannerState goal = new PathPlannerState();
        if (timer.get() < trajectory.getTotalTimeSeconds()) {
            // calculate speeds for trajectory
            goal = (PathPlannerState) trajectory.sample(timer.get());
            adjustedSpeeds = controller.calculate(currentPose, goal, goal.holonomicRotation);
        }
        ntGoalX.setNumber(goal.poseMeters.getX());
        ntGoalY.setNumber(goal.poseMeters.getY());
        ntGoalRot.setNumber(goal.poseMeters.getRotation().getDegrees());
        ntGoalHolRot.setNumber(goal.holonomicRotation.getDegrees());
        return adjustedSpeeds;
    }

    public boolean isTrajectoryCompleted() {
        return timer.get() > trajectory2.getTotalTimeSeconds();
    }

    // public ChassisSpeeds getNextChassisSpeeds_AroundTheTable(Pose2d currentPose) {
    //     // around table to cadatorium path
    //     if (trajectory3 == null) {
    //         return new ChassisSpeeds();
    //     }
    //     ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
    //     PathPlannerState goal = new PathPlannerState();
    //     if (timer.get() < trajectory3.getTotalTimeSeconds()) {
    //         // calculate speeds for trajectory
    //         goal = (PathPlannerState) trajectory3.sample(timer.get());
    //         adjustedSpeeds = controller.calculate(currentPose, goal, goal.holonomicRotation);
    //     }
    //     // Robot.m_swerve.driveFromChassisSpeeds(adjustedSpeeds);
    //     return adjustedSpeeds;
    // }

    // public ChassisSpeeds getNextChassisSpeeds_TrajWithPause(Pose2d currentPose) {

    //     // pause traj
    //     if (trajectory1 == null || trajectory2 == null) {
    //         System.out.println("could not load trajectory1 or trajectory2");
    //         return new ChassisSpeeds();
    //     }

    //     ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
    //     PathPlannerState goal = new PathPlannerState();

    //     // step 0 trajectory 1
    //     if (m_currentAutoStep == 0) {
    //         // if time is still less that total planned trajectory time
    //         if (timer.get() < trajectory1.getTotalTimeSeconds()) {
    //             // calculate speeds for trajectory
    //             goal = (PathPlannerState) trajectory1.sample(timer.get());
    //             adjustedSpeeds = controller.calculate(currentPose, goal, goal.holonomicRotation);

    //         }
    //         // if the trajectory is over
    //         else {
    //             // reset timer and go on to next step
    //             timer.reset();
    //             timer.start();
    //             m_currentAutoStep++;
    //         }
    //     }
    //     // step 1 pause in place
    //     else if (m_currentAutoStep == 1) {
    //         double time_to_wait = 1.5;
    //         if (timer.get() < time_to_wait) {
    //             // pause for shooting
    //         }
    //         // else, pause is over
    //         else {
    //             timer.reset();
    //             timer.start();
    //             m_currentAutoStep++;
    //         }
    //     }
    //     // step 2 trajectory 2
    //     else if (m_currentAutoStep == 2) {
    //         // if time is still left in trajectory, then run it
    //         if (timer.get() < trajectory2.getTotalTimeSeconds()) {
    //             // calc for trajectory
    //             goal = (PathPlannerState) trajectory2.sample(timer.get());
    //             adjustedSpeeds = controller.calculate(currentPose, goal, goal.holonomicRotation);
    //         }
    //     }
    //     // end of auto...
    //     else {
    //         adjustedSpeeds = new ChassisSpeeds();
    //     }

    //     // System.out.println(Robot.m_swerve.getPose2d());

    //     SmartDashboard.putNumber("goal/x", goal.poseMeters.getX());
    //     SmartDashboard.putNumber("goal/y", goal.poseMeters.getY());
    //     SmartDashboard.putNumber("goal/theta", goal.poseMeters.getRotation().getDegrees());

    //     SmartDashboard.putNumber("adjustedSpeeds/x", adjustedSpeeds.vxMetersPerSecond);
    //     SmartDashboard.putNumber("adjustedSpeeds/y", adjustedSpeeds.vyMetersPerSecond);
    //     SmartDashboard.putNumber("adjustedSpeeds/theta", adjustedSpeeds.omegaRadiansPerSecond);

    //     // Robot.m_swerve.driveFromChassisSpeeds(adjustedSpeeds);
    //     return adjustedSpeeds;

    // }

    public void robotPeriodic() {

    }

}
