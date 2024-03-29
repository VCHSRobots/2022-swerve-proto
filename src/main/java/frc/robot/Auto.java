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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveDrive;

public class Auto {

    PathPlannerTrajectory trajectory2;

    PathPlannerTrajectory trajectory_auto1b_part1;
    PathPlannerTrajectory trajectory_auto1b_part2;
    PathPlannerTrajectory trajectory_auto1b_part3;
    PathPlannerTrajectory trajectory_auto1b_part4;
    PathPlannerTrajectory m_chosenTrajectory;

    int m_currentAutoStep = 0;
    PIDController xController = new PIDController(1.5, 0, 0);
    PIDController yController = new PIDController(1.5, 0, 0);
    static ProfiledPIDController thetaController = new ProfiledPIDController(2.1, 0, 0,
            new Constraints(SwerveDrive.kMaxAngularSpeed, 2 * SwerveDrive.kMaxAngularSpeed));
    HolonomicDriveController controller;

    Timer timer = new Timer();
    
    // NetworkTableEntry ntGoalX = Shuffleboard.getTab("super").add("Goal/X", 0).getEntry(); 
    // NetworkTableEntry ntGoalY = Shuffleboard.getTab("super").add("Goal/Y", 0).getEntry();
    // NetworkTableEntry ntGoalRot = Shuffleboard.getTab("super").add("Goal/Rot", 0).getEntry();
    // NetworkTableEntry ntGoalHolRot = Shuffleboard.getTab("super").add("Goal/Hol Rot", 0).getEntry();

    public Auto() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        controller = new HolonomicDriveController(xController, yController, thetaController);

        trajectory2 = PathPlanner.loadPath("auto2", 1, 1);

        trajectory_auto1b_part1 = PathPlanner.loadPath("auto1b_part1", 3.5, 2.3);
        trajectory_auto1b_part2 = PathPlanner.loadPath("auto1b_part2", 3.5, 2.3);
        trajectory_auto1b_part3 = PathPlanner.loadPath("auto1b_part3", 3.5, 2.3);
        trajectory_auto1b_part4 = PathPlanner.loadPath("auto1b_part4", 3.5, 2.3);
    }

    public void robotInit() {

    }

    public void autonomousInit() {
        timer.start();
        timer.reset();
        m_currentAutoStep = 0;
    }

    public void setupAuto1bp1() {
        m_chosenTrajectory = trajectory_auto1b_part1;
        timer.start();
        timer.reset();
    }

    public void setupAuto1bp2() {
        m_chosenTrajectory = trajectory_auto1b_part2;
        timer.start();
        timer.reset();
    }

    public void setupAuto1bp3() {
        m_chosenTrajectory = trajectory_auto1b_part3;
        timer.start();
        timer.reset();
    }

    public void setupAuto1bp4() {
        m_chosenTrajectory = trajectory_auto1b_part4;
        timer.start();
        timer.reset();
    }

    public void setupAuto2() {
        m_chosenTrajectory = trajectory2;
        timer.start();
        timer.reset();
    }

    public PathPlannerState getInitialState() {
        return m_chosenTrajectory.getInitialState();
    }

    public ChassisSpeeds getNextChassisSpeeds(Pose2d currentPose) {
        // around table to cadatorium path
        if (m_chosenTrajectory == null) {
            return new ChassisSpeeds();
        }
        ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
        PathPlannerState goal = new PathPlannerState();
        if (timer.get() < m_chosenTrajectory.getTotalTimeSeconds()) {
            // calculate speeds for trajectory
            goal = (PathPlannerState) m_chosenTrajectory.sample(timer.get());
            adjustedSpeeds = controller.calculate(currentPose, goal, goal.holonomicRotation);
        }
        // ntGoalX.setNumber(goal.poseMeters.getX());
        // ntGoalY.setNumber(goal.poseMeters.getY());
        // ntGoalRot.setNumber(goal.poseMeters.getRotation().getDegrees());
        // ntGoalHolRot.setNumber(goal.holonomicRotation.getDegrees());
        return adjustedSpeeds;
    }

    public boolean isTrajectoryCompleted() {
        return timer.get() > m_chosenTrajectory.getTotalTimeSeconds();
    }

    public void robotPeriodic() {

    }
}
