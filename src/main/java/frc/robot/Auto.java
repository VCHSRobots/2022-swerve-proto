package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

public class Auto {

    String trajectoryJSON = "Paths/OneM.wpilib.json";
    Trajectory trajectory = new Trajectory();
    PathPlannerTrajectory pathPlannerTrajectory;

    PIDController xController = new PIDController(4, 0, 0);
    PIDController yController = new PIDController(4, 0, 0);
    static ProfiledPIDController thetaController = new ProfiledPIDController(4, 0, 0,
            new Constraints(SwerveDrive.kMaxAngularSpeed, SwerveDrive.kMaxAngularSpeed * 4));
    HolonomicDriveController controller = new HolonomicDriveController(xController, yController, thetaController);

    Timer timer = new Timer();

    public Auto() {
        pathPlannerTrajectory = PathPlanner.loadPath("square", 2, 3);
    }

    public void robotInit() {
        // This will load the file "Example Path.path" and generate it with a max
        // velocity of 8 m/s and a max acceleration of 5 m/s^2
        // PathPlannerTrajectory examplePath = PathPlanner.loadPath("line2meter", 1, 1);

        // // Sample the state of the path at 1.2 seconds
        // // To access PathPlanner specific information, such as holonomic rotation,
        // the
        // // state must be cast to a PathPlannerState
        // PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

        // // Print the holonomic rotation at the sampled time
        // System.out.println(exampleState.holonomicRotation.getDegrees());
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
        if (pathPlannerTrajectory == null) return;
        ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
        var goal = (PathPlannerState) pathPlannerTrajectory.sample(timer.get());
        if (timer.get() < pathPlannerTrajectory.getTotalTimeSeconds()) {

            adjustedSpeeds = controller.calculate(Robot.m_swerve.getPose2d(), goal,
                    goal.holonomicRotation);
            // System.out.println(Robot.m_swerve.getPose2d());
        } else {
            // adjustedSpeeds is initialized to zero, don't need to re assign
        }
        Robot.m_swerve.driveFromChassisSpeeds(adjustedSpeeds);

    }

}
