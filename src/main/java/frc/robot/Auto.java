package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

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

    HolonomicDriveController controller;
    PIDController xController;
    PIDController yController;
    ProfiledPIDController thetaController;

    Timer timer;

    public Auto() {

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        xController = new PIDController(.4, 0, 0);
        yController = new PIDController(.4, 0, 0);
        thetaController = new ProfiledPIDController(.2, 0, 0,
                new Constraints(Robot.m_swerve.kMaxAngularSpeed, Robot.m_swerve.kMaxAngularSpeed * 4));

        controller = new HolonomicDriveController(xController, yController, thetaController);

        timer = new Timer();

    }

    public void startTimer() {
        timer.start();
    }

    public void resetTimer() {
        timer.reset();
    }

    public void autoUpdate() {
        Trajectory.State goal = trajectory.sample(timer.get());
        ChassisSpeeds adjustedSpeeds = controller.calculate(Robot.m_swerve.getPose2d(), goal,
                Robot.m_swerve.getGyroRotation2d());
        // System.out.println(Robot.m_swerve.getPose2d());
        Robot.m_swerve.driveFromChassisSpeeds(adjustedSpeeds);

    }

}
