package frc.robot.state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Interpolable;

public class Pose2d_4415 extends Pose2d {// implements Interpolable<Pose2d_4415> {
    public Pose2d_4415() {
        super();
    }

    public Pose2d_4415(Translation2d arg0, Rotation2d arg1) {
        super(arg0, arg1);
    }

    public Pose2d_4415(double arg0, double arg1, Rotation2d arg2) {
        super(arg0, arg1, arg2);
    }

    public Pose2d_4415(Pose2d other) {
        super(other.getTranslation(), other.getRotation());
    }

    // public Pose2d_4415 interpolate(Pose2d_4415 endValue, double t) {
    //     return this.interpolate(endValue, t);
    // }

    public Pose2d getPose2d() {
        return (Pose2d) this;
    }
}
