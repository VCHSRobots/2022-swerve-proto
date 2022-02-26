package frc.robot.state;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Interpolable;

public class Rotation2d_4415 extends Rotation2d {//implements Interpolable<Rotation2d_4415> {
    public Rotation2d_4415() {
        super();
    }

    public Rotation2d_4415(double radians) {
        super(radians);
    }

    public Rotation2d_4415(double x, double y) {
        super(x,y);
    }

    public Rotation2d_4415(Rotation2d other) {
        super(other.getRadians());
    }

    // public Rotation2d_4415 interpolate(Rotation2d_4415 endValue, double t) {
    //     return this.interpolate(endValue, t);
    // }

    public Rotation2d getRotation2d() {
        return (Rotation2d) this;
    }
}
