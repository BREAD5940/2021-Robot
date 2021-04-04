package frc.robot.commons;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class BreadPose2d extends Pose2d {

    public final Rotation2d swerveRot;

    public BreadPose2d(double x, double y, Rotation2d rot, Rotation2d swerveRot) {
        super(x, y, rot);
        this.swerveRot = swerveRot;
    }
}