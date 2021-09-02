package frc.robot.trajectories;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import frc.robot.commons.BreadPose2d;

// Trajectories class
public class Trajectories {
    
    // Salom Trajectory
    public static final Trajectory salom = generateTrajectory(
        12/2.82, 
        4.0,
        List.of(
            new BreadPose2d(1.045, 5.847, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(2.101, 6.075, Rotation2d.fromDegrees(74.42), Rotation2d.fromDegrees(74.42)),
            new BreadPose2d(2.285, 7.474, Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)),
            new BreadPose2d(2.176, 5.157, Rotation2d.fromDegrees(-95.995), Rotation2d.fromDegrees(-95.995)),
            new BreadPose2d(3.276, 4.804, Rotation2d.fromDegrees(-1.581), Rotation2d.fromDegrees(-1.581)),
            new BreadPose2d(3.384, 3.195, Rotation2d.fromDegrees(-88.159), Rotation2d.fromDegrees(-88.159)),
            new BreadPose2d(4.505, 3.001, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(5.408, 3.788, Rotation2d.fromDegrees(93.408), Rotation2d.fromDegrees(93.408)),
            new BreadPose2d(5.234, 7.542, Rotation2d.fromDegrees(89.318), Rotation2d.fromDegrees(89.318)),
            new BreadPose2d(5.278, 6.219, Rotation2d.fromDegrees(-89.671), Rotation2d.fromDegrees(-89.671)),
            new BreadPose2d(5.354, 4.963, Rotation2d.fromDegrees(-95.497), Rotation2d.fromDegrees(-95.497)),
            new BreadPose2d(5.082, 3.149, Rotation2d.fromDegrees(-80.625), Rotation2d.fromDegrees(-80.625)),
            new BreadPose2d(7.182, 3.72, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
        ),                                  
        true
    );

    // Method to generate a trajectory 
    private static Trajectory generateTrajectory(double maxVel, double maxAccel, List<BreadPose2d> waypoints, boolean clampedCubic, TrajectoryConstraint... constraints) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        for (TrajectoryConstraint c: constraints) {
            config.addConstraint(c);
        }
        if (!clampedCubic) {
            return TrajectoryGenerator.generateTrajectory(waypoints.stream().map(point -> new Pose2d(point.getTranslation(), point.getRotation())).collect(Collectors.toList()), config);
        } else {
            List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
            for (int i=1; i<waypoints.size()-1; i++) {
                interiorPoints.add(waypoints.get(i).getTranslation());
            }
            return TrajectoryGenerator.generateTrajectory(waypoints.get(0), interiorPoints, waypoints.get(waypoints.size()-1), config);
        }
    }
    
}