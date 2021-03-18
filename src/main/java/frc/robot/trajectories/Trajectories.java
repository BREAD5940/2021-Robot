package frc.robot.trajectories;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

// Trajectories class
public class Trajectories {

    // Test trajectory
    public static final Trajectory test = generateTrajectory(
        2.0, 
        1.0, 
        List.of(
            new Pose2d(new Translation2d(), new Rotation2d()),
            new Pose2d(new Translation2d(1, 1), new Rotation2d(0.0)),
            new Pose2d(new Translation2d(2, 2), new Rotation2d(0.0))
        ), 
        true   
    );

    // Method to generate a trajectory
    private static Trajectory generateTrajectory(double maxVel, double maxAccel, List<Pose2d> waypoints, boolean clampedCubic) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        if (!clampedCubic) {
            return TrajectoryGenerator.generateTrajectory(waypoints, config);
        }
        List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
        for (int i=1; i<waypoints.size()-1; i++) {
            interiorPoints.add(waypoints.get(i).getTranslation());
        }
        return TrajectoryGenerator.generateTrajectory(waypoints.get(0), interiorPoints, waypoints.get(waypoints.size()-1), config);
    }
    
}
