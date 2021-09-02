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

    public static void main(String[] args) {
        System.out.println(aRed.getTotalTimeSeconds());
    }

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

    // Bounce Trajectory
    public static final Trajectory bounce = generateTrajectory(
        12/2.82,
        4.0,
        List.of(
            new BreadPose2d(1.045, 5.847, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(2.101, 6.075, Rotation2d.fromDegrees(74.42), Rotation2d.fromDegrees(74.42)),
            new BreadPose2d(2.285, 7.474, Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)),
            new BreadPose2d(2.22, 5.237, Rotation2d.fromDegrees(-95.995), Rotation2d.fromDegrees(-95.995)),
            new BreadPose2d(3.308, 4.827, Rotation2d.fromDegrees(-1.581), Rotation2d.fromDegrees(-1.581)),
            new BreadPose2d(3.395, 3.355, Rotation2d.fromDegrees(-92.281), Rotation2d.fromDegrees(-92.281)),
            new BreadPose2d(4.494, 3.104, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(5.332, 3.731, Rotation2d.fromDegrees(93.408), Rotation2d.fromDegrees(93.408)),
            new BreadPose2d(5.147, 7.873, Rotation2d.fromDegrees(89.318), Rotation2d.fromDegrees(89.318)),
            new BreadPose2d(4.984, 6.093, Rotation2d.fromDegrees(-88.993), Rotation2d.fromDegrees(-88.993)),
            new BreadPose2d(5.006, 4.998, Rotation2d.fromDegrees(-95.497), Rotation2d.fromDegrees(-95.497)),
            new BreadPose2d(5.06, 3.457, Rotation2d.fromDegrees(-79.3), Rotation2d.fromDegrees(-79.3)),
            new BreadPose2d(6.464, 3.412, Rotation2d.fromDegrees(-3.157), Rotation2d.fromDegrees(-3.157)),
            new BreadPose2d(7.846, 4.165, Rotation2d.fromDegrees(5.698), Rotation2d.fromDegrees(5.698)),
            new BreadPose2d(7.792, 8.432, Rotation2d.fromDegrees(93.817), Rotation2d.fromDegrees(93.817)),
            new BreadPose2d(7.792, 9.754, Rotation2d.fromDegrees(94), Rotation2d.fromDegrees(94)),
            new BreadPose2d(8.793, 7.759, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
        ),
        true
    );

    // Barrel Racing Trajectory
    public static final Trajectory barrel = generateTrajectory(
        12/2.82,
        4.0,
        List.of(
            new BreadPose2d(1.208, 2.024, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(4.887, 2.617, Rotation2d.fromDegrees(-31.66), Rotation2d.fromDegrees(-31.66)),
            new BreadPose2d(5.376, 0.148, Rotation2d.fromDegrees(-104.27), Rotation2d.fromDegrees(-104.27)),
            new BreadPose2d(2.198, 0.297, Rotation2d.fromDegrees(102.462), Rotation2d.fromDegrees(102.462)),
            new BreadPose2d(3.526, 2.693, Rotation2d.fromDegrees(-7.423), Rotation2d.fromDegrees(-7.423)),
            new BreadPose2d(7.226, 1.951, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(7.552, 4.758, Rotation2d.fromDegrees(124.487), Rotation2d.fromDegrees(124.487)),
            new BreadPose2d(4.647, 4.906, Rotation2d.fromDegrees(-149.898), Rotation2d.fromDegrees(-149.898)),
            new BreadPose2d(5.583, 1.677, Rotation2d.fromDegrees(-45.605), Rotation2d.fromDegrees(-45.605)),
            new BreadPose2d(10.022, 0.536, Rotation2d.fromDegrees(33.14), Rotation2d.fromDegrees(33.14)),
            new BreadPose2d(10.567, 2.864, Rotation2d.fromDegrees(96.512), Rotation2d.fromDegrees(96.512)),
            new BreadPose2d(8.695, 3.343, Rotation2d.fromDegrees(177.458), Rotation2d.fromDegrees(177.458)),
            new BreadPose2d(6.442, 3.268, Rotation2d.fromDegrees(179.87), Rotation2d.fromDegrees(179.87)),
            new BreadPose2d(-0.914, 2.804, Rotation2d.fromDegrees(178), Rotation2d.fromDegrees(178))
        ),                                                  
        true
    );

    // A Red Trajectory
    public static final Trajectory aRed = generateTrajectory(
        4.0, 
        4.0, 
        List.of(
            new BreadPose2d(0.686, 5.904, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(2.352, 5.926, Rotation2d.fromDegrees(1.683), Rotation2d.fromDegrees(1.683)),
            new BreadPose2d(3.112, 4.861, Rotation2d.fromDegrees(-6.784), Rotation2d.fromDegrees(-6.784)),
            new BreadPose2d(3.983, 4.906, Rotation2d.fromDegrees(-2.204), Rotation2d.fromDegrees(-2.204)),
            new BreadPose2d(4.081, 6.812, Rotation2d.fromDegrees(174.468), Rotation2d.fromDegrees(174.468)),
            new BreadPose2d(4.658, 8.272, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(12.5, 8.25, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
        ),                                   
        true
    );

    // B Red Trajectory
    public static final Trajectory bRed = generateTrajectory(
        4/2.82, 
        4.0, 
        List.of(
            new BreadPose2d(0.457, 7.067, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(2.243, 6.828, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(3.863, 4.69, Rotation2d.fromDegrees(-27.713), Rotation2d.fromDegrees(-27.713)),
            new BreadPose2d(4.766, 6.846, Rotation2d.fromDegrees(66.525), Rotation2d.fromDegrees(66.525)),
            new BreadPose2d(5.811, 8.033, Rotation2d.fromDegrees(67.767), Rotation2d.fromDegrees(67.767)),
            new BreadPose2d(11.927, 8.033, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
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