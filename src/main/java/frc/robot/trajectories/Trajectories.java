package frc.robot.trajectories;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.commons.BreadPose2d;

// Trajectories class
public class Trajectories {

    public static void main(String[] args) {
        for (State state: bounce.getStates()) {
            System.out.println(state);
        }
    }

    // Salom Trajectory /* CONVERT TO METERS!!!! */
    public static final Trajectory salom = generateTrajectory(
        12/2.82, 
        4.0,
        List.of(
            new BreadPose2d(2.107, 13.117, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(4.931, 13.866, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(6.677, 20.776, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(13.96, 21.188, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(23.957, 19.954, Rotation2d.fromDegrees(-52.726), Rotation2d.fromDegrees(0)),
            new BreadPose2d(24.849, 11.979, Rotation2d.fromDegrees(-24.483), Rotation2d.fromDegrees(0)),
            new BreadPose2d(31.74, 12.353, Rotation2d.fromDegrees(93.729), Rotation2d.fromDegrees(0)),
            new BreadPose2d(30.49, 20.327, Rotation2d.fromDegrees(117.501), Rotation2d.fromDegrees(0)),
            new BreadPose2d(21.351, 20.29, Rotation2d.fromDegrees(168.029), Rotation2d.fromDegrees(0)),
            new BreadPose2d(20.065, 10.032, Rotation2d.fromDegrees(-84.371), Rotation2d.fromDegrees(0)),
            new BreadPose2d(11.246, 9.021, Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)),
            new BreadPose2d(1.928, 9.209, Rotation2d.fromDegrees(176.844), Rotation2d.fromDegrees(0)),
            new BreadPose2d(1.678, 18.118, Rotation2d.fromDegrees(90.181), Rotation2d.fromDegrees(0)),
            new BreadPose2d(-0.232, 19.915, Rotation2d.fromDegrees(-7.806), Rotation2d.fromDegrees(0)),
            new BreadPose2d(-3, 18.52, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(45))
        ),
        true
    );

    // Bounce Trajectory
    public static final Trajectory bounce = generateTrajectory(
        12/2.82, 
        4.0,
        List.of(
            new BreadPose2d(0.827, 5.778, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(1.644, 6.28, Rotation2d.fromDegrees(85), Rotation2d.fromDegrees(0)),
         new BreadPose2d(1.719, 7.177, Rotation2d.fromDegrees(81.892), Rotation2d.fromDegrees(0)),
    new BreadPose2d(1.828, 6.139, Rotation2d.fromDegrees(-84.709), Rotation2d.fromDegrees(0)),
    new BreadPose2d(2.742, 5.659, Rotation2d.fromDegrees(-40.392), Rotation2d.fromDegrees(0)),
    new BreadPose2d(3.199, 4.804, Rotation2d.fromDegrees(-87.724), Rotation2d.fromDegrees(0))
),

        true
    );

    // Method to generate a trajectory 
    private static Trajectory generateTrajectory(double maxVel, double maxAccel, List<BreadPose2d> waypoints, boolean clampedCubic) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        Trajectory trajectory;
        if (!clampedCubic) {
            trajectory = TrajectoryGenerator.generateTrajectory(waypoints.stream().map(point -> new Pose2d(point.getTranslation(), point.getRotation())).collect(Collectors.toList()), config);
        } else {
            trajectory = TrajectoryGenerator.generateTrajectory(
                waypoints.get(0), 
                IntStream.range(0, waypoints.size()).filter(i -> i > 0 && i < waypoints.size() - 1).mapToObj(i -> waypoints.get(i).getTranslation()).collect(Collectors.toList()), 
                waypoints.get(waypoints.size()-1), config
            );
        }
        int curr = 0;
        for (State state: trajectory.getStates()) {
            if (Math.abs(state.poseMeters.getX() - waypoints.get(curr + 1).getX()) < 0.001 && Math.abs(state.poseMeters.getY() - waypoints.get(curr + 1).getY()) < 0.001) {
                curr += 1;
            }
            state.poseMeters = new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), waypoints.get(curr).swerveRot);
        }
        return trajectory;
    }
    
}
