package frc.robot.trajectories;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commons.BreadPose2d;

// Trajectories class
public class Trajectories {

    public static void main(String[] args) {
        for (State state : bounce.getStates()) {
            System.out.println(state);
        }
    }

    // Salom Trajectory
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
        true, 
        true
    );

    // Bounce Trajectory
    public static final Trajectory bounce = generateTrajectory(
        12.0/2.82, 
        4.0,
        List.of(
            new BreadPose2d(0.5, 17.347, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new BreadPose2d(4.252, 18.208, Rotation2d.fromDegrees(88.751), Rotation2d.fromDegrees(0)),
            new BreadPose2d(4.856, 24.146, Rotation2d.fromDegrees(89.817), Rotation2d.fromDegrees(0))
        ),
        true, 
        true
    );

    // Method to generate a trajectory 
    private static Trajectory generateTrajectory(double maxVel, double maxAccel, List<BreadPose2d> waypoints, boolean clampedCubic, boolean feet, TrajectoryConstraint... constraints) {
        List<Pose2d> transformedWaypoints = new ArrayList<>();
        for (int i=0; i<waypoints.size(); i++) {
            transformedWaypoints.add(feet ? new BreadPose2d(
                Units.feetToMeters(waypoints.get(i).getX()), 
                Units.feetToMeters(waypoints.get(i).getY()),
                waypoints.get(i).getRotation(),
                waypoints.get(i).swerveRot
            ) : waypoints.get(i));
        }
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        for (TrajectoryConstraint constraint : constraints) {
            config.addConstraint(constraint);
        }
        if (!clampedCubic) {
            int currWaypoint = 0;
            List<State> states = TrajectoryGenerator.generateTrajectory(transformedWaypoints, config).getStates();
            List<State> newStates = new ArrayList<>();
            for (State state : states) {
                if (Math.abs(state.poseMeters.getX() - transformedWaypoints.get(currWaypoint + 1).getX()) < 0.0001 && Math.abs(state.poseMeters.getY() - transformedWaypoints.get(currWaypoint + 1).getY()) < 0.0001) {
                    currWaypoint += 1;
                }
                newStates.add(new State(
                    state.timeSeconds,
                    state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq,
                    new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), currWaypoint >= waypoints.size() ? waypoints.get(currWaypoint - 1).swerveRot : waypoints.get(currWaypoint).swerveRot),
                    state.curvatureRadPerMeter
                ));
            }
            return new Trajectory(newStates);
        }
        List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
        for (int i=1; i<transformedWaypoints.size()-1; i++) {
            interiorPoints.add(transformedWaypoints.get(i).getTranslation());
        }
        int currWaypoint = 0;
        List<State> states = TrajectoryGenerator.generateTrajectory(transformedWaypoints.get(0), interiorPoints, transformedWaypoints.get(transformedWaypoints.size()-1), config).getStates();
        List<State> newStates = new ArrayList<>();
        for (State state : states) {
            if (Math.abs(state.poseMeters.getX() - transformedWaypoints.get(currWaypoint).getX()) < 0.0001 && Math.abs(state.poseMeters.getY() - transformedWaypoints.get(currWaypoint).getY()) < 0.0001) {
                currWaypoint += 1;
            }
            newStates.add(new State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), currWaypoint >= waypoints.size() ? waypoints.get(currWaypoint - 1).swerveRot : waypoints.get(currWaypoint).swerveRot),
                state.curvatureRadPerMeter
            ));
        }
        return new Trajectory(newStates);
    }
    
}
