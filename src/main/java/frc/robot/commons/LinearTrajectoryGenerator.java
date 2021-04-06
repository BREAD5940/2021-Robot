package frc.robot.commons;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

// Linear trajectory generator
public class LinearTrajectoryGenerator {
    
    /* Private constructor because this is a utility class */
    private LinearTrajectoryGenerator() {}

    // Method to generate a trajectory
    public static LinearTrajectory generateTrajectory(List<Pose2d> waypoints, double maxVelocity, double maxAcceleration) {
        new SequentialCommandGroup();
        List<Trajectory> lines = new ArrayList<>();
        Pose2d currentPose = waypoints.get(0);
        for (int i = 1; i < waypoints.size(); i++) {
            Pose2d nextPose = waypoints.get(i);
            double xOffset = nextPose.getX() - currentPose.getX();
            double yOffset = nextPose.getY() - currentPose.getY();
            double angle = Math.atan2(yOffset, xOffset);
            TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
            config.setEndVelocity(maxVelocity);
            if (lines.size() > 0) {
                Trajectory previousLine = lines.get(lines.size() - 1);
                if (previousLine.getStates().size() > 0) {
                    config.setStartVelocity(previousLine.getStates().get(previousLine.getStates().size() - 1).velocityMetersPerSecond);
                }
            }
            Trajectory line  = TrajectoryGenerator.generateTrajectory(
                new Pose2d(currentPose.getTranslation(), new Rotation2d(angle)), 
                List.of(), 
                new Pose2d(nextPose.getTranslation(), new Rotation2d(angle)), 
                config
            );      
            List<State> newStates = new ArrayList<>();
            for (State state : line.getStates()) {
                State newState = new State(
                    state.timeSeconds, 
                    state.velocityMetersPerSecond, 
                    state.accelerationMetersPerSecondSq, 
                    new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), Rotation2d.fromDegrees(nextPose.getRotation().getDegrees())), 
                    state.curvatureRadPerMeter
                );
                newStates.add(newState);
            }
            line = new Trajectory(newStates);
            lines.add(line);
            currentPose = nextPose;
        }

        return new LinearTrajectory(lines);

    }
    
}
