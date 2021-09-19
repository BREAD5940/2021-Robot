package frc.robot.autonomus;

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
import edu.wpi.first.wpilibj.util.Units;

/**
 * 
 * This class contains all the trajectory
 * components for the autonomus routines
 * 
 */
public class Trajectories {
    
    /* 8 PC auto part 1 */
    public static Trajectory EIGHT_PC_AUTO_P1 = generateTrajectory(
        2.0,
        2.0,
        List.of(
            new Pose2d(Units.feetToMeters(10.753), Units.feetToMeters(2.348), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(21.705), Units.feetToMeters(1.912), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(15.977), Units.feetToMeters(13.539), Rotation2d.fromDegrees(120.759)),
            new Pose2d(Units.feetToMeters(11.484), Units.feetToMeters(18.718), Rotation2d.fromDegrees(-178.954))
        ),
        true
    );

    /* 8 PC auto part 2 */
    public static Trajectory EIGHT_PC_AUTO_P2 = generateTrajectory(
        2.0, 
        2.0, 
        List.of(
            new Pose2d(Units.feetToMeters(11.484), Units.feetToMeters(17.718), Rotation2d.fromDegrees(1.821)),
            new Pose2d(Units.feetToMeters(17.167), Units.feetToMeters(23.334), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(23.445), Units.feetToMeters(24.614), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(27.253), Units.feetToMeters(24.738), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(11.068), Units.feetToMeters(22.024), Rotation2d.fromDegrees(-153.66))
        ),
        true
    );

    /* Method to generate a trajectory */
    private static Trajectory generateTrajectory(double maxVel, double maxAccel, List<Pose2d> waypoints, boolean clampedCubic, TrajectoryConstraint... constraints) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        for (TrajectoryConstraint c : constraints) {
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