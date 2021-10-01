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
    
    /* 5 PC auto part 1 */
    public static Trajectory FIVE_PC_AUTO_LEFT_P1 = generateTrajectory(
        1.0,
        2.0,
        List.of(
            new Pose2d(Units.feetToMeters(10.747), Units.feetToMeters(6.716), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(19.961), Units.feetToMeters(6.042), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(21.493), Units.feetToMeters(1.273), Rotation2d.fromDegrees(-84.661))
        ),
        true
    );

    /* 5 PC auto part 2 */
    public static Trajectory FIVE_PC_AUTO_LEFT_P2 = generateTrajectory(
        2.5,
        3.0,
        List.of(
            new Pose2d(Units.feetToMeters(21.618), Units.feetToMeters(0.437), Rotation2d.fromDegrees(1.075)),
            new Pose2d(Units.feetToMeters(21.457), Units.feetToMeters(2.77), Rotation2d.fromDegrees(119.906)),
            new Pose2d(Units.feetToMeters(12.781), Units.feetToMeters(18.268), Rotation2d.fromDegrees(120.742)),
            new Pose2d(Units.feetToMeters(9.961), Units.feetToMeters(21.226), Rotation2d.fromDegrees(-173.254))
        ),
        true
    );

    /* 6 PC auto */
    public static Trajectory SIX_PC_AUTO_RIGHT = generateTrajectory(
        2.0, 
        2.5, 
        List.of(
            new Pose2d(Units.feetToMeters(10.961), Units.feetToMeters(21.353), Rotation2d.fromDegrees(-0.345)),
            new Pose2d(Units.feetToMeters(13.785), Units.feetToMeters(22.663), Rotation2d.fromDegrees(1.411)),
            new Pose2d(Units.feetToMeters(17.637), Units.feetToMeters(25.306), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(23.635), Units.feetToMeters(25.793), Rotation2d.fromDegrees(1.241)),
            new Pose2d(Units.feetToMeters(27.241), Units.feetToMeters(25.381), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(24.778), Units.feetToMeters(23.659), Rotation2d.fromDegrees(-156.04)),
            new Pose2d(Units.feetToMeters(12.139), Units.feetToMeters(19.354), Rotation2d.fromDegrees(-166.831))
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