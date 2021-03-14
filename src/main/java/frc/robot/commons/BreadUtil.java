package frc.robot.commons;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

// Bread util class
public class BreadUtil {

    // Private constructor because this is a utility class
    private BreadUtil () {} 

    // Method to check whether a given value is between two given bounds
    public static boolean inBound(double val, double lowerBound, double higherBound, boolean inclusive) {
        return inclusive ? (lowerBound <= val && higherBound >= val) : (lowerBound < val && higherBound > val);
    }

    // Method to check whether a given value is at its reference, given a tolerance
    public static boolean atReference(double val, double reference, double tolerance, boolean inclusive) {
        return inclusive ? (Math.abs(reference - val) <= tolerance) : (Math.abs(reference - val) < tolerance); 
    }

    // Method to check whether a given "a" value or a given "b" value is closer to a given "c" value
    public static double closer(double a, double b, double c) {
        return Math.abs(a-c) < Math.abs(b-c) ? a : b;
    }

    // Method to return the optimal camera angle given the robot position, turret to robot center, and target position
    public static Rotation2d estimateCameraReference(Pose2d fieldToRobot, Transform2d turretToCenter, Translation2d fieldToTarget) {
        Pose2d fieldToTurret = fieldToRobot.plus(turretToCenter);
        Transform2d turretToTarget = new Transform2d(fieldToTurret, new Pose2d(fieldToTarget, new Rotation2d()));
        return new Rotation2d(turretToTarget.getX(), turretToTarget.getY());
    }

}
