package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    /* Robot dimensions & Module Locations */
    public static final double baseWidth = Units.inchesToMeters(27.0);
    public static final double baseLength = Units.inchesToMeters(32.5);
    public static final double maxRobotVelocity = 12.0/2.82;
    public static final double centerToCorner = Math.sqrt((baseWidth * baseWidth) + (baseLength * baseLength)) / 2.0;
    public static final Translation2d flLocation = new Translation2d(baseLength / 2.0, baseWidth / 2.0);
    public static final Translation2d frLocation = new Translation2d(baseLength / 2.0, -baseWidth / 2.0);
    public static final Translation2d blLocation = new Translation2d(-baseLength / 2.0, baseWidth / 2.0);
    public static final Translation2d brLocation = new Translation2d(-baseLength / 2.0, -baseWidth / 2.0);

    /* Swerve module id's and offsets */
    public static final int flDriveID = 26;
    public static final int frDriveID = 42;
    public static final int blDriveID = 28;
    public static final int brDriveID = 29;
    public static final int flTurnID = 25;
    public static final int frTurnID = 41;
    public static final int blTurnID = 27;
    public static final int brTurnID = 30;
    public static final int flAnalogID = 3;
    public static final int frAnalogID = 0;
    public static final int blAnalogID = 2;
    public static final int brAnalogID = 1;
    public static final double flOffset = Units.degreesToRadians(43.5);
    public static final double frOffset = Units.degreesToRadians(44.1);
    public static final double blOffset = Units.degreesToRadians(78.8);
    public static final double brOffset = Units.degreesToRadians(292.7);
}
