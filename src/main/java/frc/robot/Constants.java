package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    // Robot dimensions & Module Locations
    public static final double baseWidth = Units.inchesToMeters(27.0);
    public static final double baseLength = Units.inchesToMeters(32.5);
    public static final double maxRobotVelocity = 12.0/2.82;
    public static final double centerToCorner = Math.sqrt((baseWidth * baseWidth) + (baseLength * baseLength)) / 2.0;
    public static final Translation2d flLocation = new Translation2d(baseLength / 2.0, baseWidth / 2.0);
    public static final Translation2d frLocation = new Translation2d(baseLength / 2.0, -baseWidth / 2.0);
    public static final Translation2d blLocation = new Translation2d(-baseLength / 2.0, baseWidth / 2.0);
    public static final Translation2d brLocation = new Translation2d(-baseLength / 2.0, -baseWidth / 2.0);
}
