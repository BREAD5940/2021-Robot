package frc.robot.interpolation;

// Shot parameter
public class ShotParameter {

    // Variables
    public final double hoodAngle;
    public final double rpm;
    public final double offset;

    // Constructor
    public ShotParameter(double hoodAngle, double rpm, double offset) {
        this.hoodAngle = hoodAngle;
        this.rpm = rpm;
        this.offset = offset;
    }   

    // Method equals
    public boolean equals(ShotParameter other) {
        return Math.abs(this.hoodAngle - other.hoodAngle) < 0.1 &&
        Math.abs(this.rpm - other.rpm) < 0.1 &&
        Math.abs(this.offset - other.offset) < 0.1;
    }

    // Method to interpolate
    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(hoodAngle, end.hoodAngle, t), 
            lerp(rpm, end.rpm, t), 
            lerp(offset, end.offset, t)
        );
    }

    // Method lerp
    private double lerp(double y2, double y1, double t) {
        return y1 + (t * (y2 - y1));
    }
 
}