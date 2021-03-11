package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

// Turret class
public class Turret extends SubsystemBase {

    // Variables
    public final double offset = 277.0;
    private final double initialPos;
    private final double startPos;
    private final CANSparkMax motor = new CANSparkMax(33, MotorType.kBrushless);
    public final DutyCycleEncoder encoder = new DutyCycleEncoder(1);
    private final PIDController pid = new PIDController(0.05, 0.0, 0.0);
    private double reference = 0.0;
    private boolean enabled = false;

    // Constructor
    public Turret() {
        encoder.setDistancePerRotation(360.0);
        initialPos = encoder.getDistance();
        startPos = getStart(initialPos, offset);
        pid.setTolerance(3);
    }

    // Method to get the distance of the encoder
    public double getDistance() {
        return -((encoder.getDistance() - initialPos) + startPos);
    }

    // Method to enable the turret
    public void enable(double reference) {
        setReference(reference);
        enabled = true;
    }

    // Method to disable the turret
    public void disable() {
        enabled = false;
    }
    
    // Method to set the reference of the turret
    public void setReference(double reference) {
        this.reference = reference % 360;
        if (this.reference > 180.0) this.reference = -360 + this.reference;
        if (this.reference < -180.0) this.reference = 360 + this.reference;
    }

    // Method to check whether you are at the reference of the turret
    public boolean atReference() {
        return pid.atSetpoint();
    }


    // In the periodic method of this subsystem set the turret based on the parameters
    @Override
    public void periodic() {
        if (enabled) {
            double angle = getSafestPosition(reference, getDistance());
            double output = MathUtil.clamp(pid.calculate(getDistance(), angle), -6, 6);
            motor.setVoltage(output);
        } else {
            motor.setVoltage(0.0);
        }
        SmartDashboard.putNumber("Turret Encoder", getDistance());
    }

    // Private method to get the closest angle to the turret that is between the range of (-210, 210)
    private double getSafestPosition(double angleRef, double turretAngle) {
        if (angleRef > -150.0 && angleRef < 150.0) {
            return angleRef;
        } else {
            return angleRef < 0.0 ? calcClosestTo(angleRef, (angleRef + 360), turretAngle) 
            : calcClosestTo(angleRef, (angleRef - 360), turretAngle);
        }
    }

    // Private method to calculate the closest number to another number
    private double calcClosestTo(double num1, double num2, double numRef) {
        return Math.abs(num1 - numRef) < Math.abs(num2 - numRef) ? num1 : num2;
    }

    // Private method to get the starting position of the turret as an angle between (-180, 180)
    private double getStart(double angle, double offset) {
        double returnAngle = (angle - offset) % 360;
        if (returnAngle > 180.0) returnAngle = -360 + returnAngle;
        if (returnAngle < -180.0) returnAngle = 360 + returnAngle;
        return returnAngle;
    }
    
}
