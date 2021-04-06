package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.commons.BreadUtil;

// Turret class
public class Turret extends SubsystemBase {

    // Variables
    /* Center to turret is 7.5 inches */
    private final DoubleSupplier defaultRefSupplier;
    public final double offset = 311.6;
    private final double initialPos;
    private final double startPos;
    private final CANSparkMax motor = new CANSparkMax(33, MotorType.kBrushless);
    public final DutyCycleEncoder encoder = new DutyCycleEncoder(1);
    private final PIDController pid = new PIDController(0.2, 0.0, 0.0001);
    private final PIDController visionPid = new PIDController(0.1, 0.0, 0.0001);
    private double reference = 0.0;
    private boolean vision = false;
    private TurretOutput mode = TurretOutput.None;

    // Constructor
    public Turret(DoubleSupplier defaultRefSupplier) {
        encoder.setDistancePerRotation(360.0);
        initialPos = encoder.getDistance();
        startPos = BreadUtil.angleTo180Range(initialPos - offset);  
        pid.setTolerance(2);
        visionPid.setTolerance(0.5);
        this.defaultRefSupplier = defaultRefSupplier;
    }

    // Method to get the distance of the encoder
    public double getDistance() {
        return -((encoder.getDistance() - initialPos) + startPos);
    }

    // Method to disable the turret
    public void disable() {
        mode = TurretOutput.None;
        reference = 0.0;
    }
    
    // Method to set the reference of the turret
    public void setReference(double reference, boolean vision) {
        this.vision = vision;
        mode = TurretOutput.Position;
        this.reference = BreadUtil.angleTo180Range(reference);
    }

    // Overload to set the reference of the turret using the default reference supplier
    public void setReference() {
        setReference(defaultRefSupplier.getAsDouble(), false);
    }

    // Method to check whether you are at the reference of the turret
    public boolean atReference() {
        return vision ? visionPid.atSetpoint() : pid.atSetpoint();
    }


    // In the periodic method of this subsystem set the turret based on the parameters
    @Override
    public void periodic() {
        if (mode == TurretOutput.Position) {
            double angle = getSafestPosition(reference, getDistance());
            double output = MathUtil.clamp(vision ? visionPid.calculate(getDistance(), angle) : pid.calculate(getDistance(), angle), -3, 3);
            motor.setVoltage(output);
        } else {
            motor.setVoltage(0.0);
        }
        SmartDashboard.putNumber("Turret Angle", getDistance());
        SmartDashboard.putNumber("Turret EncAngle", encoder.getDistance());
        SmartDashboard.putNumber("Turret Setpoint", reference);
    }

    // Private method to get the closest angle to the turret that is between the range of (-210, 210)
    private double getSafestPosition(double angleRef, double turretAngle) {
        if (angleRef > -90.0 && angleRef < 90.0) {
            return angleRef;
        } else {
            return angleRef < 0.0 ? BreadUtil.closer(angleRef, (angleRef + 360), turretAngle) 
            : BreadUtil.closer(angleRef, (angleRef - 360), turretAngle);
        }
    }

    // Turret output enum
    public enum TurretOutput {
        Position,
        None
    }
    
}
