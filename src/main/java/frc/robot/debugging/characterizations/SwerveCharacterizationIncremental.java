package frc.robot.debugging.characterizations;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.Output;

// Swerve characterization incrementla
public class SwerveCharacterizationIncremental extends CommandBase {

    // Variables
    private final Drive drive;
    private double output;

    // Constructor
    public SwerveCharacterizationIncremental(Drive drive) {
        this.drive = drive;
    }

    // Initialize method
    @Override
    public void initialize() {
        try {
            new FileWriter(Filesystem.getOperatingDirectory() + "/incremental.csv").close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        output = 0.0;
    }

    // Execute method
    @Override
    public void execute() {
        try {
            FileWriter w = new FileWriter(Filesystem.getOperatingDirectory() + "/incremental.csv", true);
            w.write(Timer.getFPGATimestamp() + ", " + RobotController.getBatteryVoltage() + ", " + drive.getOutputs()[0] + ", " + drive.getVelocities()[0] + "\n");
            w.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        drive.setSpeeds(output, 0.0, 0.0, Output.PERCENT);
        output += 0.01;
    }

    // End method
    @Override
    public void end(boolean interrupted) {
        
    }
    
}
