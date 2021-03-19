package frc.robot.debugging.characterizations;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.Output;

// Swerve characterization sharp
public class SwerveCharacterizationSharp extends CommandBase {

    // Variables
    private final Drive drive;

    // Constructor
    public SwerveCharacterizationSharp(Drive drive) {
        this.drive = drive;
    }   

    // Initialize method
    @Override
    public void initialize() {
        try {
            new FileWriter(Filesystem.getOperatingDirectory() + "/sharp.csv").close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        drive.setSpeeds(1.0, 0.0, 0.0, Output.PERCENT);
    }

    // Execute method
    @Override
    public void execute() {
        try {
            FileWriter w = new FileWriter(Filesystem.getOperatingDirectory() + "/sharp.csv", true);
            w.write(Timer.getFPGATimestamp() + ", " + RobotController.getBatteryVoltage() + ", " + drive.getOutputs()[0] + ", " + drive.getVelocities()[0] + "\n");
            w.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // End method
    @Override
    public void end(boolean interrupted) {
        drive.setSpeeds(0.0, 0.0, 0.0, Output.PERCENT);
    }
}
