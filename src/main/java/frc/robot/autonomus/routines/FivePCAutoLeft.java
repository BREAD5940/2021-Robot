package frc.robot.autonomus.routines;

import java.time.Instant;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomus.Trajectories;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem.Output;


/* Eight PC Auto Command */
public class FivePCAutoLeft extends SequentialCommandGroup {

    /* Required subsystems */
    SuperStructure superStructure;
    DriveSubsystem drive;

    /* Instantiate subsystems, add requirements, and add commands */
    public FivePCAutoLeft(SuperStructure superStructure, DriveSubsystem drive) {
        this.superStructure = superStructure;
        this.drive = drive;
        addRequirements(superStructure, drive);

        addCommands(
            superStructure.intake.new ExtendIntakeCommand(),
            drive.new TrajectoryFollowerCommand(Trajectories.FIVE_PC_AUTO_LEFT_P1, Rotation2d.fromDegrees(drive.getDistance()))
                .beforeStarting(() -> {
                    superStructure.intake.intake(0.8);
                    superStructure.spindexer.setVelocityReference(20.0);
                })
                .andThen(() -> { 
                    superStructure.intake.disable();
                    superStructure.spindexer.disable();
                })
            // drive.new TrajectoryFollowerCommand(Trajectories.FIVE_PC_AUTO_LEFT_P2, Rotation2d.fromDegrees(drive.getDistance()))
            //     .beforeStarting(() -> {
            //         superStructure.intake.intake(0.8);
            //         superStructure.spindexer.setVelocityReference(20.0);
            //     })
            //     .andThen(() -> { 
            //         superStructure.intake.disable();
            //         superStructure.spindexer.disable();
            //     }),
            // superStructure.new ShootCommand().alongWith(new RunCommand(() -> drive.setSpeeds(0.0, 0.0, 0.0, Output.PERCENT, false)))
        );
    }

}
