package frc.robot.autonomus.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomus.Trajectories;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.swerve.DriveSubsystem;

/* Five PC Auto Command */
public class FivePCAutoLeft extends SequentialCommandGroup {

    /* Instantiate subsystems, add requirements, and add commands */
    public FivePCAutoLeft(SuperStructure superStructure, DriveSubsystem drive) {
        addRequirements(superStructure, drive);

        addCommands(
            superStructure.intake.new ExtendIntakeCommand(),
            drive.new TrajectoryFollowerCommand(Trajectories.FIVE_PC_AUTO_LEFT_P1, drive.getPose().getRotation(), drive.getPose().getRotation())
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
