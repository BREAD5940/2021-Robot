package frc.robot.autonomus.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomus.Trajectories;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.swerve.DriveSubsystem;


/* Eight PC Auto Command */
public class EightPCAuto extends SequentialCommandGroup {

    /* Required subsystems */
    SuperStructure superStructure;
    DriveSubsystem drive;

    /* Instantiate subsystems, add requirements, and add commands */
    EightPCAuto(SuperStructure superStructure, DriveSubsystem drive) {
        this.superStructure = superStructure;
        this.drive = drive;
        addRequirements(superStructure, drive);

        addCommands(
            drive.new TrajectoryFollowerCommand(Trajectories.EIGHT_PC_AUTO_P1, Trajectories.EIGHT_PC_AUTO_P1.sample(0.0).poseMeters.getRotation())
                .beforeStarting(() -> {
                    superStructure.intake.intake(0.8);
                    superStructure.spindexer.setVelocityReference(20.0);
                })
                .andThen(() -> { 
                    superStructure.intake.disable();
                    superStructure.spindexer.disable();
                }),
            superStructure.new ShootCommand(),
            drive.new TrajectoryFollowerCommand(Trajectories.EIGHT_PC_AUTO_P2, Trajectories.EIGHT_PC_AUTO_P2.sample(0.0).poseMeters.getRotation())
                .beforeStarting(() -> {
                    superStructure.intake.intake(0.8);
                    superStructure.spindexer.setVelocityReference(20.0);
                })
                .andThen(() -> { 
                    superStructure.intake.disable();
                    superStructure.spindexer.disable();
                }),
            superStructure.new ShootCommand()
        );
    }

}
