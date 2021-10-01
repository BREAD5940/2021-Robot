package frc.robot.autonomus.routines;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomus.Trajectories;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem.Output;

/* Five PC Auto Command */
public class SixPCAutoRight extends SequentialCommandGroup {
    
    /* Required subsystems */
    private final SuperStructure superStructure;
    private final DriveSubsystem drive;

    /* Instantiate subsystems, add requirements, and add commands */
    public SixPCAutoRight(SuperStructure superStructure, DriveSubsystem drive) {
        this.superStructure = superStructure;
        this.drive = drive;
        addRequirements(drive, superStructure);

        addCommands(
            superStructure.new ShootCommand(),
            superStructure.intake.new ExtendIntakeCommand(),
            drive.new TrajectoryFollowerCommand(Trajectories.SIX_PC_AUTO_RIGHT, drive.getPose().getRotation())
                .beforeStarting(() -> {
                    superStructure.intake.intake(0.8);
                    superStructure.spindexer.setVelocityReference(20.0);
                })
                .andThen(() -> { 
                    superStructure.intake.disable();
                    superStructure.spindexer.disable();
                }),
            superStructure.new ShootCommand().alongWith(new RunCommand(() -> drive.setSpeeds(0.0, 0.0, 0.0, Output.PERCENT, false)))
        );
    }

}
