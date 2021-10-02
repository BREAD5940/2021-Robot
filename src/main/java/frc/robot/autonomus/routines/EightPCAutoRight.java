package frc.robot.autonomus.routines;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomus.Trajectories;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class EightPCAutoRight extends SequentialCommandGroup {
    
    public EightPCAutoRight(SuperStructure superStructure, DriveSubsystem drive) {
        addRequirements(superStructure, drive);

        addCommands(
            superStructure.intake.new ExtendIntakeCommand(),
            drive.new TrajectoryFollowerCommand(Trajectories.EIGHT_PC_AUTO_RIGHT_P1, drive.getPose().getRotation(), Rotation2d.fromDegrees(120.0))
                .beforeStarting(() -> {
                    superStructure.intake.intake(0.8);
                    superStructure.spindexer.setVelocityReference(20.0);
                })
                .andThen(() -> { 
                    superStructure.intake.disable();
                    superStructure.spindexer.disable();
                })
        );
    }

}
