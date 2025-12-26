package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoSqeuenceCommand extends SequentialCommandGroup {
    public AutoSqeuenceCommand(DriveSubsystem drive) {
        for (int i = 0; i < Constants.kField.kAutoTargets.length; i++) {
            addCommands(
                new GoToTargetCommand(
                    drive,
                    Constants.kField.kAutoTargets[i]
                )
            );
        }
    }
}