package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class GoToTargetCommand extends Command {

  private final DriveSubsystem drive;
  private final Pose2d target;
  private final boolean angleFromTarget;

  public GoToTargetCommand(
        DriveSubsystem drive,
        Pose2d target) {
    this.drive = drive;
    this.target = target;
    this.angleFromTarget = false;
    addRequirements(drive);
  }

  public GoToTargetCommand(
      DriveSubsystem drive,
      Pose2d target,
      boolean angleFromTarget
  ) {
    this.drive = drive;
    this.target = target;
    this.angleFromTarget = angleFromTarget;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    drive.goToTarget(target, angleFromTarget);
  }

  @Override
  public boolean isFinished() {
    return drive.goToTarget(target, angleFromTarget);
  }

  @Override
  public void end(boolean interrupted) {
    drive.arcade(0, 0);
  }
}
