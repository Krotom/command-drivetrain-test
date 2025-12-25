package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class GoToRadialTargetCommand extends Command {
    private final DriveSubsystem drive;
    private final XboxController controller;

    public GoToRadialTargetCommand(
            DriveSubsystem drive,
            XboxController controller
    ) {
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        Pose2d target = getRadialTarget();
        if (target != null) {
            drive.goToTarget(target, false);
            return;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private Pose2d getRadialTarget() {
    double x = -controller.getLeftX();
    double y = -controller.getLeftY();

    if (Math.hypot(x, y) < Constants.kOperatorConstants.kJoystickDeadband)
      return null;

    double angle = Math.toDegrees(Math.atan2(y, x)) + 180.0;
    int sector = (int)Math.floor((angle + 30.0) / 60.0) % 6;

    return Constants.kField.kRadialTargets[sector];
  }
}
