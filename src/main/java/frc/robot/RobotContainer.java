package frc.robot;

import frc.robot.commands.AutoSqeuenceCommand;
import frc.robot.commands.GoToTargetCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  private final DriveSubsystem drive = new DriveSubsystem();
  private final XboxController controller =
      new XboxController(Constants.kOperatorConstants.kDriverControllerPort);

  private final SendableChooser<Integer> driveMode = new SendableChooser<>();

  public RobotContainer() {
    driveMode.setDefaultOption("Arcade", 1);
    driveMode.addOption("Tank", 0);

    Shuffleboard.getTab("Default")
        .add("Drive Mode", driveMode)
        .withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withSize(2,1)
        .withPosition(8,0);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return new AutoSqeuenceCommand(drive);
  }

  public void autonomousInit() {
    Command auto = getAutonomousCommand();
    if (auto != null) {
      auto.schedule();
    }
  }

  public void autonomousPeriodic() {
    Command auto = getAutonomousCommand();
    if (auto != null && !auto.isScheduled()) {
      drive.arcade(0, 0);
    }
  }

  public void teleopInit() {
    Command auto = getAutonomousCommand();
    if (auto != null) {
      auto.cancel();
    }
  }

  public void teleopPeriodic() {
    if (controller.getLeftBumperButton()) {
      Pose2d target = getRadialTarget();
      if (target != null) {
        new GoToTargetCommand(drive, target, false).schedule();
        return;
      }
    }

    if (controller.getRawAxis(2) > Constants.kOperatorConstants.kTriggerThreshold) {
      new GoToTargetCommand(drive, Constants.kField.kCoralGet, false).schedule();
      return;
    }

    double fwd = -controller.getLeftY();
    double rot = -controller.getRightX();
    drive.arcade(fwd, rot);
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
