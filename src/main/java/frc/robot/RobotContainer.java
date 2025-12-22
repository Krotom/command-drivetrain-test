package frc.robot;

import frc.robot.commands.AutoSqeuenceCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private Command autonomousCommand;
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final XboxController m_driverController =
      new XboxController(Constants.kOperatorConstants.kDriverControllerPort);

  private final SendableChooser<Integer> m_driveMode = new SendableChooser<>();

  public RobotContainer() {
    m_driveMode.setDefaultOption("Arcade", 0);
    m_driveMode.addOption("Tank", 1);

    Shuffleboard.getTab("Default")
        .add("Drive Mode", m_driveMode)
        .withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withSize(2,1)
        .withPosition(8,0);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autonomousCommand;
  }

  public void setAutonomousCommand(Command command) {
    autonomousCommand = command;
  }

  public void autonomousInit() {
    Command auto = getAutonomousCommand();
    if (auto != null) {
      auto.schedule();
    } else {
      setAutonomousCommand(new AutoSqeuenceCommand(m_driveSubsystem));
      getAutonomousCommand().schedule();
    }
  }

  public void autonomousPeriodic() {
    Command auto = getAutonomousCommand();
    if (auto == null || (auto != null && !auto.isScheduled())) {
      m_driveSubsystem.arcade(0, 0);
    }
  }

  public void teleopInit() {
    Command auto = getAutonomousCommand();
    if (auto != null) {
      auto.cancel();
    }
  }

  public void teleopPeriodic() {
    if (m_driverController.getLeftBumperButton()) {
      Pose2d target = getRadialTarget();
      if (target != null) {
        m_driveSubsystem.goToTarget(target, false);
        return;
      }
    }

    if (m_driverController.getRawAxis(2) > Constants.kOperatorConstants.kTriggerThreshold) {
      m_driveSubsystem.goToTarget(Constants.kField.kCoralGet, false);
      return;
    }

    double fwd = -m_driverController.getLeftY();
    double rot = -m_driverController.getRightX();
    double rFwd = -m_driverController.getRightY();

    if (m_driveMode.getSelected() == 0) {
      m_driveSubsystem.arcade(fwd, rot);
    } else {
      m_driveSubsystem.tank(fwd, rFwd);
    }
  }

  private Pose2d getRadialTarget() {
    double x = -m_driverController.getLeftX();
    double y = -m_driverController.getLeftY();

    if (Math.hypot(x, y) < Constants.kOperatorConstants.kJoystickDeadband)
      return null;

    double angle = Math.toDegrees(Math.atan2(y, x)) + 180.0;
    int sector = (int)Math.floor((angle + 30.0) / 60.0) % 6;

    return Constants.kField.kRadialTargets[sector];
  }
}
