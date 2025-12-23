package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer container;

  @Override
  public void robotInit() {
    container = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    Command auto = container.getAutonomousCommand();
    if (auto != null) {
      auto.schedule();
    } else {
      container.setAutonomousCommand();
      container.getAutonomousCommand().schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Command auto = container.getAutonomousCommand();
    if (auto == null || (auto != null && !auto.isScheduled())) {
      container.stopMotors();
    }
  }

  @Override
  public void teleopInit() {
    Command auto = container.getAutonomousCommand();
    if (auto != null) {
      auto.cancel();
    }
  }
}
