package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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
    container.autonomousInit();
  }

  @Override
  public void autonomousPeriodic() {
    container.autonomousPeriodic();
  }

  @Override
  public void teleopInit() {
    container.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    container.teleopPeriodic();
  }
}
