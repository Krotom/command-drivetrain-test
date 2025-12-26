package frc.robot;

import frc.robot.commands.AutoSqeuenceCommand;
import frc.robot.commands.GoToRadialTargetCommand;
import frc.robot.commands.GoToTargetCommand;
import frc.robot.commands.TeleopDefaultCommand;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.json.simple.parser.ParseException;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

public class RobotContainer {
    private Command autonomousCommand;
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final XboxController m_driverController =
        new XboxController(Constants.kOperatorConstants.kDriverControllerPort);

    private final SendableChooser<Integer> m_driveMode = new SendableChooser<>();

    RobotConfig robotConfig;

    public RobotContainer() {
      m_driveMode.setDefaultOption("Arcade", 0);
      m_driveMode.addOption("Tank", 1);

      Shuffleboard.getTab("Default")
          .add("Drive Mode", m_driveMode)
          .withWidget(BuiltInWidgets.kSplitButtonChooser)
          .withSize(2,1)
          .withPosition(8,0);

      m_driveSubsystem.setDefaultCommand(new TeleopDefaultCommand(m_driveSubsystem, m_driverController, m_driveMode));

      try {
        robotConfig = RobotConfig.fromGUISettings();
        AutoBuilder.configure(
            m_driveSubsystem::getPose,
            m_driveSubsystem::resetPose,
            m_driveSubsystem::getRobotRelativeSpeeds,
            m_driveSubsystem::driveRobotRelative,
            new PPLTVController(0.02),
            robotConfig,
            () -> false,
            m_driveSubsystem
        );
      } catch (IOException | ParseException e) {
          e.printStackTrace();
          throw new RuntimeException("Failed to load PathPlanner robot config, AutoBuilder not configured");
      }

      
      
      // we dont use swerve right now
      // autonomousCommand = AutoBuilder.buildAuto("MainAuto");

      configureBindings();
    }

    private void configureBindings() {
      new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
          .whileTrue(new RepeatCommand(new GoToRadialTargetCommand(m_driveSubsystem, m_driverController)));

      new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
          .onTrue(new GoToTargetCommand(m_driveSubsystem, Constants.kField.kCoralGet));
    }

    public void setAutonomousCommand() {
      autonomousCommand = new AutoSqeuenceCommand(m_driveSubsystem);
    }

    public Command getAutonomousCommand() {
      return autonomousCommand;
    }

    public void stopMotors() {
      m_driveSubsystem.arcade(0, 0);
    }
}
