package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.XboxController;

public class TeleopDefaultCommand extends Command {
    private DriveSubsystem m_driveSubsystem;
    private XboxController m_driverController;
    private SendableChooser<Integer> m_driveMode;
    
    public TeleopDefaultCommand(DriveSubsystem driveSubsystem, 
                                XboxController driverController,
                                SendableChooser<Integer> driveMode) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_driverController = driverController;
        this.m_driveMode = driveMode;
        addRequirements(driveSubsystem);
        
    }

    @Override
    public void execute() {
        double fwd = -m_driverController.getLeftY();
        double rot = -m_driverController.getRightX();
        double rFwd = -m_driverController.getRightY();

        if (m_driveMode.getSelected() == 0) {
            m_driveSubsystem.arcade(fwd, rot);
        } else {
            m_driveSubsystem.tank(fwd, rFwd);
    }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
