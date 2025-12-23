package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.system.plant.DCMotor;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DriveSubsystem extends SubsystemBase {

  private final SparkMax leftLeader =
      new SparkMax(Constants.kDriveCAN.kLeftLeader, SparkMax.MotorType.kBrushed);
  private final SparkMax leftFollower =
      new SparkMax(Constants.kDriveCAN.kLeftFollower, SparkMax.MotorType.kBrushed);
  private final SparkMax rightLeader =
      new SparkMax(Constants.kDriveCAN.kRightLeader, SparkMax.MotorType.kBrushed);
  private final SparkMax rightFollower =
      new SparkMax(Constants.kDriveCAN.kRightFollower, SparkMax.MotorType.kBrushed);

  private final DifferentialDrive drive =
      new DifferentialDrive(leftLeader, rightLeader);

  private final PIDController turnPID =
      new PIDController(Constants.kDrivePID.kTurnP,
                        Constants.kDrivePID.kTurnI,
                        Constants.kDrivePID.kTurnD);

  private final PIDController drivePID =
      new PIDController(Constants.kDrivePID.kDriveP,
                        Constants.kDrivePID.kDriveI,
                        Constants.kDrivePID.kDriveD);

  private final DifferentialDrivetrainSim sim =
      new DifferentialDrivetrainSim(
          DCMotor.getCIM(2),
          Constants.kDriveSim.kGearing,
          Constants.kDriveSim.kJkgMetersSq,
          Constants.kDriveSim.kMassKg,
          Constants.kDriveSim.kWheelRadius,
          Constants.kDriveSim.kTrackWidth,
          VecBuilder.fill(0.005,0.005,0.001,0.05,0.05,0.005,0.005)
      );

  private final Field2d field = new Field2d();

  public DriveSubsystem() {
    SparkMaxConfig follower = new SparkMaxConfig();
    follower.follow(leftLeader);
    leftFollower.configure(follower, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    follower.follow(rightLeader);
    rightFollower.configure(follower, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    Shuffleboard.getTab("Default")
        .add("Field View", field)
        .withSize(8,4);
  }

  public void arcade(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void tank(double lFwd, double rFwd) {
    drive.tankDrive(lFwd, rFwd);
  }

  public Pose2d getPose() {
    return sim.getPose();
  }
  
  boolean aligningFinalHeading = false;

  public boolean goToTarget(Pose2d target, boolean angleFromTarget) {
    Pose2d current = sim.getPose();

    double dx = target.getX() - current.getX();
    double dy = target.getY() - current.getY();
    double distance = Math.hypot(dx, dy);

    double robotHeading = current.getRotation().getRadians();
    double angleToTarget = Math.atan2(dy, dx);

    double headingError =
        -MathUtil.angleModulus(angleToTarget - robotHeading);

    double finalHeadingError =
        angleFromTarget
            ? -MathUtil.angleModulus(target.getRotation().getRadians() - robotHeading)
            : -MathUtil.angleModulus(
                  Math.atan2(
                      Constants.kField.kFieldMiddle.getY() - current.getY(),
                      Constants.kField.kFieldMiddle.getX() - current.getX()
                  ) - robotHeading
              );

    double turnOut, driveOut;

    if (distance > Constants.kDrivePID.kDistanceTolerance) {
      aligningFinalHeading = false;
      turnOut = turnPID.calculate(headingError, 0);
      driveOut = -drivePID.calculate(distance, 0);
    } else {
      if (!aligningFinalHeading) {
        turnPID.reset();
        System.out.println("PID Reset!");
        aligningFinalHeading = true;
      }
      turnOut = turnPID.calculate(finalHeadingError, 0);
      driveOut = 0;
    }
    

    drive.arcadeDrive(driveOut, turnOut);

    return distance < Constants.kDrivePID.kDistanceTolerance
        && Math.abs(finalHeadingError) < Math.toRadians(Constants.kDrivePID.kAngleToleranceDeg);
  }

  @Override
  public void simulationPeriodic() {
    double leftVolts  = leftLeader.get()  * RobotController.getBatteryVoltage();
    double rightVolts = rightLeader.get() * RobotController.getBatteryVoltage();
    sim.setInputs(leftVolts, rightVolts);
    sim.update(0.02);
    field.setRobotPose(sim.getPose());
  }
}
