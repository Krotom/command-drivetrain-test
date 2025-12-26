package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.system.plant.DCMotor;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DriveSubsystem extends SubsystemBase {

  /* ---------------- Hardware ---------------- */

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

  private final Pigeon2 gyro =
      new Pigeon2(Constants.kDriveCAN.kPigeonID);

  private final edu.wpi.first.math.controller.PIDController turnPID =
      new edu.wpi.first.math.controller.PIDController(
          Constants.kDrive.kTurnP,
          Constants.kDrive.kTurnI,
          Constants.kDrive.kTurnD
      );

  private final edu.wpi.first.math.controller.PIDController drivePID =
      new edu.wpi.first.math.controller.PIDController(
          Constants.kDrive.kDriveP,
          Constants.kDrive.kDriveI,
          Constants.kDrive.kDriveD
      );

  /* ---------------- Kinematics & Odometry ---------------- */

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.kDriveSim.kTrackWidth);

  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(
        getHeading(),
        0.0,
        0.0
      );

  /* ---------------- Simulation ---------------- */

  private final DifferentialDrivetrainSim sim =
      new DifferentialDrivetrainSim(
          DCMotor.getCIM(2),
          Constants.kDriveSim.kGearing,
          Constants.kDriveSim.kJkgMetersSq,
          Constants.kDriveSim.kMassKg,
          Constants.kDriveSim.kWheelRadius,
          Constants.kDriveSim.kTrackWidth,
          VecBuilder.fill(0.005, 0.005, 0.001, 0.05, 0.05, 0.005, 0.005)
      );

  private final Field2d field = new Field2d();

  /* ---------------- Constructor ---------------- */

  public DriveSubsystem() {
    SparkMaxConfig follower = new SparkMaxConfig();
    gyro.getGravityVectorZ();
    follower.follow(leftLeader);
    leftFollower.configure(
        follower,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
    );

    follower.follow(rightLeader);
    rightFollower.configure(
        follower,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
    );

    gyro.reset();

    Shuffleboard.getTab("Default")
        .add("Field", field);
  }

  /* ---------------- Pose & Odometry ---------------- */

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        getHeading(),
        sim.getLeftPositionMeters(),
        sim.getRightPositionMeters(),
        pose
    );
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  /* ---------------- PathPlanner hooks ---------------- */

  public ChassisSpeeds getRobotRelativeSpeeds() {
    var wheelSpeeds =
        new DifferentialDriveWheelSpeeds(
            sim.getLeftVelocityMetersPerSecond(),
            sim.getRightVelocityMetersPerSecond()
        );

    return kinematics.toChassisSpeeds(wheelSpeeds);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(Constants.kDrive.kMaxSpeedMetersPerSecond);

    double leftVolts =
        wheelSpeeds.leftMetersPerSecond
            / Constants.kDrive.kMaxSpeedMetersPerSecond
            * RobotController.getBatteryVoltage();

    double rightVolts =
        wheelSpeeds.rightMetersPerSecond
            / Constants.kDrive.kMaxSpeedMetersPerSecond
            * RobotController.getBatteryVoltage();

    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  /* ---------------- Manual Driving ---------------- */

  public void arcade(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void tank(double l, double r) {
    drive.tankDrive(l, r);
  }

  /* ---------------- Periodic ---------------- */

  @Override
  public void periodic() {
    odometry.update(
        getHeading(),
        sim.getLeftPositionMeters(),
        sim.getRightPositionMeters()
    );

    field.setRobotPose(getPose());
  }

  boolean aligningFinalHeading = false;
  double distance;
  double finalHeadingError;

  public void goToTarget(Pose2d target, boolean angleFromTarget) {
    Pose2d current = getPose();
    double dx = target.getX() - current.getX();
    double dy = target.getY() - current.getY();

    distance = Math.hypot(dx, dy);

    double robotHeading = current.getRotation().getRadians();

    double angleToTarget = Math.atan2(dy, dx);

    double headingError =
        -MathUtil.angleModulus(angleToTarget - robotHeading);

    finalHeadingError =
        angleFromTarget
            ? -MathUtil.angleModulus(target.getRotation().getRadians() - robotHeading)
            : -MathUtil.angleModulus(
                  Math.atan2(
                      Constants.kField.kFieldMiddle.getY() - current.getY(),
                      Constants.kField.kFieldMiddle.getX() - current.getX()
                  ) - robotHeading
              );

    double turnOut, driveOut;

    if (distance > Constants.kDrive.kDistanceTolerance) {
      aligningFinalHeading = false;
      turnOut = turnPID.calculate(headingError, 0);
      driveOut = -drivePID.calculate(distance, 0);
    } else {
      if (!aligningFinalHeading) {
        turnPID.reset();
        drivePID.reset();
        System.out.println("PID Reset!");
        aligningFinalHeading = true;
      }
      turnOut = turnPID.calculate(finalHeadingError, 0);
      driveOut = 0;
    }

    drive.arcadeDrive(driveOut, turnOut);
  }

  public boolean atTarget() {
    return distance < Constants.kDrive.kDistanceTolerance
        && Math.abs(finalHeadingError) < Math.toRadians(Constants.kDrive.kAngleToleranceDeg);
  }

  @Override
  public void simulationPeriodic() {
    double leftVolts = leftLeader.get() * RobotController.getBatteryVoltage();
    double rightVolts = rightLeader.get() * RobotController.getBatteryVoltage();

    sim.setInputs(leftVolts, rightVolts);
    sim.update(0.02);

    gyro.setYaw(sim.getHeading().getDegrees());
  }
}
