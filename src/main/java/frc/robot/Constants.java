package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class kOperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kJoystickDeadband = 0.25;
    public static final double kTriggerThreshold = 0.3;
  }

  public static final class kDriveCAN {
    public static final int kLeftLeader = 2;
    public static final int kLeftFollower = 3;
    public static final int kRightLeader = 0;
    public static final int kRightFollower = 1;
    public static final int kPigeonID = 4;
  }

  public static final class kDrive {
    public static final double kTurnP = 1.5;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.2;

    public static final double kDriveP = 1.0;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    public static final double kDistanceTolerance = 0.17;
    public static final double kAngleToleranceDeg = 1.0;
    public static final int kArrivedAtTargetCount = 3;

    public static final double kMaxSpeedMetersPerSecond = 3.0;
  }

  public static final class kDriveSim {
    public static final double kGearing = 10.71;
    public static final double kJkgMetersSq = 7.5;
    public static final double kMassKg = 20.0;
    public static final double kWheelRadius = Units.inchesToMeters(3);
    public static final double kTrackWidth = Units.inchesToMeters(28);
  }

  public static final class kField {
    public static final Pose2d kZeroZero = new Pose2d(0, 0, new Rotation2d());
    public static final Pose2d kFieldMiddle = new Pose2d(4.5, 4.0, new Rotation2d());
    public static final Pose2d kCoralGet = new Pose2d(1.1, 7.1, new Rotation2d());

    public static final Pose2d kLeftDown  = new Pose2d(3.7, 2.7, new Rotation2d());
    public static final Pose2d kLeftMid   = new Pose2d(3.0, 4.0, new Rotation2d());
    public static final Pose2d kLeftUp    = new Pose2d(3.8, 5.3, new Rotation2d());

    public static final Pose2d kRightDown = new Pose2d(5.3, 2.7, new Rotation2d());
    public static final Pose2d kRightMid  = new Pose2d(6.0, 4.0, new Rotation2d());
    public static final Pose2d kRightUp   = new Pose2d(5.4, 5.3, new Rotation2d());

    public static final Pose2d[] kRadialTargets = {
      kRightMid,
      kRightDown,
      kLeftDown,
      kLeftMid,
      kLeftUp,
      kRightUp
    };

    public static final Pose2d[] kAutoTargets = {
      kLeftDown, kCoralGet,
      kLeftMid,  kCoralGet,
      kLeftUp,   kCoralGet,
      kRightDown,kCoralGet,
      kRightMid, kCoralGet,
      kRightUp,  kZeroZero
    };
  }

  private Constants() {}
}
