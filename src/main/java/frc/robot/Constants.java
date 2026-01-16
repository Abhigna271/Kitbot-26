package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.littletonUtils.GeomUtil;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.lib.littletonUtils.SwerveSetpointGenerator.ModuleLimits;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import lombok.RequiredArgsConstructor;

public final class Constants {
  public static final boolean kTuningMode = true;

  public static final Mode kRealMode = Mode.REAL;
  public static final Mode kSimMode = Mode.SIM;
  public static final Mode kCurrentMode = RobotBase.isReal() ? kRealMode : kSimMode;

  public static final boolean kUsePhoenixDiagnosticServer = false;

  // set to false to disable the base refresh manager
  public static final boolean kUseBaseRefreshManager = false;

  /**
   * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
   * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
   * "replay" (log replay from a file).
   */
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running on the proto board */
    PROTO,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final boolean kUseComponents = true;

  // set to false to disable alerts
  public static final boolean kUseAlerts = true && kCurrentMode != Mode.SIM;

  public static final class DriveConstants {
    public static final double kMaxLinearSpeed = 4.5; // meters per second
    public static final double kMaxDriveToPointSpeed = 3.6;
    public static final double kMaxMeshedSpeed = 4.5;
    public static final double kMaxLinearAcceleration = 3.0; // meters per second squared
    public static final double kTrackWidthX = Units.inchesToMeters(22.75);
    public static final double kTrackWidthY = Units.inchesToMeters(22.75);
    public static final double kDriveBaseRadius =
        Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
    public static final double kMaxAngularSpeed = kMaxLinearSpeed / kDriveBaseRadius;
    public static final double kMaxAngularAcceleration = kMaxLinearAcceleration / kDriveBaseRadius;
    public static final LoggedTunableNumber kTeleopRotationSpeed =
        new LoggedTunableNumber("Teleop Rotation Speed", 10.0);
    public static final double kVisionSpeedConstantK = 0.5;

    // the exponent to raise the input to
    // ex 1.0 is linear, 2.0 is squared, etc
    public static final LoggedTunableNumber kDriveControlsExponent =
        new LoggedTunableNumber("Drive Control Mode", 2.0);

    public static final Translation2d[] kModuleTranslations =
        new Translation2d[] {
          new Translation2d(kTrackWidthX / 2.0, kTrackWidthY / 2.0),
          new Translation2d(kTrackWidthX / 2.0, -kTrackWidthY / 2.0),
          new Translation2d(-kTrackWidthX / 2.0, kTrackWidthY / 2.0),
          new Translation2d(-kTrackWidthX / 2.0, -kTrackWidthY / 2.0)
        };

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    public static final ModuleLimits kModuleLimitsFree =
        new ModuleLimits(kMaxLinearSpeed, kMaxAngularSpeed, Units.degreesToRadians(1080.0));

    public static final double kWheelRadius = Units.inchesToMeters(1.917);
    public static final double kOdometryFrequency = 250.0;

    public static final double kDriveGearRatio = 6.03 / 1.;
    public static final double kTurnGearRatio = 26.09 / 1;

    // Simulation constants
    public static final double kDriveSimGearRatio = kDriveGearRatio;
    public static final double kDriveSimMOI = 0.025;
    public static final double kTurnSimGearRatio = kTurnGearRatio;
    public static final double kTurnSimMOI = 0.004;

    // universal reversals for drive (aka the big negative sign)
    // Tommy - if you find yourself needing to enable either of these, most likely your controls are
    // wrong or your modules are ID'd wrong
    public static final boolean kRealReversed = false;
    public static final boolean kSimReversed = false;

    public static final LoggedTunableNumber kDriveToPointP =
        new LoggedTunableNumber("DriveToPoint P", 3.2);
    public static final LoggedTunableNumber kDriveToPointI =
        new LoggedTunableNumber("DriveToPoint I", 0.0);
    public static final LoggedTunableNumber kDriveToPointD =
        new LoggedTunableNumber("DriveToPoint D", 0.18);

    public static final LoggedTunableNumber kDriveToPointAutoP =
        new LoggedTunableNumber("DriveToPoint Auto P", 3.0);
    public static final LoggedTunableNumber kDriveToPointAutoI =
        new LoggedTunableNumber("DriveToPoint Auto I", 0.0);
    public static final LoggedTunableNumber kDriveToPointAutoD =
        new LoggedTunableNumber("DriveToPoint Auto D", 0.12);

    public static final LoggedTunableNumber kDriveToPointHeadingP =
        new LoggedTunableNumber("DriveToPoint Heading P", 4.0);
    public static final LoggedTunableNumber kDriveToPointHeadingI =
        new LoggedTunableNumber("DriveToPoint Heading I", 0.0);
    public static final LoggedTunableNumber kDriveToPointHeadingD =
        new LoggedTunableNumber("DriveToPoint Heading D", 0.05);

    public static final LoggedTunableNumber kDriveToPointMaxVelocity =
        new LoggedTunableNumber("DriveToPoint Max Velocity", 3.0);
    public static final LoggedTunableNumber kDriveToPointMaxAcceleration =
        new LoggedTunableNumber("DriveToPoint Max Acceleration", 4.0);
    public static final LoggedTunableNumber kDriveToPointMaxDeceleration =
        new LoggedTunableNumber("DriveToPoint Max Deceleration", 3.0);

    public static final LoggedTunableNumber kMeshedXYP =
        new LoggedTunableNumber("Drive Meshed XY P", 3.0);
    public static final LoggedTunableNumber kMeshedXYD =
        new LoggedTunableNumber("Drive Meshed XY D", 0.12);
    public static final LoggedTunableNumber kMeshedThetaP =
        new LoggedTunableNumber("Drive Meshed Theta P", 3.0);
    public static final LoggedTunableNumber kMeshedThetaD =
        new LoggedTunableNumber("Drive Meshed Theta D", 0.0);
    public static final LoggedTunableNumber kDebounceAmount =
        new LoggedTunableNumber("Meshed Drive Debounce", 0.1);
    public static final LoggedTunableNumber kMeshDrivePriority =
        new LoggedTunableNumber("Meshed Drive Priority", 0.3);

    public static final LoggedTunableNumber kAutoscoreDeployDistance =
        new LoggedTunableNumber("Autoscore Deploy Distance", 5.0);
    public static final LoggedTunableNumber kAutoscoreOuttakeDistance =
        new LoggedTunableNumber("Autoscore Outtake Distance", 1.25);
    public static final LoggedTunableNumber kAutoscoreL1OuttakeDistance =
        new LoggedTunableNumber("Autoscore L1 Outtake Distance", 18.0);
    public static final LoggedTunableNumber kBargeScoreThrowDistance =
        new LoggedTunableNumber("Barge Score Throw Distance", 10.0);
    public static final LoggedTunableNumber kLoaderStationTimeout =
        new LoggedTunableNumber("Loader Station Timeout", 0.25);
    public static final LoggedTunableNumber kAutoAutoscoreTimeout =
        new LoggedTunableNumber("Auto Autoscore Timeout", 2.0);

    // radians per second squared to be considered slipping
    public static final LoggedTunableNumber kSlipThreshold =
        new LoggedTunableNumber("Slip Threshold", 150000);

    // meters per second squared to be considered in freefall (less than)
    public static final double kFreefallAccelerationThreshold = 9.0;

    public static final Mass kRobotMass = Pounds.of(120);
    public static final MomentOfInertia kRobotMOI = KilogramSquareMeters.of(6.5);
  }

  public static final class IntakeConstants {
    public static final LoggedTunableNumber kIntakingVoltage =
        new LoggedTunableNumber("Intake/intakingVoltage", 8.0);
    public static final LoggedTunableNumber kIdleVoltage =
        new LoggedTunableNumber("Intake/idleVoltage", 0.0);

    public static final double kGearRatio = 24.0 / 56.0;

    // sim
    public static final DCMotor kSimGearbox = DCMotor.getKrakenX60(1);
    public static final double kSimMOI = .005;
  }

  public static final class AprilTagVisionConstants {
    public static final LoggedTunableNumber kUseVision = new LoggedTunableNumber("Use Vision", 1);

    public static final AprilTagFieldLayout kAprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final double kAprilTagWidth = Units.inchesToMeters(6.5);

    public static final double kAmbiguityThreshold = 0.4;
    public static final double kTargetLogTimeSecs = 0.1;
    public static final double kFieldBorderMargin = 0.5;
    // z margin is much smaller because the camera should be perfectly level with the tag
    public static final double kZMargin = 0.75;

    public static final double kErrorStandardDeviationThreshold = 0.2; // acceptable error

    public static final double kGyroAccurary =
        3.0; // higher numbers means the less we trust the vision/gyro sensor fusion

    // how long (sec) before we are considered disconnected
    public static final double kDisconnectTimeout = 5.0;

    public static final double kOdometryTrustFactorSlip = 0.2;
    public static final double kOdometryTrustFactorVisionScalar = 0.001;
    public static final double kOdometryTrustFactorNoVision = 0.04;

    public static final LoggedTunableNumber kXYStandardDeviationCoefficient =
        new LoggedTunableNumber("xyStandardDeviationCoefficient", 0.01);
    public static final LoggedTunableNumber kCloseStandardDeviation =
        new LoggedTunableNumber("AprilTagVision Close Standard Deviation", 0.01);
    public static final LoggedTunableNumber kFarStandardDeviation =
        new LoggedTunableNumber("AprilTagVision Far Standard Deviation", 0.01);
    public static final LoggedTunableNumber kThetaStandardDeviationCoefficient =
        new LoggedTunableNumber("thetaStandardDeviationCoefficient", 0.03);

    // transform from center of robot to camera
    public static final Transform3d[] kCameraTransforms =
        // NEW:
        // 0 - front right facing out, 10.4.22.229:1182, camera 3
        // 1 - front left facing out, 10.4.22.228:1182, camera 6
        // 2 - front left facing in, 10.4.22.229:1181, camera 11
        // 3 - front right facing in, 10.4.22.228:1181, camera 5
        new Transform3d[] {
          // front right (facing out)
          new Transform3d(
              Inches.of(9.796),
              Inches.of(-10.354),
              Inches.of(8.746),
              GeomUtil.constructRotation3d(Degrees.zero(), Degrees.of(-15.0), Degrees.of(-12.218))),

          // front left (facing out)
          new Transform3d(
              Inches.of(9.796),
              Inches.of(10.354),
              Inches.of(8.746),
              GeomUtil.constructRotation3d(Degrees.zero(), Degrees.of(-15.0), Degrees.of(12.218))),

          // front left (facing in)
          new Transform3d(
              Inches.of(10.484),
              Inches.of(11.464),
              Inches.of(8.771),
              GeomUtil.constructRotation3d(Degrees.zero(), Degrees.of(-17.43), Degrees.of(-19.0))),

          // front right (facing in)
          new Transform3d(
              Inches.of(10.484),
              Inches.of(-11.464),
              Inches.of(8.771),
              GeomUtil.constructRotation3d(Degrees.zero(), Degrees.of(-17.43), Degrees.of(19.0))),
        };

    // tolerances for using the vision rotation, temp values
    public static final double kRotationErrorThreshold = 0.3;
    public static final double kRotationDistanceThreshold = Units.inchesToMeters(200);
    public static final double kRotationSpeedThreshold = 0.2; // m/s
  }

  public static final class CurrentLimitConstants {
    // Drive
    public static final double kDriveDefaultSupplyCurrentLimit = 75.0;
    public static final double kDriveDefaultStatorCurrentLimit = 180.0;

    public static final double kTurnDefaultSupplyCurrentLimit = 60.0;
    public static final double kTurnDefaultStatorCurrentLimit = 120.0;

    // Intake
    public static final double kIntakeDefaultSupplyLimit = 30.0;
    public static final double kIntakeDefaultStatorLimit = 120.0;
  }

  public static final class Ports {
    public static final int kFrontLeftDrive = 0;
    public static final int kFrontLeftTurn = 1;
    public static final int kFrontLeftCancoder = 2;

    public static final int kFrontRightDrive = 3;
    public static final int kFrontRightTurn = 4;
    public static final int kFrontRightCancoder = 5;

    public static final int kBackLeftDrive = 6;
    public static final int kBackLeftTurn = 7;
    public static final int kBackLeftCancoder = 8;

    public static final int kBackRightDrive = 9;
    public static final int kBackRightTurn = 10;
    public static final int kBackRightCancoder = 11;

    public static final int kPigeon = 22;

    public static final String kDriveCanivoreName = "Drivetrain";

    public static final int kIntake = 12;

    public static final String kMainCanivoreName = "Main";
  }

  public static final class FieldConstants {
    public static final FieldType kFieldType = FieldType.WELDED;

    public static final double kFieldLength =
        AprilTagVisionConstants.kAprilTagLayout.getFieldLength();
    public static final double kFieldWidth =
        AprilTagVisionConstants.kAprilTagLayout.getFieldWidth();
    public static final double kStartingLineX =
        Units.inchesToMeters(299.438); // Measured from the inside of starting line
    public static final double kAlgaeDiameter = Units.inchesToMeters(16);
    public static final double kCoralDiameter = Units.inchesToMeters(4.5);

    public static class Processor {
      public static final Pose2d kCenterFace =
          new Pose2d(
              AprilTagVisionConstants.kAprilTagLayout.getTagPose(16).get().getX(),
              0,
              Rotation2d.fromDegrees(90));
      public static final Pose2d kOpposingCenterFace =
          new Pose2d(
              AprilTagVisionConstants.kAprilTagLayout.getTagPose(3).get().getX(),
              kFieldWidth,
              Rotation2d.fromDegrees(-90));
    }

    public static class Barge {
      public static final double kNetWidth = Units.inchesToMeters(40.0);
      public static final double kNetHeight = Units.inchesToMeters(88.0);

      public static final double kCageWidth = Units.inchesToMeters(6.0);
      public static final Translation2d kFarCage =
          new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
      public static final Translation2d kMiddleCage =
          new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
      public static final Translation2d kCloseCage =
          new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

      // Measured from floor to bottom of cage
      public static final double kDeepHeight = Units.inchesToMeters(3.125);
      public static final double kShallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
      public static final double kStationLength = Units.inchesToMeters(79.750);
      public static final Pose2d kRightCenterFace =
          new Pose2d(
              Units.inchesToMeters(33.526),
              Units.inchesToMeters(25.824),
              Rotation2d.fromDegrees(144.011 - 90));
      public static final Pose2d kLeftCenterFace =
          new Pose2d(
              kRightCenterFace.getX(),
              kFieldWidth - kRightCenterFace.getY(),
              Rotation2d.fromRadians(-kRightCenterFace.getRotation().getRadians()));
    }

    public static class Reef {
      public static final double kFaceLength = Units.inchesToMeters(36.792600);
      public static final Translation2d kCenter =
          new Translation2d(Units.inchesToMeters(176.746), kFieldWidth / 2.0);
      public static final double kFaceToZoneLine =
          Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

      public static final Pose2d[] kCenterFaces =
          new Pose2d[6]; // Starting facing the driver station in clockwise order
      public static final List<Map<ReefHeight, Pose3d>> branchPositions =
          new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
      public static final List<Map<ReefHeight, Pose2d>> branchPositions2d = new ArrayList<>();

      static {
        // Initialize faces
        var aprilTagLayout = AprilTagVisionConstants.kAprilTagLayout;
        kCenterFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
        kCenterFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
        kCenterFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
        kCenterFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
        kCenterFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
        kCenterFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

        // Initialize branch positions
        for (int face = 0; face < 6; face++) {
          Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
          Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
          Map<ReefHeight, Pose2d> fillRight2d = new HashMap<>();
          Map<ReefHeight, Pose2d> fillLeft2d = new HashMap<>();
          for (var level : ReefHeight.values()) {
            Pose2d poseDirection = new Pose2d(kCenter, Rotation2d.fromDegrees(180 - (60 * face)));
            double adjustX = Units.inchesToMeters(30.738);
            double adjustY = Units.inchesToMeters(6.469);

            var rightBranchPose =
                new Pose3d(
                    new Translation3d(
                        poseDirection
                            .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                            .getX(),
                        poseDirection
                            .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                            .getY(),
                        level.height),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(level.pitch),
                        poseDirection.getRotation().getRadians()));
            var leftBranchPose =
                new Pose3d(
                    new Translation3d(
                        poseDirection
                            .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                            .getX(),
                        poseDirection
                            .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                            .getY(),
                        level.height),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(level.pitch),
                        poseDirection.getRotation().getRadians()));

            fillRight.put(level, rightBranchPose);
            fillLeft.put(level, leftBranchPose);
            fillRight2d.put(level, rightBranchPose.toPose2d());
            fillLeft2d.put(level, leftBranchPose.toPose2d());
          }
          branchPositions.add(fillRight);
          branchPositions.add(fillLeft);
          branchPositions2d.add(fillRight2d);
          branchPositions2d.add(fillLeft2d);
        }
      }
    }

    public static class StagingPositions {
      // Measured from the center of the ice cream
      public static final double kSeparation = Units.inchesToMeters(72.0);
      public static final Translation2d[] kIceCreams = new Translation2d[3];

      static {
        for (int i = 0; i < 3; i++) {
          kIceCreams[i] =
              new Translation2d(
                  Units.inchesToMeters(48), kFieldWidth / 2.0 - kSeparation + kSeparation * i);
        }
      }
    }

    public enum ReefHeight {
      L1(0, Units.inchesToMeters(25.0), 0),
      L2(1, Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
      L3(2, Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
      L4(3, Units.inchesToMeters(72), -90);

      ReefHeight(int levelNumber, double height, double pitch) {
        this.levelNumber = levelNumber;
        this.height = height;
        this.pitch = pitch; // Degrees
      }

      public static ReefHeight fromLevel(int level) {
        return Arrays.stream(values())
            .filter(height -> height.ordinal() == level)
            .findFirst()
            .orElse(L4);
      }

      public final int levelNumber;
      public final double height;
      public final double pitch;
    }

    @RequiredArgsConstructor
    public enum FieldType {
      ANDYMARK,
      WELDED,
    }
  }
}
