package frc.robot.dashboard;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class Dashboard {
    private static final double FAST_PERIOD_SEC = 0.05; // 20 Hz
    private static final double MEDIUM_PERIOD_SEC = 0.10; // 10 Hz
    private static final double SLOW_PERIOD_SEC = 0.25; // 4 Hz

    private final Swerve swerve;
    private final Shooter shooter;
    private final Hood hood;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Limelight limelight;

    private final Field2d field = new Field2d();

    private final NetworkTable root = NetworkTableInstance.getDefault().getTable("Dashboard");

    // ================= Match =================
    private final StringPublisher matchModePub = root.getStringTopic("Match/Mode").publish();
    private final DoublePublisher matchTimePub = root.getDoubleTopic("Match/TimeSec").publish();
    private final StringPublisher alliancePub = root.getStringTopic("Match/Alliance").publish();
    private final BooleanPublisher enabledPub = root.getBooleanTopic("Match/Enabled").publish();
    private final BooleanPublisher autoPub = root.getBooleanTopic("Match/Autonomous").publish();
    private final BooleanPublisher teleopPub = root.getBooleanTopic("Match/Teleop").publish();
    private final BooleanPublisher gameDataValidPub = root.getBooleanTopic("Match/GameDataValid").publish();
    private final StringPublisher timeSourcePub = root.getStringTopic("Match/TimeSource").publish();
    private final BooleanPublisher officialActivePub = root.getBooleanTopic("Match/OfficialActive").publish();

    // ================= Auto =================
    private final StringPublisher autoSelectedPub = root.getStringTopic("Auto/Selected").publish();
    private final StringPublisher autoStatePub = root.getStringTopic("Auto/State").publish();
    private final StringPublisher autoLastCompletedPub = root.getStringTopic("Auto/LastCompletedState").publish();
    private final StringPublisher autoWaitingOnPub = root.getStringTopic("Auto/WaitingOn").publish();
    private final StringPublisher autoFailurePub = root.getStringTopic("Auto/FailureReason").publish();
    private final StringPublisher autoPathPub = root.getStringTopic("Auto/PathName").publish();
    private final StringPublisher autoMarkerPub = root.getStringTopic("Auto/LastMarker").publish();
    private final DoublePublisher autoTimeInStatePub = root.getDoubleTopic("Auto/TimeInStateSec").publish();

    // ================= Drive / pose =================
    private final DoublePublisher poseXPub = root.getDoubleTopic("Drive/PoseX").publish();
    private final DoublePublisher poseYPub = root.getDoubleTopic("Drive/PoseY").publish();
    private final DoublePublisher poseThetaDegPub = root.getDoubleTopic("Drive/PoseThetaDeg").publish();

    private final BooleanPublisher hasTargetPosePub = root.getBooleanTopic("Drive/HasTargetPose").publish();
    private final DoublePublisher targetXPub = root.getDoubleTopic("Drive/TargetX").publish();
    private final DoublePublisher targetYPub = root.getDoubleTopic("Drive/TargetY").publish();
    private final DoublePublisher targetThetaDegPub = root.getDoubleTopic("Drive/TargetThetaDeg").publish();

    private final DoublePublisher errorXPub = root.getDoubleTopic("Drive/ErrorX").publish();
    private final DoublePublisher errorYPub = root.getDoubleTopic("Drive/ErrorY").publish();
    private final DoublePublisher errorThetaDegPub = root.getDoubleTopic("Drive/ErrorThetaDeg").publish();

    // ================= Launch =================
    private final DoublePublisher shooterTargetPub = root.getDoubleTopic("Launch/ShooterTargetRPM").publish();
    private final DoublePublisher shooterActualPub = root.getDoubleTopic("Launch/ShooterActualRPM").publish();
    private final DoublePublisher shooterErrorPub = root.getDoubleTopic("Launch/ShooterErrorRPM").publish();
    private final DoublePublisher hoodTargetPub = root.getDoubleTopic("Launch/HoodTarget").publish();
    private final DoublePublisher hoodActualPub = root.getDoubleTopic("Launch/HoodActual").publish();
    private final BooleanPublisher shooterReadyPub = root.getBooleanTopic("Launch/ShooterReady").publish();
    private final BooleanPublisher hoodReadyPub = root.getBooleanTopic("Launch/HoodReady").publish();
    private final BooleanPublisher launchAllowedPub = root.getBooleanTopic("Launch/Allowed").publish();
    private final StringPublisher launchBlockReasonPub = root.getStringTopic("Launch/BlockReason").publish();

    // ================= Vision =================
    private final BooleanPublisher visionValidPub = root.getBooleanTopic("Vision/HasMeasurement").publish();
    private final DoublePublisher visionPoseXPub = root.getDoubleTopic("Vision/EstimatedPoseX").publish();
    private final DoublePublisher visionPoseYPub = root.getDoubleTopic("Vision/EstimatedPoseY").publish();
    private final DoublePublisher visionPoseThetaDegPub = root.getDoubleTopic("Vision/EstimatedPoseThetaDeg").publish();
    private final IntegerPublisher visionTagCountPub = root.getIntegerTopic("Vision/TagCount").publish();
    private final DoublePublisher visionTimestampPub = root.getDoubleTopic("Vision/TimestampSec").publish();
    private final DoublePublisher visionAgePub = root.getDoubleTopic("Vision/AgeSec").publish();
    private final DoublePublisher visionStdDevXPub = root.getDoubleTopic("Vision/StdDevX").publish();
    private final DoublePublisher visionStdDevYPub = root.getDoubleTopic("Vision/StdDevY").publish();
    private final DoublePublisher visionStdDevThetaPub = root.getDoubleTopic("Vision/StdDevTheta").publish();

    // ================= Mechanisms =================
    private final StringPublisher intakeCmdPub = root.getStringTopic("Mechanisms/IntakeCommand").publish();
    private final StringPublisher floorCmdPub = root.getStringTopic("Mechanisms/FloorCommand").publish();
    private final StringPublisher feederCmdPub = root.getStringTopic("Mechanisms/FeederCommand").publish();

    // ================= Health =================
    private final BooleanPublisher healthOkPub = root.getBooleanTopic("Health/Ok").publish();
    private final IntegerPublisher healthErrorCountPub = root.getIntegerTopic("Health/ErrorCount").publish();
    private final StringPublisher healthLastErrorPub = root.getStringTopic("Health/LastError").publish();
    private final DoublePublisher healthLastUpdatePub = root.getDoubleTopic("Health/LastUpdateSec").publish();

    // Internal state for strings / booleans / slow-changing values
    private final Map<String, String> stringCache = new HashMap<>();

    private Optional<Pose2d> targetPose = Optional.empty();

    private String autoSelected = "None";
    private String autoState = "Idle";
    private String autoLastCompleted = "None";
    private String autoWaitingOn = "";
    private String autoFailure = "";
    private String autoPath = "";
    private String autoMarker = "";

    private boolean gameDataValid = false;
    private String matchTimeSource = "Unknown";
    private boolean officialActive = false;

    private boolean launchAllowed = false;
    private String launchBlockReason = "";

    private double autoStateStartSec = Timer.getFPGATimestamp();

    private double lastFastUpdateSec = 0.0;
    private double lastMediumUpdateSec = 0.0;
    private double lastSlowUpdateSec = 0.0;

    private long errorCount = 0;
    private String lastError = "";
    private double lastErrorSec = -9999.0;

    public Dashboard(
            Swerve swerve,
            Shooter shooter,
            Hood hood,
            Intake intake,
            Floor floor,
            Feeder feeder,
            Limelight limelight) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.hood = hood;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.limelight = limelight;
    }

    public void init() {
        SmartDashboard.putData("Dashboard/Field", field);

        // Defaults to avoid weird blank states on first connection
        publishString("Match/Mode", matchModePub, "Disabled");
        publishString("Match/Alliance", alliancePub, "Unknown");
        publishString("Match/TimeSource", timeSourcePub, matchTimeSource);

        publishString("Auto/Selected", autoSelectedPub, autoSelected);
        publishString("Auto/State", autoStatePub, autoState);
        publishString("Auto/LastCompletedState", autoLastCompletedPub, autoLastCompleted);
        publishString("Auto/WaitingOn", autoWaitingOnPub, autoWaitingOn);
        publishString("Auto/FailureReason", autoFailurePub, autoFailure);
        publishString("Auto/PathName", autoPathPub, autoPath);
        publishString("Auto/LastMarker", autoMarkerPub, autoMarker);

        publishString("Launch/BlockReason", launchBlockReasonPub, launchBlockReason);

        publishString("Mechanisms/IntakeCommand", intakeCmdPub, "None");
        publishString("Mechanisms/FloorCommand", floorCmdPub, "None");
        publishString("Mechanisms/FeederCommand", feederCmdPub, "None");

        publishString("Health/LastError", healthLastErrorPub, "");
        healthOkPub.set(true);
        healthErrorCountPub.set(0);
        healthLastUpdatePub.set(Timer.getFPGATimestamp());
    }

    public void periodic() {
        final double now = Timer.getFPGATimestamp();

        if (shouldUpdate(now, lastFastUpdateSec, FAST_PERIOD_SEC)) {
            safeRun("FastUpdate", () -> updateFast(now));
            lastFastUpdateSec = now;
        }

        if (shouldUpdate(now, lastMediumUpdateSec, MEDIUM_PERIOD_SEC)) {
            safeRun("MediumUpdate", () -> updateMedium(now));
            lastMediumUpdateSec = now;
        }

        if (shouldUpdate(now, lastSlowUpdateSec, SLOW_PERIOD_SEC)) {
            safeRun("SlowUpdate", () -> updateSlow(now));
            lastSlowUpdateSec = now;
        }
    }

    private void updateFast(double now) {
        // Pose
        final Pose2d pose = safePose(swerve.getState().Pose);
        field.setRobotPose(pose);

        poseXPub.set(safeDouble(pose.getX()));
        poseYPub.set(safeDouble(pose.getY()));
        poseThetaDegPub.set(safeDouble(pose.getRotation().getDegrees()));

        // Target pose + error
        hasTargetPosePub.set(targetPose.isPresent());
        if (targetPose.isPresent()) {
            final Pose2d target = safePose(targetPose.get());

            targetXPub.set(safeDouble(target.getX()));
            targetYPub.set(safeDouble(target.getY()));
            targetThetaDegPub.set(safeDouble(target.getRotation().getDegrees()));

            errorXPub.set(safeDouble(target.getX() - pose.getX()));
            errorYPub.set(safeDouble(target.getY() - pose.getY()));
            errorThetaDegPub.set(
                    safeDouble(target.getRotation().minus(pose.getRotation()).getDegrees()));

            field.getObject("TargetPose").setPose(target);
        } else {
            targetXPub.set(0.0);
            targetYPub.set(0.0);
            targetThetaDegPub.set(0.0);
            errorXPub.set(0.0);
            errorYPub.set(0.0);
            errorThetaDegPub.set(0.0);
        }

        // Shooter fast values
        shooterActualPub.set(safeDouble(shooter.getAverageRPM()));
        shooterErrorPub.set(safeDouble(shooter.getRPMError()));

        // Vision
        final boolean hasVision = limelight.hasValidMeasurement();
        visionValidPub.set(hasVision);

        if (hasVision) {
            final Pose2d visionPose = safePose(
                    limelight.getLastEstimatedPose().orElse(new Pose2d()));
            visionPoseXPub.set(safeDouble(visionPose.getX()));
            visionPoseYPub.set(safeDouble(visionPose.getY()));
            visionPoseThetaDegPub.set(safeDouble(visionPose.getRotation().getDegrees()));
            visionTagCountPub.set(limelight.getLastTagCount());
            visionTimestampPub.set(safeDouble(limelight.getLastTimestampSeconds()));
            visionAgePub.set(safeDouble(now - limelight.getLastTimestampSeconds()));
            visionStdDevXPub.set(safeDouble(limelight.getLastStdDevX()));
            visionStdDevYPub.set(safeDouble(limelight.getLastStdDevY()));
            visionStdDevThetaPub.set(safeDouble(limelight.getLastStdDevTheta()));

            field.getObject("VisionEstimate").setPose(visionPose);
        } else {
            visionPoseXPub.set(0.0);
            visionPoseYPub.set(0.0);
            visionPoseThetaDegPub.set(0.0);
            visionTagCountPub.set(0);
            visionTimestampPub.set(0.0);
            visionAgePub.set(0.0);
            visionStdDevXPub.set(0.0);
            visionStdDevYPub.set(0.0);
            visionStdDevThetaPub.set(0.0);
        }
    }

    private void updateMedium(double now) {
        // Match numeric / frequent
        matchTimePub.set(safeDouble(DriverStation.getMatchTime()));
        enabledPub.set(DriverStation.isEnabled());
        autoPub.set(DriverStation.isAutonomousEnabled());
        teleopPub.set(DriverStation.isTeleopEnabled());

        // Auto timing
        autoTimeInStatePub.set(safeDouble(now - autoStateStartSec));

        // Launch / hood / readiness
        shooterTargetPub.set(safeDouble(shooter.getTargetRPM()));
        shooterReadyPub.set(safeBoolean(shooter.isVelocityWithinTolerance()));

        hoodTargetPub.set(safeDouble(hood.getTargetPosition()));
        hoodActualPub.set(safeDouble(hood.getCurrentPosition()));
        hoodReadyPub.set(safeBoolean(hood.isPositionWithinTolerance()));

        launchAllowedPub.set(launchAllowed);
        publishString("Launch/BlockReason", launchBlockReasonPub, launchBlockReason);

        // Health heartbeat
        healthLastUpdatePub.set(now);
        healthOkPub.set((now - lastErrorSec) > 1.0);
        healthErrorCountPub.set(errorCount);
        publishString("Health/LastError", healthLastErrorPub, lastError);
    }

    private void updateSlow(double now) {
        // Match metadata
        publishString("Match/Mode", matchModePub, currentMode());
        publishString("Match/Alliance", alliancePub,
                DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));
        gameDataValidPub.set(gameDataValid);
        publishString("Match/TimeSource", timeSourcePub, matchTimeSource);
        officialActivePub.set(officialActive);

        // Auto strings
        publishString("Auto/Selected", autoSelectedPub, autoSelected);
        publishString("Auto/State", autoStatePub, autoState);
        publishString("Auto/LastCompletedState", autoLastCompletedPub, autoLastCompleted);
        publishString("Auto/WaitingOn", autoWaitingOnPub, autoWaitingOn);
        publishString("Auto/FailureReason", autoFailurePub, autoFailure);
        publishString("Auto/PathName", autoPathPub, autoPath);
        publishString("Auto/LastMarker", autoMarkerPub, autoMarker);

        // Mechanism commands
        publishString("Mechanisms/IntakeCommand", intakeCmdPub, commandName(intake));
        publishString("Mechanisms/FloorCommand", floorCmdPub, commandName(floor));
        publishString("Mechanisms/FeederCommand", feederCmdPub, commandName(feeder));

        // Health again, so it stays fresh in slower dashboard-only views
        healthLastUpdatePub.set(now);
        healthOkPub.set((now - lastErrorSec) > 1.0);
        healthErrorCountPub.set(errorCount);
        publishString("Health/LastError", healthLastErrorPub, lastError);
    }

    private void safeRun(String sectionName, Runnable action) {
        try {
            action.run();
        } catch (Exception e) {
            errorCount++;
            lastErrorSec = Timer.getFPGATimestamp();
            lastError = sectionName + ": " + e.getClass().getSimpleName();
        }
    }

    private static boolean shouldUpdate(double now, double last, double periodSec) {
        return (now - last) >= periodSec;
    }

    private void publishString(String cacheKey, StringPublisher publisher, String value) {
        final String safe = safeString(value, "");
        final String cached = stringCache.get(cacheKey);
        if (!safe.equals(cached)) {
            publisher.set(safe);
            stringCache.put(cacheKey, safe);
        }
    }

    private static String safeString(String value, String fallback) {
        return value != null ? value : fallback;
    }

    private static boolean safeBoolean(boolean value) {
        return value;
    }

    private static double safeDouble(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private static Pose2d safePose(Pose2d pose) {
        return pose != null ? pose : new Pose2d();
    }

    private String currentMode() {
        if (DriverStation.isAutonomousEnabled()) {
            return "Autonomous";
        }
        if (DriverStation.isTeleopEnabled()) {
            return "Teleop";
        }
        if (DriverStation.isEnabled()) {
            return "Enabled";
        }
        return "Disabled";
    }

    private static String commandName(Subsystem subsystem) {
        if (subsystem == null || subsystem.getCurrentCommand() == null) {
            return "None";
        }
        final String name = subsystem.getCurrentCommand().getName();
        return name != null ? name : "UnnamedCommand";
    }

    // ---------------- Public setters used by robot code ----------------

    public void setAutoSelected(String name) {
        this.autoSelected = safeString(name, "None");
    }

    public void setAutoState(String state) {
        final String safe = safeString(state, "Unknown");
        if (!safe.equals(this.autoState)) {
            this.autoState = safe;
            this.autoStateStartSec = Timer.getFPGATimestamp();
        }
    }

    public void completeAutoState(String state) {
        this.autoLastCompleted = safeString(state, "Unknown");
    }

    public void setAutoWaitingOn(String reason) {
        this.autoWaitingOn = safeString(reason, "");
    }

    public void clearAutoWaitingOn() {
        this.autoWaitingOn = "";
    }

    public void setAutoFailure(String reason) {
        this.autoFailure = safeString(reason, "");
    }

    public void clearAutoFailure() {
        this.autoFailure = "";
    }

    public void setAutoPath(String pathName) {
        this.autoPath = safeString(pathName, "");
    }

    public void setAutoMarker(String marker) {
        this.autoMarker = safeString(marker, "");
    }

    public void setTargetPose(Pose2d pose) {
        this.targetPose = Optional.ofNullable(pose);
    }

    public void clearTargetPose() {
        this.targetPose = Optional.empty();
    }

    public void setLaunchAllowed(boolean allowed) {
        this.launchAllowed = allowed;
    }

    public void setLaunchBlockReason(String reason) {
        this.launchBlockReason = safeString(reason, "");
    }

    public void setGameDataValid(boolean valid) {
        this.gameDataValid = valid;
    }

    public void setMatchTimeSource(String source) {
        this.matchTimeSource = safeString(source, "Unknown");
    }

    public void setOfficialActive(boolean active) {
        this.officialActive = active;
    }
}