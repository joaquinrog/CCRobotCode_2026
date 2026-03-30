package frc.robot.dashboard;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

final class MatchHelpers {
    private static final double TRANSITION_END_TIME_SEC = 130.0;
    private static final double SHIFT_1_END_TIME_SEC = 105.0;
    private static final double SHIFT_2_END_TIME_SEC = 80.0;
    private static final double SHIFT_3_END_TIME_SEC = 55.0;
    private static final double SHIFT_4_END_TIME_SEC = 30.0;

    private Optional<Alliance> latchedInactiveFirstAlliance = Optional.empty();

    private Optional<Alliance> parseInactiveFirstAlliance(String gameData) {
        final String safeGameData = gameData == null ? "" : gameData.trim().toUpperCase();

        return switch (safeGameData) {
            case "R" -> Optional.of(Alliance.Red);
            case "B" -> Optional.of(Alliance.Blue);
            default -> Optional.empty();
        };
    }

    private Optional<Alliance> getInactiveFirstAllianceInternal() {
        final Optional<Alliance> parsed = parseInactiveFirstAlliance(
                DriverStation.getGameSpecificMessage());

        if (parsed.isPresent()) {
            latchedInactiveFirstAlliance = parsed;
        }

        return latchedInactiveFirstAlliance;
    }

    public boolean isGameDataValid() {
        return getInactiveFirstAllianceInternal().isPresent();
    }

    public String getInactiveFirstAllianceName() {
        return getInactiveFirstAllianceInternal()
                .map(Enum::name)
                .orElse("Unknown");
    }

    public boolean didWeWinAuto() {
        final Optional<Alliance> ourAlliance = DriverStation.getAlliance();
        final Optional<Alliance> inactiveFirstAlliance = getInactiveFirstAllianceInternal();

        return ourAlliance.isPresent()
                && inactiveFirstAlliance.isPresent()
                && ourAlliance.get() == inactiveFirstAlliance.get();
    }

    public boolean isTransitionActive() {
        return DriverStation.isTeleopEnabled()
                && DriverStation.getMatchTime() > TRANSITION_END_TIME_SEC;
    }

    public boolean isHubActive() {
        final Optional<Alliance> ourAlliance = DriverStation.getAlliance();

        if (ourAlliance.isEmpty()) {
            return false;
        }

        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        final double matchTimeSec = DriverStation.getMatchTime();
        final Optional<Alliance> inactiveFirstAlliance = getInactiveFirstAllianceInternal();

        // WPILib example recommends treating missing/invalid game data as active.
        if (inactiveFirstAlliance.isEmpty()) {
            return true;
        }

        final boolean redInactiveFirst = inactiveFirstAlliance.get() == Alliance.Red;

        final boolean shift1Active = switch (ourAlliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTimeSec > TRANSITION_END_TIME_SEC) {
            return true; // Transition Shift
        } else if (matchTimeSec > SHIFT_1_END_TIME_SEC) {
            return shift1Active; // Shift 1
        } else if (matchTimeSec > SHIFT_2_END_TIME_SEC) {
            return !shift1Active; // Shift 2
        } else if (matchTimeSec > SHIFT_3_END_TIME_SEC) {
            return shift1Active; // Shift 3
        } else if (matchTimeSec > SHIFT_4_END_TIME_SEC) {
            return !shift1Active; // Shift 4
        } else {
            return true; // End Game
        }
    }

    public boolean isOfficialActive() {
        return DriverStation.isFMSAttached();
    }

    public String getTimeSource() {
        return DriverStation.isFMSAttached() ? "FMS" : "DriverStation";
    }
}