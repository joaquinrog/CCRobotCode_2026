package frc.util;

import frc.robot.Constants;

public class DriveModeManager {
    private boolean intakePrecisionEnabled = false;

    public void setIntakePrecisionEnabled(boolean enabled) {
        intakePrecisionEnabled = enabled;
    }

    public boolean isIntakePrecisionEnabled() {
        return intakePrecisionEnabled;
    }

    public double getTranslationSpeedMultiplier() {
        return intakePrecisionEnabled
                ? Constants.Driving.kIntakingTranslationSpeedMultiplier
                : 1.0;
    }

    public double getRotationSpeedMultiplier() {
        return intakePrecisionEnabled
                ? Constants.Driving.kIntakingRotationSpeedMultiplier
                : 1.0;
    }
}