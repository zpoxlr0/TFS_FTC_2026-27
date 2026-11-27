package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

// FYI: You need to enter "Config" menu in FTC Dashboard to edit PIDF values.
@Config
public class ShooterPidTuning {
    // SHOOTER PID //
    public static double outtakeP, outtakeI, outtakeD, outtakeF;
    public static double counterspinP, counterspinI, counterspinD, counterspinF;

    // FLYWHEEL/COUNTERROLLER RAW MOTOR SPEED //
    public static double flywheelSpeed = 0.63;
    public static double counterrollerSpeed = 0.15;

    // Set true in Dashboard to apply the displayed values once.
    public static boolean apply = false;
    public static boolean applySpeed = false;
}