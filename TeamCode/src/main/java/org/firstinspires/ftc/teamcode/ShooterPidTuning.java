package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

// FYI: You need to enter "Config" menu in FTC Dashboard to edit PIDF values.
@Config
public class ShooterPidTuning {
    public static double outtakeP, outtakeI, outtakeD, outtakeF;
    public static double counterspinP, counterspinI, counterspinD, counterspinF;

    // Set true in Dashboard to apply the displayed values once.
    public static boolean apply = false;
}