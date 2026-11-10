/*
 * Adapted from RoBovines 6955 Stampede.
 *
 * Copyright (c) 2014-2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED
 * BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Wheel-encoder and IMU adaptation of Stampede's field-position tracker.
 *
 * ProgrammingBoard owns and controls the motors.
 * This class only reads their encoder positions.
 *
 * Position: inches.
 * Heading: degrees, positive counterclockwise.
 * Heading zero points along field +X.
 */
public class Stampede {

    // Replace these with YOUR measured positive counts per inch.
    // NaN deliberately disables distance tracking before calibration.
    public static final double FORWARD_ENCODER_COUNTS_PER_INCH =
            Double.NaN;

    public static final double RIGHT_ENCODER_COUNTS_PER_INCH =
            Double.NaN;

    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;

    private final IMU imu;

    private int lastLF;
    private int lastLB;
    private int lastRF;
    private int lastRB;

    private int zeroLF;
    private int zeroLB;
    private int zeroRF;
    private int zeroRB;

    private double xFieldPos;
    private double yFieldPos;
    private double headingField;
    private double headingOffset;

    public Stampede(
            DcMotor leftFrontDrive,
            DcMotor leftBackDrive,
            DcMotor rightFrontDrive,
            DcMotor rightBackDrive,
            IMU imu
    ) {
        this.leftFrontDrive = leftFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.rightBackDrive = rightBackDrive;
        this.imu = imu;

        setFieldPosition(0, 0, 0);
        resetCalibrationReadings();
    }

    public boolean isCalibrated() {
        return Double.isFinite(FORWARD_ENCODER_COUNTS_PER_INCH)
                && FORWARD_ENCODER_COUNTS_PER_INCH > 0
                && Double.isFinite(RIGHT_ENCODER_COUNTS_PER_INCH)
                && RIGHT_ENCODER_COUNTS_PER_INCH > 0;
    }

    public boolean isTrackingReady() {
        return imu != null && isCalibrated();
    }

    public boolean hasImu() {
        return imu != null;
    }

    // Check isTrackingReady() before using X/Y for robot decisions.
    public double getX() {
        return xFieldPos;
    }

    public double getY() {
        return yFieldPos;
    }

    public double getHeading() {
        return imu == null ? Double.NaN : headingField;
    }

    private double readHeading() {
        return normalize(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
                        + headingOffset
        );
    }

    private static double normalize(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    /**
     * Assign a known field position and refresh encoder baselines.
     * Movement before this call will not be counted again afterward.
     */
    public void setFieldPosition(double x, double y, double heading) {
        if (!Double.isFinite(x)
                || !Double.isFinite(y)
                || !Double.isFinite(heading)) {
            throw new IllegalArgumentException(
                    "Field position must be finite"
            );
        }

        xFieldPos = x;
        yFieldPos = y;
        headingField = normalize(heading);

        if (imu != null) {
            headingOffset = headingField
                    - imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);
        }

        lastLF = leftFrontDrive.getCurrentPosition();
        lastLB = leftBackDrive.getCurrentPosition();
        lastRF = rightFrontDrive.getCurrentPosition();
        lastRB = rightBackDrive.getCurrentPosition();
    }

    /**
     * Call once each TeleOp loop, even before calibration is complete.
     */
    public void updateFieldPosition() {
        int lf = leftFrontDrive.getCurrentPosition();
        int lb = leftBackDrive.getCurrentPosition();
        int rf = rightFrontDrive.getCurrentPosition();
        int rb = rightBackDrive.getCurrentPosition();

        double dLF = lf - lastLF;
        double dLB = lb - lastLB;
        double dRF = rf - lastRF;
        double dRB = rb - lastRB;

        lastLF = lf;
        lastLB = lb;
        lastRF = rf;
        lastRB = rb;

        if (imu == null) {
            return;
        }

        double newHeading = readHeading();

        if (isCalibrated()) {
            // Inverse of ProgrammingBoard.drive().
            // Counts already account for each motor's configured direction.
            double forward =
                    (dLF + dLB + dRF + dRB)
                            / 4.0
                            / FORWARD_ENCODER_COUNTS_PER_INCH;

            double right =
                    (dLF - dLB - dRF + dRB)
                            / 4.0
                            / RIGHT_ENCODER_COUNTS_PER_INCH;

            // Shortest heading change handles the -180/180 boundary.
            double averageHeading = Math.toRadians(
                    headingField
                            + normalize(newHeading - headingField) / 2.0
            );

            // Rotate robot-relative travel into field coordinates.
            xFieldPos +=
                    forward * Math.cos(averageHeading)
                            + right * Math.sin(averageHeading);

            yFieldPos +=
                    forward * Math.sin(averageHeading)
                            - right * Math.cos(averageHeading);
        }

        headingField = newHeading;
    }

    /**
     * Zero calibration telemetry only.
     * Does not reset hardware encoders or change the field position.
     */
    public void resetCalibrationReadings() {
        zeroLF = leftFrontDrive.getCurrentPosition();
        zeroLB = leftBackDrive.getCurrentPosition();
        zeroRF = rightFrontDrive.getCurrentPosition();
        zeroRB = rightBackDrive.getCurrentPosition();
    }

    public void reportTelemetry(Telemetry telemetry) {
        int lf = leftFrontDrive.getCurrentPosition() - zeroLF;
        int lb = leftBackDrive.getCurrentPosition() - zeroLB;
        int rf = rightFrontDrive.getCurrentPosition() - zeroRF;
        int rb = rightBackDrive.getCurrentPosition() - zeroRB;

        telemetry.addData(
                "Encoders since A (LF/LB/RF/RB)",
                "%d / %d / %d / %d",
                lf, lb, rf, rb
        );

        telemetry.addData(
                "Forward counts",
                "%.1f",
                (lf + (double) lb + rf + rb) / 4.0
        );

        telemetry.addData(
                "Right counts",
                "%.1f",
                (lf - (double) lb - rf + rb) / 4.0
        );

        telemetry.addData(
                "CW turn counts",
                "%.1f",
                (lf + (double) lb - rf - rb) / 4.0
        );

        telemetry.addData(
                "IMU",
                imu == null
                        ? "Set hub logo direction in ProgrammingBoard"
                        : "Configured"
        );

        if (imu != null) {
            telemetry.addData(
                    "Heading (degrees CCW)",
                    "%.1f",
                    headingField
            );
        }

        telemetry.addData(
                "Distance calibration",
                isCalibrated()
                        ? "Configured; verify on field"
                        : "NOT SET in Stampede.java"
        );

        if (isTrackingReady()) {
            telemetry.addData(
                    "Field position (inches)",
                    "X %.2f | Y %.2f",
                    xFieldPos,
                    yFieldPos
            );
        } else {
            telemetry.addData(
                    "Field position",
                    "UNAVAILABLE until IMU and distance calibration are set"
            );
        }
    }
}