/*
 * Copyright (c) 2026 Base 10 Assets, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This file includes an autonomous file for the goBILDA® StarterBot for the
 * 2026-2027 FIRST® Tech Challenge season BIOBUZZ™.
 *
 * This version maps four drive motors independently. It also uses one motor
 * driving an intake roller, two servos which pull elements out of corners,
 * a transfer servo, and a high-speed launcher motor.
 *
 * This robot starts up against the playing field wall and attempts to launch
 * its preloaded projectiles for 10 seconds before driving away from the wall.
 *
 * This program leverages a "state machine" - an Enum which captures the state
 * of the robot at any time. As it moves through the autonomous period and
 * completes different functions, it will move forward in the enum.
 *
 * This allows us to run the autonomous period inside of our main robot "loop,"
 * continuously checking for conditions that allow us to move to the next step.
 */

@Autonomous(name = "Narwhals Auton A", group = "Narwhals")
//@Disabled
public class Narwhals_Auton_A extends OpMode {

    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second.
     *
     * The starter's launcher motor has an encoder resolution of 28 ticks per
     * revolution. We can convert this to RPM by dividing the value by 28,
     * to get revolutions per second, before multiplying by 60 to get RPM.
     *
     * We pass the target velocity variable to our motor to set the goal.
     * We use the minimum velocity in the launch() function to only run the
     * transfer servo when the motor is spinning fast enough to make a throw.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    /*
     * Here we capture a few variables used in driving the robot.
     * DRIVE_SPEED and ROTATE_SPEED are from 0-1, with 1 being full speed.
     *
     * Encoder ticks per revolution is specific to the motor ratio that we use
     * in the kit. If you're using a different motor, this value can be found
     * on the product page for the motor you're using.
     *
     * Track width is the distance between the center of the drive wheels on
     * either side of the robot. The starter uses this to estimate the distance
     * each wheel needs to travel to create a specified rotation.
     *
     * For a mecanum drivetrain, the rotation estimate also depends on wheelbase
     * and wheel slip, so the rotation helper will require tuning.
     */
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;

    final double TICKS_PER_MM =
            ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);

    final double TRACK_WIDTH_MM = 402;

    // Create a variable to set to the intake.
    double intakePower = 0;

    /*
     * Here we create a timer which we use in our drive function, and a timer
     * to control the flow of our auto as a whole.
     */
    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    // Declare OpMode members.
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    // This array lets us apply shared setup to all four independent motors.
    private DcMotor[] driveMotors;

    private DcMotorEx launcher = null;
    private DcMotor intake = null;

    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * TECH TIP: State Machines
     *
     * We use "state machines" in a few different ways in this auto.
     * The first step of a state machine is creating an enum that captures
     * the different "states" that our code can be in.
     *
     * The core advantage of a state machine is that it allows us to continue
     * to loop through code and only run the bits of code we need at different
     * times. This state machine is called the "AutonomousState."
     *
     * It reflects the current state of our auto. It starts at LAUNCH, and we
     * can use higher level code to cycle through these states.
     *
     * This allows us to write functions and autonomous routines in a way
     * that avoids loops within loops and blocking waits.
     */
    private enum AutonomousState {
        LAUNCH,
        DRIVE,
        COMPLETE
    }

    /*
     * Here we create the instance of AutonomousState that we use in code.
     * This creates a variable which stores the current state of the routine.
     *
     * In other applications, you may have multiple variables using the same
     * enum which have different names. Here we just have one.
     */
    private AutonomousState autonomousState;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {

        /*
         * Here we set the first step of our autonomous state machine by setting
         * autonomousState = AutonomousState.LAUNCH.
         *
         * Later in our code, we will progress through the state machine by
         * moving to other enum members.
         *
         * The launcher is commanded off during initialization.
         */
        autonomousState = AutonomousState.LAUNCH;

        /*
         * Initialize the hardware variables. Note that the strings used here
         * as parameters to 'get' must correspond to the names assigned during
         * the robot configuration step.
         */
        frontLeftMotor =
                hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftMotor =
                hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightMotor =
                hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightMotor =
                hardwareMap.get(DcMotor.class, "back_right_motor");

        intake =
                hardwareMap.get(DcMotor.class, "intake_motor");
        launcher =
                hardwareMap.get(DcMotorEx.class, "outtake_motor");

        windmillServo =
                hardwareMap.get(CRServo.class, "transfer_servo");
        leftIntakeServo =
                hardwareMap.get(CRServo.class, "left_intake");
        rightIntakeServo =
                hardwareMap.get(CRServo.class, "right_intake");

        /*
         * To drive forward, most robots need the motors on one side reversed,
         * because the axles point in opposite directions.
         *
         * These settings match the modified starter TeleOp. Adjust individual
         * motor directions based on your first test drive.
         *
         * Note: The settings here assume direct drive on left and right wheels.
         * Gear reductions or 90-degree drives may require direction flips.
         */
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        driveMotors = new DcMotor[]{
                frontLeftMotor,
                backLeftMotor,
                frontRightMotor,
                backRightMotor
        };

        /*
         * Here we reset the encoders on our drive motors before we start moving.
         * After resetting, we select a normal driving mode so the motors are
         * not left in STOP_AND_RESET_ENCODER.
         */
        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode."
         * This causes the motor to slow down much faster when it is coasting.
         * This creates a much more controllable drivetrain.
         */
        for (DcMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(BRAKE);
        }

        intake.setZeroPowerBehavior(BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         *
         * If you notice that you have no control over the velocity of the motor,
         * and it jumps much higher than your set point, make sure the encoder
         * cable is plugged into the port beside that motor's power connection.
         *
         * Also check that the motor polarity is consistent through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5)
        );

        launcher.setVelocity(0);

        /*
         * Set feeders to an initial value to initialize the servo controller.
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo
         * to reverse so both servos work to pull elements into the intake.
         */
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT,
     * but before they hit START.
     */
    @Override
    public void init_loop() {
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        autoTimer.reset();
        driveTimer.reset();
    }

    /*
     * This code runs REPEATEDLY after the driver hits START,
     * but before they hit STOP.
     */
    @Override
    public void loop() {

        /*
         * TECH TIP: Switch Statements
         *
         * Switch statements are an excellent way to take advantage of an enum.
         * They work very similarly to a series of "if" statements, but allow
         * for cleaner and more readable code.
         *
         * We switch between each enum member and write the code that should
         * run when our enum reflects that state.
         *
         * We end each case with "break" to skip checking the rest of the cases.
         */
        switch (autonomousState) {
            case LAUNCH:
                launch(true);

                if (autoTimer.seconds() > 10) {
                    launch(false);
                    intakePower = 0;

                    driveTimer.reset();
                    autonomousState = AutonomousState.DRIVE;
                }
                break;

            case DRIVE:
                if (drive(DRIVE_SPEED, -120, DistanceUnit.MM, 2)) {
                    stopDrive();
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            case COMPLETE:
                stopDrive();
                telemetry.addLine("Auto Complete!");
                break;
        }

        intake.setPower(intakePower);

        /*
         * Here is our telemetry that keeps us informed of what is going on
         * in the robot.
         *
         * Since this part of the code exists outside of our switch statement,
         * it will run once every loop, no matter what state our robot is in.
         *
         * This is the huge advantage of using state machines. We can have code
         * inside our state machine that runs only when necessary, and code
         * after the last case that runs every loop.
         *
         * This means we can avoid a lot of copy-and-paste that non-state-machine
         * autonomous routines fall into.
         */
        telemetry.addData("AutoState", autonomousState);

        telemetry.addData(
                "Motor Current Positions",
                "FL %d | BL %d | FR %d | BR %d",
                frontLeftMotor.getCurrentPosition(),
                backLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(),
                backRightMotor.getCurrentPosition()
        );

        telemetry.addData(
                "Motor Target Positions",
                "FL %d | BL %d | FR %d | BR %d",
                frontLeftMotor.getTargetPosition(),
                backLeftMotor.getTargetPosition(),
                frontRightMotor.getTargetPosition(),
                backRightMotor.getTargetPosition()
        );

        telemetry.update();
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
        stopDrive();

        launcher.setVelocity(0);
        intake.setPower(0);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
    }

    void launch(boolean input) {
        if (input) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (input && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);

            // Set the assist power instead of adding to it every loop.
            intakePower = 0.5;
        } else {
            windmillServo.setPower(0);
            intakePower = 0;
        }
    }

    /**
     * @param speed From 0-1.
     * @param distance In the specified unit.
     * @param distanceUnit The unit of measurement for distance.
     * @param holdSeconds Seconds to remain at position before returning true.
     * @return True if all four motors remain within tolerance of their targets
     *         for more than holdSeconds; false otherwise.
     */
    boolean drive(
            double speed,
            double distance,
            DistanceUnit distanceUnit,
            double holdSeconds
    ) {
        final double TOLERANCE_MM = 10;

        /*
         * In this function we use DistanceUnit. This is a class that the FTC SDK
         * implements which allows us to accept different input units depending
         * on the user's preference.
         *
         * To use this, put both a double and a DistanceUnit in the parameters,
         * then call distanceUnit.toMm(distance). This returns the number of
         * millimeters equivalent to the specified distance.
         *
         * We are working in millimeters here, so that is the unit we request.
         * DistanceUnit can also convert to inches when needed.
         *
         * These targets are measured from the encoder zero established at INIT.
         */
        double targetPosition =
                distanceUnit.toMm(distance) * TICKS_PER_MM;

        frontLeftMotor.setTargetPosition((int) targetPosition);
        backLeftMotor.setTargetPosition((int) targetPosition);
        frontRightMotor.setTargetPosition((int) targetPosition);
        backRightMotor.setTargetPosition((int) targetPosition);

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.min(1.0, Math.abs(speed)));
        }

        /*
         * Here we check if we are within tolerance of our target position.
         * We calculate the absolute error, regardless of whether it is positive
         * or negative, and compare it to our tolerance.
         *
         * If any motor has not reached its target, we reset driveTimer.
         * Only after all four motors reach their targets can the timer count
         * higher than our holdSeconds variable.
         */
        for (DcMotor motor : driveMotors) {
            double error = Math.abs(
                    (double) motor.getTargetPosition()
                            - motor.getCurrentPosition()
            );

            if (error > TOLERANCE_MM * TICKS_PER_MM) {
                driveTimer.reset();
                return false;
            }
        }

        return driveTimer.seconds() > holdSeconds;
    }

    /**
     * @param speed From 0-1.
     * @param angle The amount the robot should rotate.
     * @param angleUnit The unit that angle is in.
     * @param holdSeconds Seconds to remain at position before returning true.
     * @return True if all four motors remain within tolerance of their targets
     *         for more than holdSeconds; false otherwise.
     */
    boolean rotate(
            double speed,
            double angle,
            AngleUnit angleUnit,
            double holdSeconds
    ) {
        final double TOLERANCE_MM = 10;

        /*
         * Here we estimate the millimeters that our drive wheels need to cover
         * to create the requested angle.
         *
         * We use radians because it makes the math simpler. In the starter's
         * differential-drive model, the robot rotates one radian when its wheels
         * travel half the track width along their turning circles.
         *
         * To find the requested wheel travel, multiply the angle in radians
         * by this radius.
         *
         * This is an approximation for mecanum wheels and needs tuning.
         */
        double targetMm =
                angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2.0);

        /*
         * We set both left motors to the inverse of the right-side target
         * so the robot rotates instead of driving straight.
         *
         * Like drive(), these are absolute encoder targets measured from INIT.
         */
        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        frontLeftMotor.setTargetPosition((int) leftTargetPosition);
        backLeftMotor.setTargetPosition((int) leftTargetPosition);
        frontRightMotor.setTargetPosition((int) rightTargetPosition);
        backRightMotor.setTargetPosition((int) rightTargetPosition);

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.min(1.0, Math.abs(speed)));
        }

        // All four motors must reach their targets before the hold can finish.
        for (DcMotor motor : driveMotors) {
            double error = Math.abs(
                    (double) motor.getTargetPosition()
                            - motor.getCurrentPosition()
            );

            if (error > TOLERANCE_MM * TICKS_PER_MM) {
                driveTimer.reset();
                return false;
            }
        }

        return driveTimer.seconds() > holdSeconds;
    }

    // Stop all four independently mapped drive motors.
    private void stopDrive() {
        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
        }
    }
}