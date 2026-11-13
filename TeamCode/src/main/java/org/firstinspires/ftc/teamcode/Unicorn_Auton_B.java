package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedHashMap;
import java.util.Map;

@Autonomous(name = "Unicorn Auton B")
public class Unicorn_Auton_B extends OpMode {
    // VARIABLES //
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    private DcMotor outtakeMotor;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    // the very important ElapsedTime which is very important until we actually can do auto calibration
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime stateTime = new ElapsedTime();

    // -------- State machine --------
    private enum State {
        DELAY_START, // init

        // Shared between both strats: you shoot immediately on game start and leave the hive.
        // Also, strat 1 (main) does 1.5 tips, strat 2 (opposite) does 1 tip.
        FIRST_SPIN_UP,
        FIRST_SHOOT,
        LEAVE_HIVE,

        // Main strat (from initially untipped hive): After the initial shots, we go pick up 4 pollen
        // to half tip the second hive. Experimentally, we will shoot from an angle as to not disturb
        // our teammate's auton.
        M_FLOWER_TURN,
        M_FLOWER_DRIVE,
        M_FLOWER_INTAKE,
        M_FLOWER_REVERSE,
        M_CROSSFIELD_ROTATE,
        M_CROSSFIELD,
        M_NEWPOS_TURN,
        M_SECOND_SPINUP,
        M_SECOND_SHOOT,

        // Delay strat (from initially tipped hive): After the initial shots, we need 4 more pollen
        // to tip the hive, so we go get it and then come back for a second shot.
        FIRST_FLOWER_TURN,
        DRIVE_TO_FLOWER,
        SECOND_FLOWER_TURN,
        APPROACH_FLOWER,
        COLLECT,
        RETURN_FROM_FLOWER,
        REVERSE_FLOWER_APPROACH,
        FIRST_RETURN_TURN,
        REVERSE_FLOWER_DRIVE,
        SECOND_RETURN_TURN,
        RETURN_TO_HIVE,
        SECOND_SPIN_UP,
        SECOND_SHOOT,
        DONE
    }

    private State state = State.DELAY_START;

    // list of tunables
    // (TEMP TUNES FOR INIT VERSION, REMOVE/MODIFY AFTER PID. this only does one hive, one flower (so one tip) then done)
    private static final Map<String, Double> TUNABLE_VALUES = new LinkedHashMap<>();

    static {
        TUNABLE_VALUES.put("DRIVE_POWER", 0.7);
        TUNABLE_VALUES.put("STRAT", 0.1); // strat, 0.1 is regular (no delay), 0.2 is delay
        TUNABLE_VALUES.put("ALLIANCE_ZERO_RED_ZEROONE_BLUE", 0.0); // alliance, 0.0 is red, 0.1 is blue
        TUNABLE_VALUES.put("INIT_MOVE_TIME", 2.0); // how long to move to 1st hive
        TUNABLE_VALUES.put("ROTATION_TIME", 1.3); // how long to rotate for (forwards and backwards)
        TUNABLE_VALUES.put("REVERSE_TIME", 0.5); // how long to reverse for
        TUNABLE_VALUES.put("REV_TIME", 2.0); // how long to rev for before turning the transfer
        TUNABLE_VALUES.put("TRANSFER_TIME", 5.0); // how long to run the transfer
        TUNABLE_VALUES.put("FLOWER_P1_TIME", 0.3); // how long to move forward toward the flower
        TUNABLE_VALUES.put("FLOWER_P2_TIME", 1.0); // how long to strafe while collecting from the flower
    }

    private static final String[] TUNABLES =
            TUNABLE_VALUES.keySet().toArray(new String[0]);

    // variable that lets you choose which setting you want to change
    private static int TUNABLE_SETTING = 0;

    // misc
    private boolean upWasPressed = false;
    private boolean downWasPressed = false;
    private boolean leftWasPressed = false;
    private boolean rightWasPressed = false;

    @Override
    public void init() {
        // HARDWAREMAP BUREAUCRACY //
        leftFrontDrive =
                hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive =
                hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive =
                hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive =
                hardwareMap.get(DcMotor.class, "right_back_drive");

        leftIntakeServo =
                hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo =
                hardwareMap.get(CRServo.class, "right_intake_servo");

        outtakeMotor =
                hardwareMap.get(DcMotor.class, "outtake_motor");
        intakeMotor =
                hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor =
                hardwareMap.get(DcMotor.class, "transfer_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stopAll();

        for (DcMotor motor : new DcMotor[]{
                leftFrontDrive, leftBackDrive,
                rightFrontDrive, rightBackDrive
        }) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up && !upWasPressed) {
            TUNABLE_SETTING = Math.min(TUNABLE_SETTING + 1, TUNABLES.length - 1);
        } else if (gamepad1.dpad_down && !downWasPressed) {
            TUNABLE_SETTING = Math.max(TUNABLE_SETTING - 1, 0);
        }

        upWasPressed = gamepad1.dpad_up;
        downWasPressed = gamepad1.dpad_down;

        String selected = TUNABLES[TUNABLE_SETTING];
        double value = TUNABLE_VALUES.get(selected);

        if (gamepad1.dpad_right && !rightWasPressed) {
            double max = selected.equals("DRIVE_POWER") ? 1.0
                    : selected.equals("ALLIANCE_ZERO_RED_ZEROONE_BLUE") ? 0.1 : Double.MAX_VALUE;
            value = Math.min(max, value + 0.1);
        } else if (gamepad1.dpad_left && !leftWasPressed) {
            value = Math.max(0.0, value - 0.1);
        }

        leftWasPressed = gamepad1.dpad_left;
        rightWasPressed = gamepad1.dpad_right;
        TUNABLE_VALUES.put(selected, value);

        telemetry.addData(selected, "%.1f", value);
        telemetry.addData("Alliance", TUNABLE_VALUES.get("ALLIANCE_ZERO_RED_ZEROONE_BLUE") >= 0.05
                ? "BLUE" : "RED");
        telemetry.addData("Strat", TUNABLE_VALUES.get("STRAT") >= 0.15 ? "2 - TIPPED SIDE" : "1 - UNTIPPED SIDE");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        stateTime.reset();
        state = State.DELAY_START;
    }

    @Override
    public void loop() {
        double power = TUNABLE_VALUES.get("DRIVE_POWER");

        switch (state) {
            // INIT STATE //
            case DELAY_START:
                transitionTo(State.FIRST_SPIN_UP);
                break;

            // Unused, keep in case we need it again.
//            case DRIVE_TO_HIVE:
//                runStep("INIT_MOVE_TIME", State.TURN_TO_HIVE, power, 0, 0, false, false, false);
//                break;
//            case TURN_TO_HIVE:
//                runStep("ROTATION_TIME", State.REVERSE_TO_HIVE, 0, 0, power, false, false, false);
//                break;
//            case REVERSE_TO_HIVE:
//                runStep("REVERSE_TIME", State.FIRST_SPIN_UP, -power, 0, 0, false, false, false);
//                break;

            // SHARED CODE //
            case FIRST_SPIN_UP:
                runStep("REV_TIME", State.FIRST_SHOOT, 0, 0, 0, false, true, false);
                break;
            case FIRST_SHOOT:
                runStep("TRANSFER_TIME", State.LEAVE_HIVE, 0, 0, 0, false, true, true);
                break;
            case LEAVE_HIVE:
                if (TUNABLE_VALUES.get("STRAT") >= 0.15) {
                    runStep("REVERSE_TIME", State.FIRST_FLOWER_TURN, power, 0, 0, false, false, false);
                } else {
                    runStep("REVERSE_TIME", State.M_FLOWER_TURN, power, 0, 0, false, false, false);
                }
                break;

            // STRAT 1 //
            case M_FLOWER_TURN:
                runStep("ROTATION_TIME", State.M_FLOWER_DRIVE, 0, 0, -power, false, false, false);
                break;
            case M_FLOWER_DRIVE:
                runStep("INIT_MOVE_TIME", State.M_FLOWER_INTAKE, power, 0, 0, false, false, false, 0.85);
                break;
            case M_FLOWER_INTAKE:
                runStep("FLOWER_P2_TIME", State.M_FLOWER_REVERSE, power*0.2, power, 0, true, false, false, 0.75);
                break;
            case M_FLOWER_REVERSE:
                runStep("FLOWER_P2_TIME", State.M_CROSSFIELD_ROTATE, -power*0.2, -power, 0, true, false, false, 0.75);
                break;
            case M_CROSSFIELD_ROTATE:
                runStep("ROTATION_TIME", State.M_CROSSFIELD, 0, 0, power, false, false, false);
                break;
            case M_CROSSFIELD:
                runStep("INIT_MOVE_TIME", State.M_NEWPOS_TURN, power, 0, 0, false, false, false, 1.8);
                break;
            case M_NEWPOS_TURN:
                runStep("ROTATION_TIME", State.M_SECOND_SPINUP, 0, 0, power, false, false, false, 1.33);
                break;
            case M_SECOND_SPINUP:
                runStep("REV_TIME", State.M_SECOND_SHOOT, 0, 0, 0, false, true, false);
                break;
            case M_SECOND_SHOOT:
                runStep("TRANSFER_TIME", State.DONE, 0, 0, 0, false, true, true);
                break;

            // STRAT 2 //
            case FIRST_FLOWER_TURN:
                runStep("ROTATION_TIME", State.DRIVE_TO_FLOWER, 0, 0, power, false, false, false);
                break;
            case DRIVE_TO_FLOWER:
                runStep("FLOWER_P1_TIME", State.SECOND_FLOWER_TURN, power, 0, 0, false, false, false);
                break;
            case SECOND_FLOWER_TURN:
                runStep("ROTATION_TIME", State.APPROACH_FLOWER, 0, 0, power, false, false, false);
                break;
            case APPROACH_FLOWER:
                runStep("REVERSE_TIME", State.COLLECT, power, 0, 0, false, false, false);
                break;
            case COLLECT:
                runStep("FLOWER_P2_TIME", State.RETURN_FROM_FLOWER, 0, -power, 0, true, false, false);
                break;
            case RETURN_FROM_FLOWER:
                runStep("FLOWER_P2_TIME", State.REVERSE_FLOWER_APPROACH, 0, power, 0, true, false, false);
                break;
            case REVERSE_FLOWER_APPROACH:
                runStep("REVERSE_TIME", State.FIRST_RETURN_TURN, -power, 0, 0, false, false, false);
                break;
            case FIRST_RETURN_TURN:
                runStep("ROTATION_TIME", State.REVERSE_FLOWER_DRIVE, 0, 0, -power, false, false, false);
                break;
            case REVERSE_FLOWER_DRIVE:
                runStep("FLOWER_P1_TIME", State.SECOND_RETURN_TURN, -power, 0, 0, false, false, false);
                break;
            case SECOND_RETURN_TURN:
                runStep("ROTATION_TIME", State.RETURN_TO_HIVE, 0, 0, -power, false, false, false);
                break;
            case RETURN_TO_HIVE:
                runStep("REVERSE_TIME", State.SECOND_SPIN_UP, -power, 0, 0, false, false, false);
                break;
            case SECOND_SPIN_UP:
                runStep("REV_TIME", State.SECOND_SHOOT, 0, 0, 0, false, true, false);
                break;
            case SECOND_SHOOT:
                runStep("TRANSFER_TIME", State.DONE, 0, 0, 0, false, true, true);
                break;
            case DONE:
                stopAll();
                break;
        }

        telemetry.addData("Run Time: ", runtime);
        telemetry.addData("State: ", state);
        telemetry.update();
    }

    private void runStep(String duration, State next, double forward, double strafe, double turn,
                         boolean intake, boolean outtake, boolean transfer, double durationMultiplier) {
        if (TUNABLE_VALUES.get(duration) != 0) {
            if (stateTime.seconds() >= TUNABLE_VALUES.get(duration) * durationMultiplier) {
                if (next != State.FIRST_SHOOT && next != State.SECOND_SHOOT) {
                    stopAll();
                }
                transitionTo(next);
                return;
            }

            double direction = TUNABLE_VALUES.get("ALLIANCE_ZERO_RED_ZEROONE_BLUE") >= 0.05 ? -1.0 : 1.0;
            strafe *= direction;
            turn *= direction;

            leftFrontDrive.setPower(forward + strafe + turn);
            leftBackDrive.setPower(forward - strafe + turn);
            rightFrontDrive.setPower(forward - strafe - turn);
            rightBackDrive.setPower(forward + strafe - turn);
            leftIntakeServo.setPower(intake ? -1.0 : 0.0);
            rightIntakeServo.setPower(intake ? 1.0 : 0.0);
            intakeMotor.setPower(intake ? 1.0 : 0.0);
            outtakeMotor.setPower(outtake ? 0.67 : 0.0);
            transferMotor.setPower(transfer ? 1.0 : 0.0);
        }
    }
    private void runStep(String duration, State next, double forward, double strafe, double turn,
                         boolean intake, boolean outtake, boolean transfer) {
        runStep(duration, next, forward, strafe, turn, intake, outtake, transfer, 1.0);
    }

    private void transitionTo(State next) {
        state = next;
        stateTime.reset();
    }

    private void stopAll() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    @Override
    public void stop() {
        stopAll();
    }
}