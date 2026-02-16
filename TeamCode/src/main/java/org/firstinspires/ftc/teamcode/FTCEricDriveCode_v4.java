package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/// this runs launch artifacts without the interpolation
@TeleOp(name="FTCEricDriveCode_v4")
@Configurable

public class FTCEricDriveCode_v4 extends LinearOpMode {
    //we love being gracious and professional
    private PIDController controller;
    public static double p = 0.005, i = 0, d = 0;
    public static double f = 0;
    public static double target = 1600;

    private Servo LegServo;
    private Servo kicker;
    private Servo helper;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor transfer;
    private DcMotor intake;
    private DcMotorEx fly1;
    private DcMotorEx fly2;
    private Limelight3A limelight;
    private IMU imu;


    //Declare variables
    boolean wasAButtonPressedLastLoop = false;
    boolean wasBButtonPressedLastLoop = false;
    boolean settleInitialized = false;
    boolean pushInitialized = false;
    boolean kickInitialized = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;


    private Timer actionTimer;
    public static double speedReducer = 1;
    ElapsedTime timer = new ElapsedTime();

    enum LaunchState {
        WAIT,
        IDLE,
        SPINNING_UP,
        PUSH_IF_FINAL,
        FEED,
        KICK,
        RESET_SERVO,
        SETTLE,
        DONE
    }
    private LaunchState launchState = LaunchState.IDLE;
    private double stateStartTime = 0;
    public static double flyTolerance = 70;     // allowed velocity error
    public static double resetTime = 0.25;
    public static double waitTime = 0.25;      // time to close gate
    // time to close gate
    public static double settleTime = 1;     // allow artifact to settle
    public static double feedTime = 0.125;
    public static double kickUpTime = 0.125;
    boolean isBCurrentlyPressed = false;
    boolean isACurrentlyPressed = false;
    public static double backOffSpeed = -600;
    public static double long_launch_speed = 1850;
    public static double close_launch_speed = 1800;
    public static double kicker_kick = 0;
    public static double kicker_closed = 0.185;
    public static double kickTime = 0.25;
    //for 3 at once combo deal
    public static double servo_closed = 0.2;
    public static double servo_opened = 0;
    public static double helper_open = 0.75;
    public static double helper_closed = 0.4;
    public static int transferBump1 = 1000;
    public static int intakeBump1 = 1000;
    public static int transferBump2 = 1000;
    public static int transferBump3 = 300;
    public int intakePosition = 0;
    private boolean multiSequenceActive = false;
    private boolean killLaunch = false;

    private void startLaunch() {
        launchState = LaunchState.SPINNING_UP;
        stateStartTime = getRuntime();
    }

    private boolean isLaunching() {
        return launchState != LaunchState.IDLE;
    }

    private int launcher = 0;
    public static double flyspeed2 = 1580;
    public static double flyspeed3 = 1500;
    public static double flyspeed4 = 1500;
    public static double flyspeed5 = 1550;
    int transferStartPosition;
    int intakeStartPosition;
    double distance;
    double turnCorrection;
    InterpLUT lut = new InterpLUT();
    int addition = 300;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {


///////////////LOOKUP TABLE SETUP/////////////////////////
        lut.add(30, 950);
        lut.add(35, 950 + addition);
        lut.add(45, 1000 + addition);
        lut.add(50, 1100 + addition);
        lut.add(55, 1120 + addition);
        lut.add(60, 1140 + addition);
        lut.add(70, 1160 + addition);
        lut.add(80, 1170 + addition);
        lut.add(90, 1180 + addition);
        lut.add(100, 1190 + addition);
        lut.add(110, 1200 + addition);
        lut.add(120, 1260 + addition);
        lut.add(130, 1280 + addition);
        lut.add(140, 1340 + addition);
        lut.add(150, 1360 + addition);

        lut.createLUT();

        // Initialize the hardware variables.

        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        LegServo = hardwareMap.get(Servo.class, "LegServo");
        kicker = hardwareMap.get(Servo.class, "kicker");
        helper = hardwareMap.get(Servo.class, "helper");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");

        fly1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        fly2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        transfer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDController(p, i, d);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); //this is the april tag


        //set the initial position for the kicker and helper servos
        kicker.setPosition(kicker_closed);
        helper.setPosition(helper_open);
        LegServo.setPosition(servo_closed);

        launchState = LaunchState.IDLE;
        // Wait for the game to start (driver presses START)
        waitForStart();

        limelight.start();


        /////////////////This is the start of the loop///////////////////////////

        while (opModeIsActive()) {


            boolean currentDpadLeft = gamepad2.dpad_left;
            boolean currentDpadDown = gamepad2.dpad_down;

            LaunchArtifacts();

//////////////////////LIMELIGHT SETUP//////////////////////////////////
            LLResult llResult = limelight.getLatestResult();


            if (llResult != null && llResult.isValid()) {

                distance = 67.82807 * Math.pow(llResult.getTa(), -0.5);

                if (distance > 150) {
                    distance = 40;
                }

                if (llResult.getTx() < -5) {
                    turnCorrection = -0.25;
                } else if (llResult.getTx() > 1) {
                    turnCorrection = 0.25;
                }

                //stop turning if you're facing the target (whether or not you can see AprilTag)
                else {
                    turnCorrection = 0;
                }
            } else {
                turnCorrection = 0;
                distance = 60;
            }

/////////////////////////DRIVE CONTROLS///////////////////////////////////


            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double leftFrontPower;
            double leftBackPower;
            double rightFrontPower;
            double rightBackPower;

            if (gamepad1.x) {
                rx += turnCorrection; // add correction to rotation
            }

            leftFrontPower = ((y + x + rx) / denominator);
            leftBackPower = ((y - x + rx) / denominator);
            rightFrontPower = ((y - x - rx) / denominator);
            rightBackPower = ((y + x - rx) / denominator);

            FL.setPower(leftFrontPower * speedReducer);
            BL.setPower(leftBackPower * speedReducer);
            FR.setPower(rightFrontPower * speedReducer);
            BR.setPower(rightBackPower * speedReducer);

            //dPad can be used to make small corrections

            if (gamepad1.dpad_left) {

                FL.setPower(-.15);
                FR.setPower(.15);
                BL.setPower(-.15);
                BR.setPower(.15);
            }
            if (gamepad1.dpad_right) {

                FL.setPower(.15);
                FR.setPower(-.15);
                BL.setPower(.15);
                BR.setPower(-.15);
            }
            if (gamepad1.dpad_up) {

                FL.setPower(.15);
                FR.setPower(.15);
                BL.setPower(.15);
                BR.setPower(.15);
            }
            if (gamepad1.dpad_down) {

                FL.setPower(-.15);
                FR.setPower(-.15);
                BL.setPower(-.15);
                BR.setPower(-.15);
            }


///////////////////FLYWHEEL CONTROLS///////////////////////////////////
            controller.setPID(p, i, d);
            double fly1Current = fly1.getVelocity();
            double fly2Current = fly2.getVelocity();
            double pid = controller.calculate(fly1Current, target);
            double pid2 = controller.calculate(fly2Current, target);

//            if (flywheel == false) {
//                fly1.setVelocity(0);
//                fly2.setVelocity(0);
//            } else {
            fly1.setPower(pid);
            fly2.setPower(pid2);


            /// launch system - 1 at a time

// Single shot
            if (currentDpadDown && !lastDpadDown && launchState == LaunchState.IDLE) {
                multiSequenceActive = false;
                startLaunch();
            }

// Triple shot
            if (currentDpadLeft && !lastDpadLeft && launchState == LaunchState.IDLE) {
                multiSequenceActive = true;
                launcher = 0;
                startLaunch();
            }

            lastDpadDown = currentDpadDown;
            lastDpadLeft = currentDpadLeft;

            if (gamepad2.dpad_up) {
                killLaunch = true;
                launchState = LaunchState.IDLE;
                multiSequenceActive = false;
                launcher = 0;
                transfer.setPower(0);
                LegServo.setPosition(servo_closed);
                helper.setPosition(helper_open);
            } else {
                killLaunch = false;
            }

            /// alternate launch


            if (gamepad2.right_trigger > .1) {
                LegServo.setPosition(servo_opened);
                target = close_launch_speed;
            }
//

//
//            }
//            else if (gamepad2.left_trigger > .1) {
//                LegServo.setPosition(servo_opened);
//                target = long_launch_speed;
//
//
//            }
//            else if (gamepad2.x){
//                LegServo.setPosition(servo_closed);
//                target = backOffSpeed;
//                transfer.setPower(-1);
//            }


            if (gamepad2.y) {
                resetRuntime();
                if (getRuntime() < kickTime) {
                    kicker.setPosition(kicker_kick);
                }
            }


///////////////////TRANSFER CONTROLS///////////////////////////////////

        if (!isLaunching()) {
            if (gamepad2.left_bumper || gamepad1.left_bumper) {
                transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                transfer.setPower(1);
            } else if (gamepad2.right_bumper || gamepad1.right_bumper) {
                transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                transfer.setPower(-1);
            } else {
                transfer.setPower(0);
            }
            if (gamepad2.dpad_right || gamepad1.y) {
                transfer.setPower(1);
                helper.setPosition(helper_closed);
            } else {
                helper.setPosition(helper_open);
            }
        }


///////////////////INTAKE CONTROLS///////////////////////////////////

            if (!isLaunching()) {
                if (gamepad1.a || gamepad2.a || gamepad1.left_bumper || gamepad2.dpad_right) {

                    isACurrentlyPressed = true;
                } else if (gamepad1.b || gamepad2.b || gamepad1.right_bumper) {

                    isBCurrentlyPressed = true;
                } else {
                    isACurrentlyPressed = false;
                    isBCurrentlyPressed = false;
                }

                if (isACurrentlyPressed && !wasAButtonPressedLastLoop) {

                    intakePosition = intakePosition + 600;
                }
                if (isBCurrentlyPressed && !wasBButtonPressedLastLoop) {

                    intakePosition = intakePosition - 400;
                }

                // Update the previous button state for the next loop iteration
                wasAButtonPressedLastLoop = isACurrentlyPressed;
                wasBButtonPressedLastLoop = isBCurrentlyPressed;

                intake.setTargetPosition(intakePosition);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(1);
            }


            telemetry.addData("Launch State", launchState);
            telemetry.addData("Button Pressed", isACurrentlyPressed);
            telemetry.addData("Current Velocity", fly1.getVelocity());
            telemetry.addData("Current Velocity", fly2.getVelocity());
            telemetry.addData("Target Velocity", target);
            telemetry.addData("Launch Count", launcher);
            telemetry.update();

        }
    }


//    public void runIntake(int newIntakePosition) {
//        intake.setTargetPosition(newIntakePosition);
//        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        intake.setPower(1);
//        while (intake.isBusy()) {
//            //wait for the motor to reach its target position
//        }
//        transfer.setPower(0);
//        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }


    /// LAUNCH ARTIFACTS///
    public void LaunchArtifacts() {

        if (killLaunch) {
            launchState = LaunchState.IDLE;
            multiSequenceActive = false;
            launcher = 0;
            transfer.setPower(0);
            intake.setPower(0);
            LegServo.setPosition(servo_closed);
            helper.setPosition(helper_open);
            return;
        }

        switch (launchState) {

            case IDLE:
                break;

            case SPINNING_UP:

                fly1.setVelocity(flyspeed4);
                fly2.setVelocity(flyspeed4);
                LegServo.setPosition(servo_opened);

                if (Math.abs(fly1.getVelocity() - flyspeed4) < flyTolerance) {

                    // If this is the 3rd shot, do PUSH
                    if (launcher == 2) {
                        launchState = LaunchState.PUSH_IF_FINAL;
                    } else {
                        launchState = LaunchState.FEED;
                    }

                    stateStartTime = getRuntime();
                }
                break;

            case PUSH_IF_FINAL:

                if (!pushInitialized) {
                    transferStartPosition = transfer.getCurrentPosition();
                    intakeStartPosition = intake.getCurrentPosition();
                    helper.setPosition(helper_closed);
                    intake.setTargetPosition(intakeStartPosition + intakeBump1);
                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setPower(1);
                    transfer.setTargetPosition(transferStartPosition + transferBump1);
                    transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    transfer.setPower(1);
                    pushInitialized = true;
                }

                if (!intake.isBusy() && !transfer.isBusy()) {
                    pushInitialized = false;
                    intake.setPower(0);
                    transfer.setPower(0);
                    helper.setPosition(helper_open);
                    launchState = LaunchState.FEED;
                    stateStartTime = getRuntime();
                }
                break;

            case FEED:

                transfer.setPower(1);

                if (getRuntime() - stateStartTime > feedTime) {
                    transfer.setPower(0);
                    launchState = LaunchState.KICK;
                    stateStartTime = getRuntime();
                }
                break;

            case KICK:

                if (!kickInitialized) {
                    transferStartPosition = transfer.getCurrentPosition();
                    transfer.setTargetPosition(transferStartPosition + transferBump3);
                    transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    transfer.setPower(1);
                    kicker.setPosition(kicker_kick);
                    kickInitialized = true;
                }

                if (getRuntime() - stateStartTime > kickUpTime || !transfer.isBusy()) {
                    kickInitialized = false;
                    launchState = LaunchState.RESET_SERVO;
                    stateStartTime = getRuntime();
                }
                break;

            case RESET_SERVO:

                LegServo.setPosition(servo_closed);
                kicker.setPosition(kicker_closed);

                if (getRuntime() - stateStartTime > resetTime) {
                    if (launcher >= 1) {
                        launchState = LaunchState.WAIT;
                        stateStartTime = getRuntime();
                    }
                    else {
                        launchState = LaunchState.SETTLE;
                        stateStartTime = getRuntime();
                    }
                }

                break;

            case WAIT:
             if (getRuntime() - stateStartTime > waitTime) {
                 launchState = LaunchState.DONE;
             }
                break;
            case SETTLE:

                if (!settleInitialized) {
                    transferStartPosition = transfer.getCurrentPosition();
                    transfer.setTargetPosition(transferStartPosition + transferBump2);
                    transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    transfer.setPower(1);
                    settleInitialized = true;
                }

                if (!transfer.isBusy()) {
                    settleInitialized = false;
                    launchState = LaunchState.DONE;
                }

                break;

            case DONE:

                LegServo.setPosition(servo_closed);
                helper.setPosition(helper_open);
                transfer.setPower(0);
                transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intake.setPower(0);
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                launcher++;

                // If multi-shot AND not yet at 3, continue
                if (multiSequenceActive && launcher < 3) {

                    launchState = LaunchState.SPINNING_UP;

                }
                else {

                    launchState = LaunchState.IDLE;
                    multiSequenceActive = false;

                    // Reset counter AFTER sequence ends
                    if (launcher >= 3) {
                        launcher = 0;
                    }
                }
                break;
        }
    }
}

