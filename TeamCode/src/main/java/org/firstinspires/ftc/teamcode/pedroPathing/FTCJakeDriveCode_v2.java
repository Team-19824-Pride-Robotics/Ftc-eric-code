package org.firstinspires.ftc.teamcode.pedroPathing;

import android.content.pm.LauncherApps;

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

import org.firstinspires.ftc.teamcode.auto.launchAuto;
@Disabled
@TeleOp(name="FTCJakeDriveCode_v2")
@Configurable

public class FTCJakeDriveCode_v2 extends LinearOpMode {
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
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;





    private Timer actionTimer;
    public static double speedReducer = 1;
    ElapsedTime timer = new ElapsedTime();

    enum LaunchState {
        IDLE,
        SPINNING_UP,
        FEED,
        KICK,
        FIRE,
        RESET_SERVO,
        SETTLE,
        DONE
    }
    enum SecondLaunchState {
        IDLE,
        SPINNING_UP,
        FEED,
        KICK,
        FIRE,
        RESET_SERVO,
        DONE
    }
    enum FinalLaunchState {
        IDLE,
        SPINNING_UP,
        PUSH,
        FEED,
        KICK,
        RESET_SERVO,
        SETTLE,
        DONE
    }


    private FinalLaunchState finalLaunchState = FinalLaunchState.IDLE;
    private SecondLaunchState secondLaunchState = SecondLaunchState.IDLE;
    private LaunchState launchState = LaunchState.IDLE;
    private double stateStartTime = 0;

    public static double flyTolerance = 100;     // allowed velocity error
    public static double fireTime = 1;       // time gate is open
    public static double pushTime = 4;
    public static double resetTime = 0.5;      // time to close gate
    public static double settleTime = 2;     // allow artifact to settle
    public static double feedTime = 0.25;
    public static double kickUpTime = 0.25;

    boolean isBCurrentlyPressed = false;
    boolean isACurrentlyPressed = false;
    public static double kicker_kick = 0;
    public static double kicker_closed = 0.185;
    public static double kickTime = 0.25;

    public static double backOffSpeed = -600;
    public static double long_launch_speed = 1850;
    public static double close_launch_speed = 1800;
    //for 3 at once combo deal
    public static double servo_closed = 0.2;
    public static double servo_opened = 0;
    public static double helper_open = 0.75;
    public static double helper_closed = 0.4;

    public static double intakeOn = 1;
    public static int transferBump1 = 3000;
    public static int intakeBump1 = 3000;
    public static int transferBump2 = 3000;

    public int intakePosition = 0;
    public static double scoreZone = 1;
    public static double p_turn = 1;
    private boolean launch = false;
    private boolean multiSequenceActive = false;

    private boolean FinalArtifact = false;
    private boolean flywheel = false;
    private boolean killLaunch = false;
    private boolean isLaunching() {
        return launchState != LaunchState.IDLE || finalLaunchState != FinalLaunchState.IDLE || secondLaunchState != SecondLaunchState.IDLE;
    }

    private int launcher = 0;
    public static double flyspeed2 = 1580;
    public static double flyspeed3 = 1500;
    public static double flyspeed4 = 1500;
    public static double flyspeed5 = 1550;
    int transferStartPosition;
    double distance;
    double turnCorrection;
    InterpLUT lut = new InterpLUT();

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {




///////////////LOOKUP TABLE SETUP/////////////////////////
        lut.add(30, 950);
        lut.add(45, 950 + 300);
        lut.add(48, 1000 + 300);
        lut.add(56, 1100 + 300);
        lut.add(60, 1120 + 300);
        lut.add(70, 1140 + 300);
        lut.add(80, 1160 + 300);
        lut.add(90, 1180 + 300);
        lut.add(100, 1200 + 300);
        lut.add(110, 1260 + 300);
        lut.add(120, 1280 + 300);
        lut.add(130, 1340 + 300);
        lut.add(140, 1360 + 300);

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

        controller = new PIDController(p, i, d);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); //this is the april tag


        //set the initial position for the kicker and helper servos
        kicker.setPosition(kicker_closed);
        helper.setPosition(helper_open);
        LegServo.setPosition(servo_closed);

        launchState = LaunchState.IDLE;

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses START)
        waitForStart();

        limelight.start();


        /////////////////This is the start of the loop///////////////////////////

        while (opModeIsActive()) {


            boolean currentDpadLeft = gamepad2.dpad_left;
            boolean currentDpadDown = gamepad2.dpad_down;

            LaunchArtifacts();
            SecondLaunchArtifacts();
            FinalLaunchArtifacts();

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
                leftFrontPower = turnCorrection;
                leftBackPower = turnCorrection;
                rightFrontPower = -turnCorrection;
                rightBackPower = -turnCorrection;
                //otherwise drive normally
            } else {
                leftFrontPower = ((y + x + rx) / denominator);
                leftBackPower = ((y - x + rx) / denominator);
                rightFrontPower = ((y - x - rx) / denominator);
                rightBackPower = ((y + x - rx) / denominator);
            }
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

            if (currentDpadDown && !lastDpadDown && launchState == LaunchState.IDLE && finalLaunchState == FinalLaunchState.IDLE && secondLaunchState == SecondLaunchState.IDLE ) {
                if (launcher <= 1) {
                    launchState = LaunchState.SPINNING_UP;
                }
                else if (launcher == 2) {
                    secondLaunchState = SecondLaunchState.SPINNING_UP;
                }
                else if (launcher == 3) {
                    finalLaunchState = FinalLaunchState.SPINNING_UP;
                }

                stateStartTime = getRuntime();
            }
            lastDpadDown = currentDpadDown;
            ///  Launch system - all three
            if (currentDpadLeft && !lastDpadLeft && launchState == LaunchState.IDLE && finalLaunchState == FinalLaunchState.IDLE) {

                launcher = 0;
                multiSequenceActive = true;
                launchState = LaunchState.SPINNING_UP;
                stateStartTime = getRuntime();
            }
            lastDpadLeft = currentDpadLeft;


            if (gamepad2.dpad_up) {
                    killLaunch = true;
                    launchState = LaunchState.IDLE;
                    finalLaunchState = FinalLaunchState.IDLE;
                    launch = false;
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
            launch = false;
            multiSequenceActive = false;
            transfer.setPower(0);
            LegServo.setPosition(servo_closed);
            helper.setPosition(helper_open);
            return;
        }

        switch (launchState) {

            case IDLE:
                break;

            case SPINNING_UP:
                transferStartPosition = transfer.getCurrentPosition();
                fly1.setVelocity(flyspeed4);
                fly2.setVelocity(flyspeed4);
                LegServo.setPosition(servo_opened);


                if (Math.abs(fly1.getVelocity() - flyspeed4) < flyTolerance) {
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
                kicker.setPosition(kicker_kick);

                if (getRuntime() - stateStartTime > kickUpTime) {
                    launchState = LaunchState.RESET_SERVO;
                    stateStartTime = getRuntime();
                }
                break;


            case RESET_SERVO:

                LegServo.setPosition(servo_closed);
                kicker.setPosition(kicker_closed);

                if (getRuntime() - stateStartTime > resetTime) {
                    launchState = LaunchState.SETTLE;
                }
                break;


//            case INDEX_NEXT:
//
//                if (!transfer.isBusy()) {
//                    transfer.setPower(0);
//                    launchState = LaunchState.SETTLE;
//                    stateStartTime = getRuntime();
//                }
//                break;


            case SETTLE:

                transfer.setTargetPosition(transfer.getCurrentPosition() + transferBump2);
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                transfer.setPower(1);

                if (getRuntime() - stateStartTime > settleTime) {

                    launchState = LaunchState.DONE;
                }
                break;


            case DONE:
                LegServo.setPosition(servo_closed);
                helper.setPosition(helper_open);
                transfer.setPower(0);
                transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                launch = false;
                launcher++;
                launchState = LaunchState.IDLE;

                if (multiSequenceActive) {

                    if (launcher <= 1) {
                        launchState = LaunchState.SPINNING_UP;
                        stateStartTime = getRuntime();
                    }
                    else if (launcher == 2) {
                        secondLaunchState = SecondLaunchState.SPINNING_UP;
                        stateStartTime = getRuntime();
                    }
                    else if (launcher == 3){
                        finalLaunchState = FinalLaunchState.SPINNING_UP;
                        stateStartTime = getRuntime();
                    }
                }

                break;

        }
    }
    public void SecondLaunchArtifacts() {

        if (killLaunch) {
            secondLaunchState = SecondLaunchState.IDLE;
            launch = false;
            multiSequenceActive = false;
            transfer.setPower(0);
            LegServo.setPosition(servo_closed);
            helper.setPosition(helper_open);
            return;
        }

        switch (secondLaunchState) {

            case IDLE:
                break;

            case SPINNING_UP:
                transferStartPosition = transfer.getCurrentPosition();
                fly1.setVelocity(flyspeed4);
                fly2.setVelocity(flyspeed4);
                LegServo.setPosition(servo_opened);


                if (Math.abs(fly1.getVelocity() - flyspeed4) < flyTolerance) {
                    secondLaunchState = SecondLaunchState.FEED;
                    stateStartTime = getRuntime();
                }
                break;


            case FEED:
                transfer.setPower(1);

                if (getRuntime() - stateStartTime > feedTime) {
                    transfer.setPower(0);
                    secondLaunchState = SecondLaunchState.KICK;
                    stateStartTime = getRuntime();
                }
                break;

            case KICK:
                kicker.setPosition(kicker_kick);

                if (getRuntime() - stateStartTime > kickUpTime) {
                    secondLaunchState = SecondLaunchState.RESET_SERVO;
                    stateStartTime = getRuntime();
                }
                break;


            case RESET_SERVO:

                LegServo.setPosition(servo_closed);
                kicker.setPosition(kicker_closed);

                if (getRuntime() - stateStartTime > resetTime) {
                    secondLaunchState = SecondLaunchState.DONE;
                }
                break;


//            case INDEX_NEXT:
//
//                if (!transfer.isBusy()) {
//                    transfer.setPower(0);
//                    launchState = LaunchState.SETTLE;
//                    stateStartTime = getRuntime();
//                }
//                break;
            case DONE:
                LegServo.setPosition(servo_closed);
                helper.setPosition(helper_open);
                transfer.setPower(0);
                transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                launch = false;
                launcher++;
                secondLaunchState = SecondLaunchState.IDLE;

                if (multiSequenceActive && launcher == 3) {
                    finalLaunchState = FinalLaunchState.SPINNING_UP;
                    stateStartTime = getRuntime();
                }

                break;

        }
    }
    public void FinalLaunchArtifacts() {
        if (killLaunch) {
            finalLaunchState = FinalLaunchState.IDLE;
            launch = false;
            multiSequenceActive = false;
            transfer.setPower(0);
            LegServo.setPosition(servo_closed);
            helper.setPosition(helper_open);
            return;
        }

        switch (finalLaunchState) {

            case IDLE:
                break;

            case SPINNING_UP:
                transferStartPosition = transfer.getCurrentPosition();
                fly1.setVelocity(flyspeed4);
                fly2.setVelocity(flyspeed4);
                LegServo.setPosition(servo_opened);
                intake.setPower(1);

                if (Math.abs(fly1.getVelocity() - flyspeed4) < flyTolerance) {
                    finalLaunchState = FinalLaunchState.PUSH;
                    stateStartTime = getRuntime();
                }
                break;

            case PUSH:
                helper.setPosition(helper_closed);
            intake.setTargetPosition(intake.getCurrentPosition() + intakeBump1);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(1);
            transfer.setTargetPosition(transfer.getCurrentPosition() + transferBump1);
            transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            transfer.setPower(1);

                if (!intake.isBusy() && !transfer.isBusy()) {
                    intake.setPower(0);
                    transfer.setPower(0);
                    helper.setPosition(helper_open);
                    finalLaunchState = FinalLaunchState.FEED;
                    stateStartTime = getRuntime();
                }
             break;

            case FEED:
                transfer.setPower(1);

                if (getRuntime() - stateStartTime > feedTime) {
                    transfer.setPower(0);
                    finalLaunchState = FinalLaunchState.KICK;
                    stateStartTime = getRuntime();
                }
                break;

            case KICK:
                kicker.setPosition(kicker_kick);

                if (getRuntime() - stateStartTime > kickUpTime) {
                    finalLaunchState = FinalLaunchState.RESET_SERVO;
                    stateStartTime = getRuntime();
                }
                break;

            case RESET_SERVO:
                LegServo.setPosition(servo_closed);
                kicker.setPosition(kicker_closed);

                if (getRuntime() - stateStartTime > resetTime) {
                    finalLaunchState = FinalLaunchState.SETTLE;
                    stateStartTime = getRuntime();
                }
                break;

            case SETTLE:
                transfer.setTargetPosition(transfer.getCurrentPosition() + transferBump2);
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                transfer.setPower(1);

                if (getRuntime() - stateStartTime > settleTime) {
                    finalLaunchState = FinalLaunchState.DONE;
                }
                break;

            case DONE:
                LegServo.setPosition(servo_closed);
                helper.setPosition(helper_open);
                transfer.setPower(0);
                transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                launch = false;
                launcher = 0;   // reset after final shot
                finalLaunchState = FinalLaunchState.IDLE;
                multiSequenceActive = false;
                intake.setPower(0);
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
    }
}
