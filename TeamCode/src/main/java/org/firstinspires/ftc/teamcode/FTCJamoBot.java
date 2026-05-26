package org.firstinspires.ftc.teamcode;

//import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.auto_BLUESIDE_v5;

/// adds interpolation table; needs tuning
/// we're just so gracious and professional
@TeleOp(name="FTCJamoBot")
@Configurable

public class FTCJamoBot extends LinearOpMode {
    //we love being gracious and professional
//    private PIDController controller;
    public static double p = 0.005, i = 0, d = 0;
    public static double f = 0;
    public static double target = 1600;
    double flywheelTarget = 1600;
    private Servo blocker;
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx BL;
    private DcMotorEx BR;
    private DcMotor transfer;
    private DcMotor intake;
    private DcMotorEx fly1;
    private DcMotorEx fly2;
    private Limelight3A limelight;
    private IMU imu;

    //Declare variables
    boolean wasAButtonPressedLastLoop = false;
    boolean wasBButtonPressedLastLoop = false;
    private double lastValidFlywheelTarget = 1500;
    private double visionTimeout = 0.5;
    private ElapsedTime visionTimer = new ElapsedTime();
    private boolean ValidTarget = false;
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
    public static double flyTolerance = 50;     // allowed velocity error
    public static double resetTime = 0.2;
    public static double waitTime = 0.27;      // time to close gate
    // time to close gate
    public static double settleTime = 0.8;     // allow artifact to settle
    public static double feedTime = 0.11;
    public static double kickUpTime = 0.125;
    boolean isBCurrentlyPressed = false;
    boolean isACurrentlyPressed = false;
    public static double backOffSpeed = -600;
    public static double long_launch_speed = 1850;
    public static double close_launch_speed = 1800;


    //for 3 at once combo deal
    public static double servo_closed = 1;
    public static double servo_opened = 0.5;
    public static double servo_closed2 = 0.2;
    public static double servo_opened2 = 0;
    public static int transferBump1 = 1000;
    public static int intakeBump1 = 1000;
    public static int transferBump2 = 650;
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
    public static double flyspeed4 = 1750;
    public static double driveSpeed = 900;
    public static double flyspeed5 = 1550;
    int transferStartPosition;
    int intakeStartPosition;
    double distance;
    double turnCorrection;
    InterpLUT lut = new InterpLUT();
    public static int shortAddition = 450;

    public static int addition = 510;
    public static int longAddition = 50;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

///////////////LOOKUP TABLE SETUP/////////////////////////
        lut.add(-2000, 950);
        lut.add(30, 950);
        lut.add(35, 950 + shortAddition);
        lut.add (40, 975 + shortAddition);
        lut.add(45, 1000 + shortAddition);
        lut.add(50, 1550);
        lut.add(55, 1110 + addition);
        lut.add(60, 1125 + addition);
        lut.add(70, 1215 + addition);
        lut.add(80, 1200 + addition + longAddition);
        lut.add(90, 1220 + addition + longAddition);
        lut.add(100, 1210 + addition + longAddition);
        lut.add(110, 1425 + addition + longAddition);
        lut.add(120, 1475 + addition + longAddition);
        lut.add(125, 1480 + addition + longAddition);
        lut.add(130, 1500 + addition + longAddition);
        lut.add(140, 1550 + addition + longAddition);
        lut.add(150, 1650 + addition + longAddition);
        lut.add(10000000, 1700 + addition + longAddition);

        lut.createLUT();

        // Initialize the hardware variables.

        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");


        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        blocker = hardwareMap.get(Servo.class, "blocker");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");

        fly1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        fly2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        transfer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        controller = new PIDController(p, i, d);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); //this is the april tag


        //set the initial position for the kicker and helper servos
        blocker.setPosition(servo_closed);

        launchState = LaunchState.IDLE;

BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

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

                ValidTarget = true;
                visionTimer.reset();

                distance = 67.82807 * Math.pow(llResult.getTa(), -0.5);

                if (distance > 150) {
                    distance = Math.min(Math.max(distance, 30), 150);
                }


                flywheelTarget = lut.get(distance);              // LUT-based velocity
                flywheelTarget = Math.round(flywheelTarget / 10.0) * 10.0;

                double computedTarget = lut.get(distance);
                computedTarget = Math.round(computedTarget / 10.0) * 10.0;

                lastValidFlywheelTarget = computedTarget;

                if (llResult.getTx() < -3) {
                    turnCorrection = -0.25;
                } else if (llResult.getTx() > 3) {
                    turnCorrection = 0.25;
                }
                //stop turning if you're facing the target (whether or not you can see AprilTag)
                else {
                    ValidTarget = false;
                    turnCorrection = 0;
                }
            } else {
                turnCorrection = 0;
                distance = 60;
            }

/////////////////////////DRIVE CONTROLS///////////////////////////////////


            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed! forwards n back
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing believe this is
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double leftFrontPower;
            double leftBackPower;
            double rightFrontPower;
            double rightBackPower;

            if (gamepad1.x) {
                rx += turnCorrection; // add correction to rotation
            }

            leftFrontPower = ((y + x - rx) / denominator);
            leftBackPower = ((y - x - rx) / denominator);
            rightFrontPower = ((y - x + rx) / denominator);
            rightBackPower = ((y + x + rx) / denominator);

            FL.setPower(leftFrontPower * speedReducer);
            BL.setPower(leftBackPower * speedReducer);
            FR.setPower(rightFrontPower * speedReducer);
            BR.setPower(rightBackPower * speedReducer);

            //dPad can be used to make small corrections

            if (gamepad1.dpad_left) {

                FL.setPower(-.25);
                FR.setPower(.25);
                BL.setPower(-.25);
                BR.setPower(.25);
            }
            if (gamepad1.dpad_right) {

                FL.setPower(.25);
                FR.setPower(-.25);
                BL.setPower(.25);
                BR.setPower(-.25);
            }
            if (gamepad1.dpad_up) {

                FL.setPower(.25);
                FR.setPower(.25);
                BL.setPower(.25);
                BR.setPower(.25);
            }
            if (gamepad1.dpad_down) {

                FL.setPower(-.25);
                FR.setPower(-.25);
                BL.setPower(-.25);
                BR.setPower(-.25);
            }

///////////////////FLYWHEEL CONTROLS///////////////////////////////////

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
                blocker.setPosition(servo_closed);

            } else {
                killLaunch = false;
            }
/////////////////////////////////MAIN LAUNCH//////////////////////////////

            if (!isLaunching()) {

                fly1.setVelocity(flywheelTarget);
                fly2.setVelocity(flywheelTarget);

                if (gamepad2.left_bumper || gamepad2.right_bumper) {
                    blocker.setPosition(servo_opened);
                }
                else {
                    blocker.setPosition(servo_closed);
                }

                if (gamepad2.left_bumper || gamepad2.a) {
                    transfer.setPower(1);
                } else if (gamepad2.right_bumper || gamepad2.b) {
                    transfer.setPower(-1);
                } else {
                    transfer.setPower(0);
                }

                ///INTAKE
                if (gamepad2.a || gamepad2.left_bumper) {

                    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setPower(1);
                }
                else if (gamepad2.b || gamepad2.right_bumper) {

                    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setPower(-1);


                    if (gamepad2.right_bumper || gamepad1.right_bumper || gamepad2.a || gamepad2.b) {
///                          //Just another precaution//                                ///
                        launchState = LaunchState.IDLE;
                        multiSequenceActive = false;
                        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        transfer.setPower(-1);
                    }
                }
                else {
                    intake.setPower(0);
                }
            }   telemetry.addData("Limelight active: ", limelight.isRunning());
                telemetry.addData("Distance from Goal", distance);
                telemetry.addData("Launch State", launchState);
                telemetry.addData("Button Pressed", isACurrentlyPressed);
                telemetry.addData("Current Velocity", fly1.getVelocity());
                telemetry.addData("Current Velocity", fly2.getVelocity());
                telemetry.addData("Target Velocity", flywheelTarget);
                telemetry.addData("Launch Count", launcher);
                telemetry.addData("FR Velocity", FR.getVelocity());
                telemetry.addData("FL Velocity", FL.getVelocity());
                telemetry.addData("BR Velocity", BR.getVelocity());
                telemetry.addData("BL Velocity", BL.getVelocity());




            telemetry.update();


        }
    }

    /// /////////////////////////////////ALTERNATE LAUNCH///////////////////////////////////////////
    private void LaunchArtifacts() {
        if (killLaunch) {
            launchState = LaunchState.IDLE;
            multiSequenceActive = false;
            launcher = 0;
            transfer.setPower(0);
            intake.setPower(0);
            return;
        }

        switch (launchState) {

            case IDLE:
                break;

            case SPINNING_UP:
                fly1.setVelocity(flywheelTarget);
                fly2.setVelocity(flywheelTarget);
                blocker.setPosition(servo_opened);


                if (Math.abs(fly2.getVelocity() - flywheelTarget) < flyTolerance &&
                        getRuntime() - stateStartTime > 0.1) {
                        launchState = LaunchState.FEED;
                    stateStartTime = getRuntime();
                }

                break;

            case FEED: {
                intake.setPower(1);
                transfer.setPower(1);

                if (getRuntime() - stateStartTime > feedTime) {
                    transfer.setPower(0);
                    intake.setPower(0);
                    launchState = LaunchState.DONE;
                    stateStartTime = getRuntime();
                }
                break;
            }

            case DONE: {

                blocker.setPosition(servo_closed);

                transfer.setPower(0);
                transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intake.setPower(0);
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                launchState = LaunchState.IDLE;
                multiSequenceActive = false;
            }
        }
    }
}
