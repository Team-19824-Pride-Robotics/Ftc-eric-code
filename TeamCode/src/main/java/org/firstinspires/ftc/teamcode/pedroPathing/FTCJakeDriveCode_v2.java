package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.launchAuto;

@TeleOp(name="FTCJakeDriveCode_v1")
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
    private Timer actionTimer;
    public static double speedReducer = 0.75;
    ElapsedTime timer = new ElapsedTime();

    enum LaunchState {
        IDLE,
        SPINNING_UP,
        FIRE,
        RESET_SERVO,
        INDEX_NEXT,
        SETTLE,
        DONE
    }

    private LaunchState launchState = LaunchState.IDLE;
    private double stateStartTime = 0;

    public static double flyTolerance = 40;     // allowed velocity error
    public static double fireTime = 0.20;       // time gate is open
    public static double resetTime = 0.15;      // time to close gate
    public static double settleTime = 0.10;     // allow artifact to settle

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
    public static int transferBump1 = 50;

    public static int transferBump2 = 250;
    public int intakePosition = 0;
    public static double scoreZone = 1;
    public static double p_turn = 1;
    private boolean launch = false;
    private boolean flywheel = false;
    public static double flyspeed2 = 1580;
    public static double flyspeed3 = 1500;
    public static double flyspeed4 = 1500;
    public static double flyspeed5 = 1550;
    public static double i0 = 0;
    public static double t0 = i0;
    public static double i1 = 0.25;
    public static double t1 = t0 + i1;
    //interval for transfer to run and throw the second ball into the flywheel
    public static double i2 = 0.5;
    public static double t2 = t1 + i2;
    //interval to move the third ball into position
    public static double i3 = 0.5;
    public static double t3 = t2 + i3;
    //interval for transfer to run and throw the third ball into the flywheel
    public static double i4 = 1;
    public static double t4 = t3 + i4;
    public static double i5 = 1;
    public static double t5 = t4 + i5;
    public static double i6 = 2;

    public static double t6 = t5+ i6;






    //interval to do nothing but before it all shuts down

    public static double launchTime = i0 + i1 + i2 + i3 + i4;

    int transferPosition1 = 0;

    int transferPosition2 = 0;
    double distance;
    double turnCorrection;
    InterpLUT lut = new InterpLUT();

    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {

///////////////LOOKUP TABLE SETUP/////////////////////////
        lut.add(30, 950 );
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
        LegServo.setPosition(servo_opened);



        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime.reset();
        actionTimer = new Timer();

       // Wait for the game to start (driver presses START)
        waitForStart();

       limelight.start();


 /////////////////This is the start of the loop///////////////////////////

        while (opModeIsActive()) {



//////////////////////LIMELIGHT SETUP//////////////////////////////////
            LLResult llResult = limelight.getLatestResult();



            if (llResult != null && llResult.isValid()) {

                distance = 67.82807 * Math.pow(llResult.getTa(), -0.5);

                if (distance > 150) {
                    distance = 40;
                }

                if (llResult.getTx() < -5  ) {
                    turnCorrection = -0.25;
                }

                else if (llResult.getTx() > 1 ) {
                    turnCorrection = 0.25;
                }

                //stop turning if you're facing the target (whether or not you can see AprilTag)
                else {
                    turnCorrection = 0;
                }
            }
            else {
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

            if (gamepad1.x){
                leftFrontPower = turnCorrection;
                leftBackPower = turnCorrection;
                rightFrontPower = -turnCorrection;
                rightBackPower = -turnCorrection;
                //otherwise drive normally
            }
            else{
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

            if (flywheel == false) {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }
            else {
                fly1.setPower(pid);
                fly2.setPower(pid2);
            }

            if (gamepad1.start || gamepad2.dpad_left) {
                flywheel = false;
            }
            else {
                flywheel = true;
            }

            if (gamepad2.dpad_down && launchState == LaunchState.IDLE) {
                launchState = LaunchState.SPINNING_UP;
                stateStartTime = getRuntime();
            }

            if (gamepad2.right_trigger > .1) {
                transfer.setPower(1);
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


            if(gamepad2.y) {
                resetRuntime();
                while (getRuntime() < kickTime) {
                    kicker.setPosition(kicker_kick);
                }
            }



///////////////////TRANSFER CONTROLS///////////////////////////////////



            if(gamepad2.left_bumper || gamepad1.left_bumper) {
                  transfer.setPower(1);
            }

            else if(gamepad2.right_bumper || gamepad1.right_bumper) {
                  transfer.setPower(-1);
            }

            else if (gamepad2.dpad_right || gamepad1.y) {
                transfer.setPower(1);
                resetRuntime();
                while (getRuntime() < kickTime) {
                    helper.setPosition(helper_closed);
                }

            }


///////////////////INTAKE CONTROLS///////////////////////////////////

            if (!launch) {
                kicker.setPosition(kicker_closed);
            }

            if (gamepad1.a || gamepad2.a || gamepad1.left_bumper || gamepad2.dpad_right) {

                isACurrentlyPressed = true;
            }
            else if (gamepad1.b || gamepad2.b || gamepad1.right_bumper) {

                isBCurrentlyPressed = true;
            }
            else {
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

            LaunchArtifacts();

            telemetry.addData("Launch State", launchState);
            telemetry.addData("Button Pressed", isACurrentlyPressed);
            telemetry.addData("Current Velocity", fly1.getVelocity());
            telemetry.addData("Current Velocity", fly2.getVelocity());
            telemetry.addData("Target Velocity", target);
            telemetry.update();
            
        }
    }


    public void runIntake(int newIntakePosition) {
        intake.setTargetPosition(newIntakePosition);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(1);
        while (intake.isBusy()) {
            //wait for the motor to reach its target position
        }
        transfer.setPower(0);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    ///LAUNCH ARTIFACTS///
    public void LaunchArtifacts() {

        switch (launchState) {

            case IDLE:
                break;

            case SPINNING_UP:

                fly1.setVelocity(flyspeed4);
                fly2.setVelocity(flyspeed4);
                LegServo.setPosition(servo_opened);


                if (Math.abs(fly1.getVelocity() - flyspeed4) < flyTolerance) {
                    launchState = LaunchState.FIRE;
                    stateStartTime = getRuntime();
                }
                break;


            case FIRE:

                transferPosition1 += transferBump1;
                transfer.setTargetPosition(transferPosition1);
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (getRuntime() - stateStartTime > fireTime) {
                    launchState = LaunchState.RESET_SERVO;
                    stateStartTime = getRuntime();
                }
                break;


            case RESET_SERVO:

                LegServo.setPosition(servo_closed);

                if (getRuntime() - stateStartTime > resetTime) {
                    launchState = LaunchState.INDEX_NEXT;

                    // Move transfer exactly one artifact forward
                    transferPosition2 += transferBump2;
                    transfer.setTargetPosition(transferPosition2);
                    transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;


            case INDEX_NEXT:

                if (!transfer.isBusy()) {
                    transfer.setPower(0);
                    launchState = LaunchState.SETTLE;
                    stateStartTime = getRuntime();
                }
                break;


            case SETTLE:

                if (getRuntime() - stateStartTime > settleTime) {
                    launchState = LaunchState.DONE;
                }
                break;


            case DONE:

                // Stay spun up if you want rapid fire
                // Or shut down flywheel here if desired

                launchState = LaunchState.IDLE;
                LegServo.setPosition(servo_closed);
                transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
        }
    }
//    public void waitTimer(double time) {
//        actionTimer.resetTimer();
//        while (actionTimer.getElapsedTimeSeconds() < time) {
//        }
//    }
}
