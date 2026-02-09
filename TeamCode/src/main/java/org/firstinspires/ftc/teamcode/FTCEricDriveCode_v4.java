package org.firstinspires.ftc.teamcode;

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

@TeleOp(name="FTCEricdrivecode_v3")
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
    private Timer actionTimer;
    public static double speedReducer = 0.75;

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
    public static int transferBump = 250;
    public int intakePosition = 0;
    public static double scoreZone = 1;
    public static double p_turn = 1;
    private boolean launch = false;
    public static double flyspeed2 = 1580;
    public static double flyspeed3 = 1500;
    public static double flyspeed4 = 1500;
    public static double flyspeed5 = 1550;
    public static double i0 = 0;
    public static double t0 = i0;
    public static double i1 = 1;
    public static double t1 = t0 + i1;
    //interval for transfer to run and throw the second ball into the flywheel
    public static double i2 = 1.4;
    public static double t2 = t1 + i2;
    //interval to move the third ball into position
    public static double i3 = 1.25;
    public static double t3 = t2 + i3;
    //interval for transfer to run and throw the third ball into the flywheel
    public static double i4 = 1;
    public static double t4 = t3 + i4;
    //interval to do nothing but before it all shuts down
    public static double i5 = 0.5;
    public static double t5 = t4 + i5;
    public static double launchTime = i0 + i1 + i2 + i3 + i4 + i5;

    int transferPosition = 0;
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
        kicker.setPosition(0);
        helper.setPosition(helper_open);

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

                if (llResult.getTx() < -3  ) {
                    turnCorrection = -0.25;
                }

                else if (llResult.getTx() > 0 ) {
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

                FL.setPower(-.2);
                FR.setPower(.2);
                BL.setPower(-.2);
                BR.setPower(.2);
            }
            if (gamepad1.dpad_right) {

                FL.setPower(.2);
                FR.setPower(-.2);
                BL.setPower(.2);
                BR.setPower(-.2);
            }
            if (gamepad1.dpad_up) {

                FL.setPower(.2);
                FR.setPower(.2);
                BL.setPower(.2);
                BR.setPower(.2);
            }
            if (gamepad1.dpad_down) {

                FL.setPower(-.2);
                FR.setPower(-.2);
                BL.setPower(-.2);
                BR.setPower(-.2);
            }




///////////////////FLYWHEEL CONTROLS///////////////////////////////////
            controller.setPID(p, i, d);
            double fly1Current = fly1.getVelocity();
            double fly2Current = fly2.getVelocity();
            double pid = controller.calculate(fly1Current, target);
            double pid2 = controller.calculate(fly2Current, target);

            fly1.setPower(pid);
            fly2.setPower(pid2);

            if (gamepad2.dpad_down) {
                launch = true;
                actionTimer.resetTimer();
            }

            if (launch) {
                launchArtifacts2();
            }
            else {
                LegServo.setPosition(servo_closed);
            }

            if (gamepad2.dpad_up) {
                launch = false;
            }

            if (gamepad2.right_trigger > .1) {
                LegServo.setPosition(servo_opened);
                target = close_launch_speed;



            }
            else if (gamepad2.left_trigger > .1) {
                LegServo.setPosition(servo_opened);
                target = long_launch_speed;


            }
            else if (gamepad2.x){
                LegServo.setPosition(servo_closed);
                target = backOffSpeed;
                transfer.setPower(-1);
            }


            if(gamepad2.y) {
                resetRuntime();
                while (getRuntime() < kickTime) {
                    kicker.setPosition(kicker_kick);
                }
            }



///////////////////TRANSFER CONTROLS///////////////////////////////////

            helper.setPosition(helper_open);

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
            else {
                transfer.setPower(0);
            }


///////////////////INTAKE CONTROLS///////////////////////////////////



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
    public void launchArtifacts2() {
        //spin up the flywheel for long enough to launch three artifacts
        LegServo.setPosition(servo_opened);
        kicker.setPosition(0.185);
        helper.setPosition(0.75);
        int tPos;


        if(actionTimer.getElapsedTimeSeconds() < launchTime) {
            //at the start of the sequence; corrects aiming

            fly1.setVelocity(flyspeed4);
            fly2.setVelocity(flyspeed4);

//lets flywheel charge up

//            while(Math.abs(fly1.getVelocity()-flySpeed)<40){
//
//            }
//first interval is to kick the first ball into the flywheel
            if (actionTimer.getElapsedTimeSeconds() > t0 && actionTimer.getElapsedTimeSeconds() < t1) {
                kicker.setPosition(0);

            }

//next interval is to run the transfer-only to move the second ball into position
            if (actionTimer.getElapsedTimeSeconds() > t1 && actionTimer.getElapsedTimeSeconds() < t2) {
                kicker.setPosition(0.185);
                transfer.setPower(1);

            }

            //next interval is to kick the second ball into the flywheel
            if(actionTimer.getElapsedTimeSeconds() > t2 && actionTimer.getElapsedTimeSeconds() < t3) {
                kicker.setPosition(0);
            }


//next interval is to move the third ball into position
            if(actionTimer.getElapsedTimeSeconds() > t3 && actionTimer.getElapsedTimeSeconds() < t4) {
                helper.setPosition(0.4);
                kicker.setPosition(0.185);
                intake.setPower(1);
                transfer.setPower(1);

            }

//last interval is to kick the third ball into the flywheel
            if(actionTimer.getElapsedTimeSeconds() > t4 && actionTimer.getElapsedTimeSeconds() < t5) {
                intake.setPower(0);
                helper.setPosition(0.75);
                transfer.setPower(0);
                kicker.setPosition(0);

            }
//            while(actionTimer.getElapsedTimeSeconds() > t4 && actionTimer.getElapsedTimeSeconds() < t5) {
//                kicker.setPosition(0);
//
//            }
        }
        //once you're done scoring, shut it all down!
        if (actionTimer.getElapsedTimeSeconds()> t5) {
            intake.setPower(0);
            kicker.setPosition(0.185);
            transfer.setPower(0);
            helper.setPosition(0.75);
            fly1.setPower(0);
            fly2.setPower(0);
            fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LegServo.setPosition(servo_closed);
            launch = false;
        }
    }
    public void waitTimer(double time) {
        actionTimer.resetTimer();
        while (actionTimer.getElapsedTimeSeconds() < time) {
        }
    }
}
