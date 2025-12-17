package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="FTCEricdrivecode_v3")
@Configurable

public class FTCEricDriveCode_v3 extends LinearOpMode {



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
    public static double speedReducer = 0.75;

   public static boolean intakeOnly = false;
    public static double kicker_kick = 0.1;
    public static double kicker_closed = 0;
    public static double kickTime = 0.25;
    public static double backOffSpeed = -600;
    public static double long_launch_speed = 1700;
    public static double close_launch_speed = 1550;
    public static double servo_closed = 0.4;
    public static double helper_open = 0.75;
    public static double helper_closed = 0.4;

    public static double intakeOn = 1;
    public static int transferBump = 250;
    public static double scoreZone = 1;
    public static double p_turn = 1;
    double fly1Speed = 0;
    double fly2Speed = 0;
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
        

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); //this is the april tag


        //set the initial position for the kicker and helper servos
        kicker.setPosition(0);
        helper.setPosition(helper_open);

        runtime.reset();

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

                if (llResult.getTx() < -4) {
                    turnCorrection = -0.25;
                }

                else if (llResult.getTx() > 4) {
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

            //if pressing the right bumper, turn the robot towards the target
            if (gamepad1.right_bumper) {

                leftFrontPower = turnCorrection;
                leftBackPower = turnCorrection;
                rightFrontPower = -turnCorrection;
                rightBackPower = -turnCorrection;

            }

            //otherwise drive normally
            else {

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
//            if (gamepad2.dpad_left) {
//
//               transfer.setPower(0.1);
//            }

            helper.setPosition(helper_open);

            if (gamepad2.dpad_right || gamepad1.y) {
                transfer.setPower(1);
                resetRuntime();
                while (getRuntime() < kickTime) {
                    helper.setPosition(helper_closed);
                }

            }
            else{
                transfer.setPower(0);
            }



///////////////////FLYWHEEL CONTROLS///////////////////////////////////

            if (gamepad2.right_trigger > .1) {
                LegServo.setPosition(0);
                fly1Speed = close_launch_speed;
                fly2Speed = close_launch_speed;
            }
            else if (gamepad2.left_trigger > .1) {
                LegServo.setPosition(0);
                fly1Speed = long_launch_speed;
                fly2Speed = long_launch_speed;

            }
            else if (gamepad2.x){
                LegServo.setPosition(servo_closed);
                fly1Speed = backOffSpeed;
                fly2Speed = backOffSpeed;
                transfer.setPower(-1);
            }

            else {
                LegServo.setPosition(servo_closed);
                fly1Speed = 0;
                fly2Speed = 0;
            }

            fly1.setVelocity(fly1Speed);
            fly2.setVelocity(fly2Speed);

            kicker.setPosition(kicker_closed);

            if(gamepad2.y) {
                resetRuntime();
                while (getRuntime() < kickTime) {
                    kicker.setPosition(kicker_kick);
                }
            }



///////////////////TRANSFER CONTROLS///////////////////////////////////
            if(gamepad2.left_bumper) {
//                transferPosition += transferBump;
//                runTransfer(transferPosition);
                  transfer.setPower(1);
            }

            else if(gamepad2.right_bumper) {
//                transferPosition -= transferBump;
//                runTransfer(transferPosition);
                  transfer.setPower(-1);
            }
            else {
                transfer.setPower(0);
            }


///////////////////INTAKE CONTROLS///////////////////////////////////

            if (gamepad1.a || gamepad2.a) {
                intake.setPower(intakeOn);
                transfer.setPower(1);

            }
            else if (gamepad1.b || gamepad2.b) {
                intake.setPower(-1);
                transfer.setPower(-1);

            }
            else {
                intake.setPower(0);
            }

//            if (gamepad2.a && intakeOnly == true) {
//
//                intake.setPower(intakeOn);
//            }
//            else if (gamepad2.a && intakeOnly == false) {
//                intake.setPower(intakeOn);
//                transfer.setPower(1);
//            }
//            else if (gamepad2.b && intakeOnly == true) {
//
//                intake.setPower(-1);
//            }
//            else if (gamepad2.b && intakeOnly == false) {
//                intake.setPower(-1);
//                transfer.setPower(-1);
//            }
//            else {
//                intake.setPower(0);
//                transfer.setPower(0);
//            }


            telemetry.addData("Distance", distance);
            telemetry.addData("Current Velocity", fly1.getVelocity());
            telemetry.addData("Current Velocity", fly2.getVelocity());
            telemetry.addData("intakeOnly", intakeOnly);

            telemetry.update();
            
        }
    }


    public void runTransfer(int newTransferPosition) {
        transfer.setTargetPosition(newTransferPosition);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(1);
        while (transfer.isBusy()) {
            //wait for the motor to reach its target position
        }
        transfer.setPower(0);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
