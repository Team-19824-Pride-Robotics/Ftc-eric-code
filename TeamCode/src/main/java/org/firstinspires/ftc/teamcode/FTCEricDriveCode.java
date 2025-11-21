package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="FTCEricdrivecode")
@Configurable

public class FTCEricDriveCode extends LinearOpMode {



    private Servo LegServo;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor transfer;
    private DcMotor intake;
    private DcMotorEx fly1;
    private DcMotorEx fly2;
    private Limelight3A limelight;
//    private IMU imu;


    //Declare variables
    public static double speedReducer = 0.75;
    public static double backOffSpeed = -600;
    public static double long_launch_speed = 2110;
    public static double close_launch_speed = 1650;
    public static double intakeOn = 0.75;
    public static int transferBump = 250;
    double fly1Speed = 0;
    double fly2Speed = 0;
    int transferPosition = 0;



    @Override
    public void runOpMode() {

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
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        fly1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        transfer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(8); //this is the april tag
//        imu = hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
//        imu.initialize((new IMU.Parameters(revHubOrientationOnRobot)));




       // Wait for the game to start (driver presses START)
        waitForStart();

       //limelight.start();


 //////////////This is the start of the loop///////////////////////////

        while (opModeIsActive()) {


///////////////////DRIVE CONTROLS///////////////////////////////////


            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = ((y + x + rx) / denominator);
            double leftBackPower = ((y - x + rx) / denominator);
            double rightFrontPower = ((y - x - rx) / denominator);
            double rightBackPower = ((y + x - rx) / denominator);

            FL.setPower(leftFrontPower * speedReducer);
            BL.setPower(leftBackPower * speedReducer);
            FR.setPower(rightFrontPower * speedReducer);
            BR.setPower(rightBackPower * speedReducer);


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
                fly1Speed = backOffSpeed;
                fly2Speed = backOffSpeed;
                LegServo.setPosition(0.3);
            }
            else {
                LegServo.setPosition(0.3);
                fly1Speed = 0;
                fly2Speed = 0;
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

            if (gamepad2.a || gamepad1.a) {
                intake.setPower(intakeOn);
            }
            else if(gamepad2.b || gamepad1.b) {
                intake.setPower(-0.4);
            }
            else {
                intake.setPower(0);
            }

            if (gamepad2.dpad_left || gamepad1.dpad_left) {

                FL.setPower(-.2);
                FR.setPower(.2);
                BL.setPower(-.2);
                BR.setPower(.2);
            }
            if (gamepad2.dpad_right || gamepad1.dpad_right) {

                FL.setPower(.2);
                FR.setPower(-.2);
                BL.setPower(.2);
                BR.setPower(-.2);
            }
            if (gamepad2.dpad_up || gamepad1.dpad_up) {

                FL.setPower(.2);
                FR.setPower(.2);
                BL.setPower(.2);
                BR.setPower(.2);
            }
            if (gamepad2.dpad_down || gamepad1.dpad_down) {

                FL.setPower(-.2);
                FR.setPower(-.2);
                BL.setPower(-.2);
                BR.setPower(-.2);
            }



///////////////////MOTOR CONTROLS///////////////////////////////////

            fly1.setVelocity(fly1Speed);
            fly2.setVelocity(fly2Speed);



            telemetry.addData("Transfer Position", transferPosition);
            telemetry.addData("Short Target Velocity", close_launch_speed);
            telemetry.addData("Long Target Velocity", long_launch_speed);
            telemetry.addData("Current Velocity", fly1.getVelocity());
            telemetry.addData("Current Velocity", fly2.getVelocity());
            telemetry.addData("flywheel1 power", fly1.getPower());
            telemetry.addData("flywheel2 power", fly2.getPower());
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
