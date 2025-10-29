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






public class FTCEricDriveCode extends LinearOpMode {

    // Declare OpMode members.

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
    private IMU imu;


    //Declare variables

    public static double backOffSpeed = -600;
    public static double transferback = -1;
    public static double long_launch_speed = 2110;
    public static double close_launch_speed = 1650;
    public static double intakeOn = 1;
    public static double transferOn = 1;
    double fly1Speed = 0;
    double fly2Speed = 0;
    int intakePosition = 0;
    double transferSpeed = 0;
    double transferRuntime = 3;
    double Balldown = 0.6;
    int transferPosition;

    int indexPosition;
    final double TURN_GAIN = 0.01;
    final double MAX_AUTO_TURN = 0.3;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables.

        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        BR.setDirection(DcMotor.Direction.REVERSE);

        
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        fly1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(8); //this is the april tag
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize((new IMU.Parameters(revHubOrientationOnRobot)));

       // Wait for the game to start (driver presses START)
        waitForStart();

       //limelight.start();


 //////////////This is the start of the loop///////////////////////////

        while (opModeIsActive()) {


///////////////////DRIVE CONTROLS///////////////////////////////////


            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double rx = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double x = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            FL.setPower(leftFrontPower);
            BL.setPower(leftBackPower);
            FR.setPower(rightFrontPower);
            BR.setPower(rightBackPower);


///////////////////FLYWHEEL CONTROLS///////////////////////////////////

            if (gamepad2.right_trigger > .1) {
                fly1Speed = close_launch_speed;
                fly2Speed = close_launch_speed;
            }
            else if (gamepad2.left_trigger > .1) {
                fly1Speed = long_launch_speed;
                fly2Speed = long_launch_speed;

            }else if (gamepad2.x){
                fly1Speed = backOffSpeed;
                fly2Speed = backOffSpeed;
            } else {
                fly1Speed = 0;
                fly2Speed = 0;
            }


///////////////////TRANSFER CONTROLS///////////////////////////////////
            if(gamepad2.y){
                    transferPosition = transfer.getCurrentPosition() + 30;

            }else if(gamepad2.x){
                transferPosition = transfer.getCurrentPosition() - 30;
            }else{
                transferPosition = transfer.getCurrentPosition();
            }
            if (gamepad2.start){

                LegServo.setPosition(Balldown);
            }
            else {

                LegServo.setPosition(0);
            }
            //ARUA


            ///////////////////INTAKE CONTROLS///////////////////////////////////

            if (gamepad2.a || gamepad1.a) {
                intakePosition = intake.getCurrentPosition() + 30;


            }
            else if(gamepad2.b || gamepad1.b) {
                intakePosition = intake.getCurrentPosition() + 30;


            }
            else {

                intakePosition = intake.getCurrentPosition();
            }


///////////////////MOTOR CONTROLS///////////////////////////////////

            fly1.setVelocity(fly1Speed);
            fly2.setVelocity(fly2Speed);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setTargetPosition(intakePosition);
            transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            transfer.setTargetPosition(transferPosition);
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
}
