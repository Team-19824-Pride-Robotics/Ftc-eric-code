package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name="FTCericdrivecode")


public class FTCericdrivecode extends LinearOpMode {



    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private DcMotor intake;

    private DcMotorEx fly1;
    private DcMotorEx fly2;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        BL = hardwareMap.get(DcMotor.class, "leftBack");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1Motor");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2Motor");
        fly1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        double fly1Speed = 0;

        double fly2Speed = 0;


        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);


// Wait for the game to start (driver presses START)

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

// Denominator is the largest motor power (absolute value) or 1
// This ensures all the powers maintain proportionality while staying <= 1
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator; // Corrected formula
            double rightBackPower = (y + x - rx) / denominator; // Corrected formula

            FL.setPower(leftFrontPower);
            BL.setPower(leftBackPower);
            FR.setPower(rightFrontPower);
            BR.setPower(rightBackPower);

            if (gamepad1.a) {
                fly1Speed = 1;
                fly2Speed = 1;
            } else if (gamepad1.b) {
                // This makes 'b' the button to stop the flywheels
                fly1Speed = 0;
                fly2Speed = 0;
            }
            if (gamepad1.right_trigger > 0.1) {

                intake.setPower(1);
            }
            if (gamepad1.left_trigger > 0.1) {

                intake.setPower(0);
            }
            if (gamepad1.left_bumper) {
                fly1.setPower(-0.1);
                fly2.setPower(-0.1);

            }
            if (gamepad1.right_bumper) {

                intake.setPower(-0.1);
            }


            fly1.setPower(fly1Speed);
            fly2.setPower(fly2Speed);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Flywheel 1 Velocity (ticks/sec)", fly1.getVelocity());
            telemetry.addData("Flywheel 2 Velocity (ticks/sec)", fly2.getVelocity());

            telemetry.update();
        }
    }  
}