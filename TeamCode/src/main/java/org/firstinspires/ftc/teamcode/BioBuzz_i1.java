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

/// adds interpolation table; needs tuning
/// we're just so gracious and professional
@TeleOp(name="BioBuzz_i1")
@Configurable

public class BioBuzz_i1 extends LinearOpMode {

    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx BL;
    private DcMotorEx BR;


    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;
    double speedReducer = 1;

    @Override
    public void runOpMode() {


        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");


        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed! forwards n back
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing believe this is
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);


            leftFrontPower = ((y + x - rx) / denominator);
            leftBackPower = ((y - x - rx) / denominator);
            rightFrontPower = ((y - x + rx) / denominator);
            rightBackPower = ((y + x + rx) / denominator);

            FL.setPower(leftFrontPower * speedReducer);
            BL.setPower(leftBackPower * speedReducer);
            FR.setPower(rightFrontPower * speedReducer);
            BR.setPower(rightBackPower * speedReducer);

            //dPad can be used to make small corrections

            if (gamepad1.dpad_right || gamepad2.dpad_right) {

                FL.setPower(-.25);
                FR.setPower(.25);
                BL.setPower(-.25);
                BR.setPower(.25);
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {

                FL.setPower(.25);
                FR.setPower(-.25);
                BL.setPower(.25);
                BR.setPower(-.25);
            }
            if (gamepad1.dpad_up || gamepad2.dpad_up) {

                FL.setPower(.25);
                FR.setPower(.25);
                BL.setPower(.25);
                BR.setPower(.25);
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {

                FL.setPower(-.25);
                FR.setPower(-.25);
                BL.setPower(-.25);
                BR.setPower(-.25);
            }

            telemetry.addData("FR Velocity", FR.getVelocity());
            telemetry.addData("FL Velocity", FL.getVelocity());
            telemetry.addData("BR Velocity", BR.getVelocity());
            telemetry.addData("BL Velocity", BL.getVelocity());
        }
    }
}
//we love being gracious and professional
