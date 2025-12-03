package org.firstinspires.ftc.teamcode.auto;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Configurable
public class launchAuto extends LinearOpMode {

    double seconds = 0;
    private static double driveTime = 0.75;

    private static double power = 0.5;
    private DcMotorEx transfer;
    private DcMotor intake;
    private DcMotorEx fly1;
    private DcMotorEx fly2;
    private Servo LegServo;


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables.

        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        LegServo = hardwareMap.get(Servo.class, "LegServo");

        fly1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        timer.startTime();
        while (timer.seconds() > 0 && timer.seconds() < 2) {
            fly1.setVelocity(2800);
            fly2.setVelocity(2800);
            LegServo.setPosition(.3);
        }
        while (timer.seconds() > 2 && timer.seconds() < 2.25) {

            transfer.setPower(1);
        }
        while (timer.seconds() > 2.25 && timer.seconds() < 3.25) {

            transfer.setPower(0);
        }
        while (timer.seconds() > 3.25 && timer.seconds() < 3.50) {

            transfer.setPower(1);
            intake.setPower(1);
        }

        ///////////////// Second Repeat //////////////////////
        while (timer.seconds() > 4.25 && timer.seconds() < 5.25) {

            ///////////////// Second Reapeat //////////////////////
            while (timer.seconds() > 2.25 && timer.seconds() < 3.25) {


                transfer.setPower(0);
            }
            while (timer.seconds() > 3.25 && timer.seconds() < 3.50) {

                transfer.setPower(1);
                LegServo.setPosition(0);


            }

            while (timer.seconds() > 3.50 && timer.seconds() < 4.25) {

                BL.setPower(-power);
                BR.setPower(power);
                FL.setPower(power);
                FR.setPower(power);
            }
            BL.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            FR.setPower(0);
            transfer.setPower(0);
            intake.setPower(0);

        }

    }
}
