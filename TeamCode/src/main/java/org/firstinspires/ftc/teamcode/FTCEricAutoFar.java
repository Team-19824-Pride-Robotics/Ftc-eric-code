package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Configurable
public class FTCEricAutoFar extends LinearOpMode {
    double seconds = 0;
    private static double driveTime = 1;
    private static double power = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables.

        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");

        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        timer.startTime();

        while (timer.seconds() < driveTime) {

            BL.setPower(power);
            BR.setPower(power);
            FL.setPower(power);
            FR.setPower(power);
        }
            BL.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            FR.setPower(0);

    }

}
