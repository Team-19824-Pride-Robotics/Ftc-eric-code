package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Configurable
public class Intake {

    private DcMotor intake;

    public static double intakeTime = 3;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }


    public void intake_on() {

        intake.setPower(0.75);

    }

    public void intake_off() {

        intake.setPower(0.75);

    }

}