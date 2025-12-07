package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import dev.nextftc.core.commands.Command;
import dev.nextftc.hardware.powerable.Powerable;
import dev.nextftc.hardware.powerable.SetPower;


@Configurable
public class Flywheel {


    public MotorEx fly1;
    public MotorEx fly2;
    public Servo LegServo;
    public PIDFController controller = new PIDFController(0.005, 0.0, 0.0, new StaticFeedforward(0.0));


    public static double intakeTime = 3;


    public void initialize() {

        fly1 = OpModeData.INSTANCE.getHardwareMap().get(MotorEx.class, "fly1");
        fly2 = OpModeData.INSTANCE.getHardwareMap().get(MotorEx.class, "fly2");
        LegServo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "LegServo");
    }


//    public Command intake_on() {
//
//        return new SetPower((Powerable) motor, 0.75);
//
//    }
//
//    public Command intake_off() {
//
//        return new SetPower((Powerable) motor, 0);
//
//    }

}