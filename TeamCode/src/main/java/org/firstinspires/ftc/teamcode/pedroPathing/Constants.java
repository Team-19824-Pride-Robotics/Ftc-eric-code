package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .forwardZeroPowerAcceleration(-45.8691)
            .lateralZeroPowerAcceleration(-41.3835)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    2,
                    0,
                    0.01,
                    0.03))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.075, .07483889, .001702737))
            .centripetalScaling(0);



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("BL")
            .rightRearMotorName("FL")
            .leftRearMotorName("FR")
            .leftFrontMotorName("BR")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)//This reverses the robots front right motor
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)//This reverses the robots back RIGHT motor
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)// reverses the front right motor
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)// reverses the front left motor
            .xVelocity(67.55524)
            .yVelocity(49.928276);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.4,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

    }

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.71954)
            .strafePodX(-0.555202)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

}