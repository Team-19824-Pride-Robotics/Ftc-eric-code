package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "auto_REDSIDE")
@Configurable

public class auto_REDSIDE extends OpMode {

    private DcMotorEx transfer;
    private DcMotor intake;
    private DcMotorEx fly1;
    private DcMotorEx fly2;
    private Servo LegServo;
    private Servo kicker;
    private Servo helper;

   





    public static double intake_full = 1;
    public static double servo_closed = 0;
    public static double robotFast = 0.6;
    public static double robotSlow = 0.5;
    public static double robotSlower = 0.3;
    public double intake_state = 0;
    public double transfer_state = 0;
    public static double scorePos = 44;
    public static double scorePos2 = 45;
    public static double scorePos3 = 46;
    public static int tChange1 = 100;
    public static int tChange2 = 160;
    public static int tChange3 = 300;
    public static double flySpeed = 1200;
    public static double flySpeed2 = 1300;


////////timings for launchArtifacts function/////////////

//interval for initial kick into flywheel
    public static double i0 = 0;
    public static double t0 = i0;
    public static double i1 = 2;
    public static double t1 = t0 + i1;
//interval for transfer to run and throw the second ball into the flywheel
    public static double i2 = 1.4;
    public static double t2 = t1 + i2;
//interval to move the third ball into position
    public static double i3 = 0.5;
    public static double t3 = t2 + i3;
//interval for transfer to run and throw the third ball into the flywheel
    public static double i4 = 1;
    public static double t4 = t3 + i4;
//interval to do nothing but before it all shuts down
    public static double i5 = 0.5;
    public static double t5 = t4 + i5;

    public static double launchTime = i0 + i1 + i2 + i3 + i4 + i5;


    public static double t6 = t5 + 0.1;
    public static double t7 = t6 + 0.3;
    public static double t8 = t7 + 0.2;
    public static double t9 = t8 + 0.2;
    public static double t10 = t9 + 0.7;



    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(118, 128, Math.toRadians(43)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(100, 100, Math.toRadians(scorePos)); // Scoring Pose of our robot. It is facing the goal at a 136 degree angle.
    private final Pose lineup1Pose = new Pose(100, 85.5, Math.toRadians(0));
    private final Pose lineup1_5Pose = new Pose(40, 86.5, Math.toRadians(0));// Highest (First Set)
    private final Pose lineup1_6Pose = new Pose(45, 86.5, Math.toRadians(0));
    private final Pose gobble1Pose = new Pose(119.5, 85.5, Math.toRadians(0)); // Highest (First Set)
    private final Pose lineup2Pose = new Pose(100, 62, Math.toRadians(0)); // Middle (Second Set)
    private final Pose gobble2Pose = new Pose(125, 62, Math.toRadians(0)); // Middle (Second Set)
    private final Pose scorePose2 = new Pose(100, 100, Math.toRadians(scorePos2));
    private final Pose lineup2_5Pose = new Pose (40,62, Math.toRadians(0));
    private final Pose lineup2_6Pose = new Pose (45,62, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(100, 100, Math.toRadians(scorePos3));
    private final Pose lineup3Pose = new Pose(100, 41, Math.toRadians(0)); // Middle (Second Set)
    private final Pose gobble3Pose = new Pose(125, 41, Math.toRadians(0)); // Middle (Second Set)

    private PathChain scorePreload, lineup1, getFirstBall1, backOff1, getTwo1, getLast1, grabPickup1, scorePickup1,lineup2, getFirstBall2, backOff2, getTwo2, getLast2, grabPickup2, scorePickup2, grabPickup3, scorePickup3, justPark;


    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();


        lineup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, lineup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(lineup1Pose, gobble1Pose))
                .setConstantHeadingInterpolation(lineup1Pose.getHeading())
                .build();

        getFirstBall1 = follower.pathBuilder()
                .addPath(new BezierLine(lineup1Pose, lineup1_5Pose))
                .setConstantHeadingInterpolation(lineup1Pose.getHeading())
                .build();

        backOff1 = follower.pathBuilder()
                .addPath(new BezierLine(lineup1_5Pose, lineup1_6Pose))
                .setConstantHeadingInterpolation(lineup1_5Pose.getHeading())
                .build();


        getLast1 = follower.pathBuilder()

                .addPath(new BezierLine(lineup1_6Pose, gobble1Pose))
                .setConstantHeadingInterpolation(lineup1_6Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gobble1Pose, scorePose2))
                .setLinearHeadingInterpolation(gobble1Pose.getHeading(), scorePose2.getHeading())
                .build();

        lineup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, lineup2Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), lineup2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(lineup2Pose, gobble2Pose))
                .setConstantHeadingInterpolation(lineup2Pose.getHeading())
                .build();


        getFirstBall2 = follower.pathBuilder()
                .addPath(new BezierLine(lineup2Pose, lineup2_5Pose))
                .setConstantHeadingInterpolation(lineup2Pose.getHeading())
                .build();

        backOff2 = follower.pathBuilder()
                .addPath(new BezierLine(lineup2_5Pose, lineup2_6Pose))
                .setConstantHeadingInterpolation(lineup2_5Pose.getHeading())
                .build();


        getLast2 = follower.pathBuilder()

                .addPath(new BezierLine(lineup2_6Pose, gobble2Pose))
                .setConstantHeadingInterpolation(gobble2Pose.getHeading())
                .build();


        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(lineup2Pose, scorePose))
                .setLinearHeadingInterpolation(lineup2Pose.getHeading(), scorePose3.getHeading())
                .build();



        grabPickup3 = follower.pathBuilder()

                .addPath(new BezierLine(scorePose, lineup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineup3Pose.getHeading())
                .addPath(new BezierLine(lineup3Pose, gobble3Pose))
                .setConstantHeadingInterpolation(lineup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(lineup3Pose, scorePose))
                .setLinearHeadingInterpolation(lineup3Pose.getHeading(), scorePose3.getHeading())
                .build();

        justPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, gobble1Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), gobble1Pose.getHeading())
                .build();

    }



    public void autonomousPathUpdate() {
        switch (pathState) {

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */


            case 0:
                LegServo.setPosition(0.2);
                follower.setMaxPower(robotFast);
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:

                if(!follower.isBusy()) {
                    kicker.setPosition(0.185);
                    LegServo.setPosition(0.2);
                    launchArtifacts();

                    follower.setMaxPower(robotFast);
                    follower.followPath(lineup1,true);

                    setPathState(2);
                }
                break;

            case 2:

                if(!follower.isBusy()) {
                    LegServo.setPosition(0);
                    intake_state = 0.75;
                    transfer_state = 0.5;
                    //follower.setMaxPower(robotSlower);
                    follower.followPath(grabPickup1,true);
                    setPathState(5);
                }
                break;

            case 3:

                if(!follower.isBusy()) {

                    intake_state = 0.8;
                    transfer_state = 0.30;

                    follower.setMaxPower(robotSlow);
                    follower.followPath(backOff1,true);
                    setPathState(4);
                }
                break;

            case 4:

                if(!follower.isBusy()) {

                    intake_state = 0.8;
                    transfer_state = 0;

                    follower.setMaxPower(robotSlow);
                    follower.followPath(getLast1,true);
                    setPathState(5);
                }
                break;
//gets into scoring position
            case 5:

                if(!follower.isBusy()) {
                    intake_state = 0.075;
                    transfer_state = 0;

                    follower.followPath(scorePickup1,true);

                    setPathState(6);
                }
                break;
//scores the balls after opening the servo and gets back in position to pick up the balls
            case 6:

                LegServo.setPosition(0.2);
                if(!follower.isBusy()) {
                    launchArtifacts();

                    setPathState(7);
                }
                break;
            case 7:

                if(!follower.isBusy()) {
                    kicker.setPosition(0.185);
                    LegServo.setPosition(0);
                    //follower.setMaxPower(robotSlow);
                    follower.followPath(lineup2,true);

                    setPathState(8);
                }
                break;

            case 8:

                if(!follower.isBusy()) {
                    intake_state = 0.75;
                    transfer_state = 0.5;
                    //follower.setMaxPower(robotSlow);
                    follower.followPath(grabPickup2,true);
                    setPathState(11);
                }
                break;
//launches the balls, then sets the intake and transfer on, closes the servo and slows it down then it will pick up the balls
            case 9:

                if(!follower.isBusy()) {

                    intake_state = 0.8;
                    transfer_state = 0.30;

                    follower.setMaxPower(robotSlow);
                    follower.followPath(backOff2,true);
                    setPathState(10);
                }
                break;

            case 10:

                if(!follower.isBusy()) {

                    intake_state = 0.8;
                    transfer_state = 0;

                    follower.setMaxPower(robotSlow);
                    follower.followPath(getLast2,true);
                    setPathState(11);
                }
                break;


//gets into scoring position
            case 11:

                if(!follower.isBusy()) {
                    intake_state = 0.075;
                    transfer_state = 0;
                    follower.followPath(scorePickup2,true);

                    setPathState(12);
                }
                break;
//scores the balls after opening the servo and gets back in position to pick up the balls
            case 12:

                LegServo.setPosition(0);
                if(!follower.isBusy()) {
                    launchArtifacts();
                    //                  follower.setMaxPower(robotSlow);
                    //                   follower.followPath(grabPickup2,true);
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(justPark,true);
                    intake_state = 0;
                    transfer_state = 0;
                    setPathState(-1);
                }
                break;

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        intake.setPower(intake_state);
        transfer.setPower(transfer_state);
        fly1.setVelocity(flySpeed);
        fly2.setVelocity(flySpeed);


//        double p = 0.005, i = 0, d = 0;
//        PIDController controller = new PIDController(p, i, d);
//
//        controller.setPID(p, i, d);
//        double fly1Current = fly1.getVelocity();
//        double fly2Current = fly2.getVelocity();
//        double pid = controller.calculate(fly1Current, flySpeed);
//        double pid2 = controller.calculate(fly2Current, flySpeed);
//
//        fly1.setPower(pid);
//        fly2.setPower(pid2);
//
//        telemetry.addData("flywheel", fly1Current);
//        telemetry.update();

        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

       

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        fly1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        LegServo = hardwareMap.get(Servo.class, "LegServo");
        kicker = hardwareMap.get(Servo.class, "kicker");
        helper = hardwareMap.get(Servo.class, "helper");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        LegServo.setPosition(servo_closed);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}





    public Runnable intake_change(double power) {
        intake.setPower(power);
        return null;
    }




    public void launchArtifacts() {
        //spin up the flywheel for long enough to launch three artifacts
        actionTimer.resetTimer();
        LegServo.setPosition(0);
        kicker.setPosition(0.185);
        helper.setPosition(0.75);
        int tPos;

        while(actionTimer.getElapsedTimeSeconds() < launchTime) {

            fly1.setVelocity(flySpeed);
            fly2.setVelocity(flySpeed);

//lets flywheel charge up

//            while(Math.abs(fly1.getVelocity()-flySpeed)<40){
//
//            }
//first interval is to kick the first ball into the flywheel
            while(actionTimer.getElapsedTimeSeconds() > t0 && actionTimer.getElapsedTimeSeconds() < t1) {
             kicker.setPosition(0);
                fly1.setVelocity(flySpeed2);
                fly2.setVelocity(flySpeed2);
            }

//next interval is to run the transfer-only to move the second ball into position
            while(actionTimer.getElapsedTimeSeconds() > t1 && actionTimer.getElapsedTimeSeconds() < t2) {
                kicker.setPosition(0.185);
                transfer.setPower(0.8);

            }

            //next interval is to kick the second ball into the flywheel
            while(actionTimer.getElapsedTimeSeconds() > t2 && actionTimer.getElapsedTimeSeconds() < t3) {
                kicker.setPosition(0);
            }


//next interval is to move the third ball into position
            while(actionTimer.getElapsedTimeSeconds() > t3 && actionTimer.getElapsedTimeSeconds() < t4) {
                helper.setPosition(0.4);
                kicker.setPosition(0.185);
                intake.setPower(1);
                transfer.setPower(0.8);

            }

//last interval is to kick the third ball into the flywheel
            while(actionTimer.getElapsedTimeSeconds() > t4 && actionTimer.getElapsedTimeSeconds() < t5) {
                intake.setPower(0);
                helper.setPosition(0.75);
                transfer.setPower(0);
                kicker.setPosition(0);

            }
//            while(actionTimer.getElapsedTimeSeconds() > t4 && actionTimer.getElapsedTimeSeconds() < t5) {
//                kicker.setPosition(0);
//
//            }
        }
    //once you're done scoring, shut it all down!
        intake.setPower(0);
        kicker.setPosition(0.185);
        transfer.setPower(0);
        helper.setPosition(0.75);
        fly1.setPower(0);
        fly2.setPower(0);
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LegServo.setPosition(servo_closed);

    }

}



