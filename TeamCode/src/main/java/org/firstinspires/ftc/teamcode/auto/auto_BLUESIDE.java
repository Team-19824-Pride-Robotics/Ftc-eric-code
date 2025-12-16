package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Intake;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "auto_BLUESIDE")
@Configurable

public class auto_BLUESIDE extends OpMode {

    private DcMotorEx transfer;
    private DcMotor intake;
    private DcMotorEx fly1;
    private DcMotorEx fly2;
    private Servo LegServo;
    private Servo kicker;





    public static double flySpeed = 1400;
    public static double flySpeed2 = 1400;
    public static double transferTime = 0.3;
    public static double launchTime = 5;
    public static double spinUpTime = 1.8;
    public static double intake_full = 1;
    public static double servo_closed = 0.4;
    public static double robotSpeed = 0.7;
    public static double robotSlow = 0.6;
    public double intake_state = 0;
    public double transfer_state = 0;
    public static double scorePos = 140;
    public static double scorePos2 = 140;
    public static double scorePos3 = 140;
    public static int tChange1 = 100;
    public static int tChange2 = 160;
    public static int tChange3 = 300;



    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(28, 130, Math.toRadians(136)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(55, 100, Math.toRadians(scorePos)); // Scoring Pose of our robot. It is facing the goal at a 136 degree angle.
    private final Pose lineup1Pose = new Pose(55, 87, Math.toRadians(180)); // Highest (First Set)
    private final Pose gobble1Pose = new Pose(18, 87, Math.toRadians(180)); // Highest (First Set)
    private final Pose lineup2Pose = new Pose(55, 64, Math.toRadians(180)); // Middle (Second Set)
    private final Pose gobble2Pose = new Pose(9, 64, Math.toRadians(180)); // Middle (Second Set)
    private final Pose scorePose2 = new Pose(55, 100, Math.toRadians(scorePos2));
    private final Pose scorePose3 = new Pose(55, 100, Math.toRadians(scorePos3));
    private final Pose lineup3Pose = new Pose(55, 43, Math.toRadians(180)); // Middle (Second Set)
    private final Pose gobble3Pose = new Pose(12, 43, Math.toRadians(180)); // Middle (Second Set)
    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;


    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        /* grabPickup1 PathChain --> lines up for the first set of artifacts, then
          turns on the intake and gobbles them up in a line  */

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, lineup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading())
                //.addTemporalCallback(1, intake_change(1))
                .addPath(new BezierLine(lineup1Pose, gobble1Pose))
                .setConstantHeadingInterpolation(lineup1Pose.getHeading())
                .build();

        /* scorePickup1 PathChain --> moves to the scoring position  */

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gobble1Pose, scorePose))
                .setLinearHeadingInterpolation(gobble1Pose.getHeading(), scorePose2.getHeading())
                .build();

        /* grabPickup2 PathChain --> lines up for the second set of artifacts, then
           turns on the intake and gobbles them up in a line  */

        grabPickup2 = follower.pathBuilder()

                .addPath(new BezierLine(scorePose, lineup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineup2Pose.getHeading())
                .addPath(new BezierLine(lineup2Pose, gobble2Pose))
                .setConstantHeadingInterpolation(lineup2Pose.getHeading())
                .build();

        /* scorePickup2 PathChain --> moves from the gobble2Pose back to the scoring position  */

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

    }



    public void autonomousPathUpdate() {
        switch (pathState) {

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

//set the speed and back up
            case 0:
                follower.setMaxPower(robotSpeed);  //slow down the path following if necessary
                follower.followPath(scorePreload);
                setPathState(1);
                break;
//Launches artifacts, then sets the transfer and intake on and sets the robot to slow. Then it will run grab pickup 1
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    /* score the preload artifacts by spinning the transfer */
                    launchArtifacts();

                    intake_state = 0.75;
                    transfer_state = 0.75;
                    follower.setMaxPower(robotSlow);
                    follower.followPath(grabPickup1,true);

                    setPathState(2);
                }
                break;
//picks up the balls, makes the robot fast again, and then gets into position to score
            case 2:

                if(!follower.isBusy()) {
                    intake_state = 0;
                    transfer_state = 0;
                    follower.setMaxPower(robotSpeed);
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
//launches the balls, then sets the intake and transfer on, closes the servo and slows it down then it will pick up the balls
            case 3:

                if(!follower.isBusy()) {

                    intake_state = 0;
                    transfer_state = 0;
                    /* Score the pickup1 artifacts */
                    launchArtifacts();
                    intake_state = 0.75;
                    transfer_state = 0.75;
                    LegServo.setPosition(servo_closed);
                    follower.setMaxPower(robotSlow);
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
//gets into scoring position
            case 4:

                if(!follower.isBusy()) {
                    intake_state = 0.5;
                    transfer_state = 0;

                    follower.followPath(scorePickup2,true);

                    setPathState(5);
                }
                break;
//scores the balls after opening the servo and gets back in position to pick up the balls
            case 5:
                follower.setMaxPower(robotSpeed);
                LegServo.setPosition(0);
                if(!follower.isBusy()) {
                    launchArtifacts();
                    intake_state = 0.75;
                    transfer_state = 0.75;
                    follower.setMaxPower(robotSlow);
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;

            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                        intake_state = 0.5;
                        transfer_state = 0;
                        follower.setMaxPower(robotSpeed);
                        follower.followPath(scorePickup3,true);


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    setPathState(7);
                }
                break;

            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    LegServo.setPosition(0);

                        launchArtifacts();
                        intake_state = 0.75;
                        transfer_state = 0.75;
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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
    int tPos;

        while(actionTimer.getElapsedTimeSeconds() < launchTime) {
                fly1.setVelocity(flySpeed2);
                fly2.setVelocity(flySpeed2);

         //  **score first ball**  -->  wait till the flywheel is up to speed, then turn on the transfer but only for 0.05 seconds
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime && actionTimer.getElapsedTimeSeconds() < spinUpTime+transferTime) {


                kicker.setPosition(0.15);
            }
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+0.3 && actionTimer.getElapsedTimeSeconds() < spinUpTime+1) {

                kicker.setPosition(0);
                transfer.setPower(0);
                intake.setPower(0);
            }


        //  **wait time**  one second??
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+1 && actionTimer.getElapsedTimeSeconds() < spinUpTime+1.1) {

                transfer.setPower(1);
                intake.setPower(1);

            }
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+1.1 && actionTimer.getElapsedTimeSeconds() < spinUpTime+1.3) {

                kicker.setPosition(0.15);

            }
        //  **score second ball**  --> now turn on the intake and the transfer for 0.25 seconds
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+1.3 && actionTimer.getElapsedTimeSeconds() < spinUpTime+2) {
                kicker.setPosition(0);
                transfer.setPower(0);
                intake.setPower(0);


            }


        //  **wait time**  one second??
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+2 && actionTimer.getElapsedTimeSeconds() < spinUpTime+2.1) {

                transfer.setPower(1);
                intake.setPower(1);

            }
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+2.1 && actionTimer.getElapsedTimeSeconds() < spinUpTime+2.3) {

                kicker.setPosition(0.15);

            }

     //  **score third ball**  --> now turn on the intake and the transfer for 0.25 seconds
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+2.3 && actionTimer.getElapsedTimeSeconds() < spinUpTime+3) {
                transfer.setPower(0);
                intake.setPower(0);
                kicker.setPosition(0);

            }


        }
    //once you're done scoring, shut it all down!
        intake.setPower(0);
        kicker.setPosition(0);
        transfer.setPower(0);
        fly1.setPower(0);
        fly2.setPower(0);
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LegServo.setPosition(servo_closed);

    }

}



