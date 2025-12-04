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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "auto_BLUESIDE")
@Configurable

public class auto_BLUESIDE extends OpMode {

    private DcMotorEx transfer;
    private DcMotor intake;
    private DcMotorEx fly1;
    private DcMotorEx fly2;

    public static double flySpeed = 1700;
    public static double launchTime = 5;
    public static double spinUpTime = 2;
    public static double intakeTime = 3;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(28, 130, Math.toRadians(136)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(55, 100, Math.toRadians(136)); // Scoring Pose of our robot. It is facing the goal at a 136 degree angle.
    private final Pose lineup1Pose = new Pose(44, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose gobble1Pose = new Pose(15, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose lineup2Pose = new Pose(44, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose gobble2Pose = new Pose(15, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.



    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantHeadingInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain.*/
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, lineup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading())
                .addPath(new BezierLine(lineup1Pose, gobble1Pose))
                .setConstantHeadingInterpolation(lineup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain.*/
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gobble1Pose, scorePose))
                .setLinearHeadingInterpolation(gobble1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain.*/
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, lineup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineup2Pose.getHeading())
                .addPath(new BezierLine(lineup2Pose, gobble2Pose))
                .setConstantHeadingInterpolation(lineup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain.*/
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(lineup2Pose, scorePose))
                .setLinearHeadingInterpolation(lineup2Pose.getHeading(), scorePose.getHeading())
                .build();

    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload
                    * Do we need to use a separate timer here to make it wait until we're done scoring???
                    * like reset pathTimer and then wait until enough time has passed before calling "followpath"?*/
                    launchArtifacts();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the artifacts */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Turn on the intake */
                    intakeArtifacts();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    launchArtifacts();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    intakeArtifacts();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    launchArtifacts();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(-1);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
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

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        fly1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    public void intakeArtifacts() {
        // Put your intake logic/functions here
        actionTimer.resetTimer();

        while(actionTimer.getElapsedTimeSeconds() < intakeTime) {
            intake.setPower(0.75);
        }

        intake.setPower(0);

    }
    public void launchArtifacts() {
        //spin up the flywheel for long enough to launch three artifacts
        actionTimer.resetTimer();

        while(actionTimer.getElapsedTimeSeconds() < launchTime) {
                fly1.setVelocity(flySpeed);
                fly2.setVelocity(flySpeed);

         //  **score first ball**  -->  wait till the flywheel is up to speed, then turn on the transfer but only for 0.25 seconds
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime && actionTimer.getElapsedTimeSeconds() < spinUpTime+0.25) {
                transfer.setPower(1);
            }
        //  **wait time**  one second??
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+0.25 && actionTimer.getElapsedTimeSeconds() < spinUpTime+1.25) {
                transfer.setPower(0);
            }
        //  **score second ball**  --> now turn on the intake and the transfer for 0.25 seconds
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+1.25 && actionTimer.getElapsedTimeSeconds() < spinUpTime+1.5) {
                intake.setPower(0.5);
                transfer.setPower(1);
            }
        //  **wait time**  one second??
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+1.5 && actionTimer.getElapsedTimeSeconds() < spinUpTime+2.5) {
                transfer.setPower(0);
            }
     //  **score third ball**  --> now turn on the intake and the transfer for 0.25 seconds
            while(actionTimer.getElapsedTimeSeconds() > spinUpTime+2.5 && actionTimer.getElapsedTimeSeconds() < spinUpTime+2.75) {
                intake.setPower(0.5);
                transfer.setPower(1);
            }

        }
    //once you're done scoring, shut it all down!
        intake.setPower(0);
        transfer.setPower(0);
        fly1.setPower(0);
        fly2.setPower(0);
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}


/*
public static class Paths {

  public PathChain scorepreloads;
  public PathChain lineupset1;
  public PathChain gobbleset1;
  public PathChain scoreset1;
  public PathChain lineupset2;
  public PathChain gobbleset2;
  public PathChain scoreset2;
  public PathChain park;

  public Paths(Follower follower) {
    scorepreloads = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(28.000, 130.000), new Pose(55.000, 100.000))
      )
      .setConstantHeadingInterpolation(Math.toRadians(136))
      .build();

    lineupset1 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(55.000, 100.000), new Pose(44.000, 84.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
      .build();

    gobbleset1 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(44.000, 84.000), new Pose(15.000, 84.000))
      )
      .setConstantHeadingInterpolation(Math.toRadians(180))
      .build();

    scoreset1 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(15.000, 84.000), new Pose(55.000, 100.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
      .build();

    lineupset2 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(55.000, 100.000), new Pose(44.000, 60.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
      .build();

    gobbleset2 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(44.000, 60.000), new Pose(15.000, 60.000))
      )
      .setConstantHeadingInterpolation(Math.toRadians(180))
      .build();

    scoreset2 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(15.000, 60.000), new Pose(55.000, 100.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
      .build();

    park = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(55.000, 100.000), new Pose(24.000, 70.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
      .build();
  }
}

 */
