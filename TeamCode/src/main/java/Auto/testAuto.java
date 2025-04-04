package Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "testAuto", group = "Examples")
public class testAuto extends OpMode {

    private Servo bigPivot = null;
    private Servo smallPivot = null;
    private CRServo crSmallPivot = null;
    private Servo claw = null;
    private Servo clawTwist = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, elapsedTime;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 65, Math.toRadians(0));
    private final Pose chamberPose = new Pose(39.5, 68, Math.toRadians(0));
    private final Pose waitPose = new Pose(39.4, 70, Math.toRadians(0));
    private final Pose pickUpPose = new Pose(16.5, 34, Math.toRadians(0));

    private final Pose goToSample1 = new Pose(56, 30, Math.toRadians(0));
    private final Pose goToSample1_controlPoint = new Pose(23, 40, Math.toRadians(0));
    private final Pose pushSampleIn1 = new Pose(26, 30, Math.toRadians(0));
    private final Pose goToSample2 = new Pose(59, 20, Math.toRadians(0));
    private final Pose goToSample2_controlPoint = new Pose(52, 35, Math.toRadians(0));
    private final Pose pushSampleIn2 = new Pose(27, 20, Math.toRadians(0));
    private final Pose goToSample3 = new Pose(58, 11, Math.toRadians(0));
    private final Pose goToSample3_controlPoint = new Pose(53, 22, Math.toRadians(0));
    private final Pose pushSampleIn3 = new Pose(16, 11, Math.toRadians(0));

    private final Pose scoreSpec2 = new Pose(38, 70, Math.toRadians(0));
    private final Pose scoreSpec2_controlPoint = new Pose(30, 72, Math.toRadians(0));
    private final Pose scoreSpec3 = new Pose(39.5, 72, Math.toRadians(0));
    private final Pose scoreSpec4 = new Pose(39.5, 74, Math.toRadians(0));
    private final Pose scoreSpec5 = new Pose(39.5, 76, Math.toRadians(0));
    private final Pose waitPose1 = new Pose(38.9, 72, Math.toRadians(0));
    private final Pose waitPose2 = new Pose(39.4, 71, Math.toRadians(0));
    private final Pose waitPose3 = new Pose(39.4, 74, Math.toRadians(0));
    private final Pose waitPose4 = new Pose(39.4, 77, Math.toRadians(0));
    private Path scorePreload, pickUpSpec, spec2, wait, grabSpec3, spec3, grabSpec4, wait2, spec4, wait3, grabSpec5, spec5, wait4, park, wait0;
    private PathChain samples, samples2, samples3;

    private boolean speedToggle = true;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        bigPivot = hardwareMap.get(Servo.class,"bigPivot");
        smallPivot = hardwareMap.get(Servo.class,"smallPivot");
        crSmallPivot = hardwareMap.get(CRServo.class,"crSmallPivot");
        claw = hardwareMap.get(Servo.class,"claw");
        clawTwist = hardwareMap.get(Servo.class,"clawTwist");


        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(chamberPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading());
        scorePreload.setPathEndTimeoutConstraint(10);

        wait0 =  new Path(new BezierLine(new Point(chamberPose), new Point(waitPose)));
        wait0.setLinearHeadingInterpolation(chamberPose.getHeading(), waitPose.getHeading());

        samples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(chamberPose), new Point(goToSample1_controlPoint), new Point(goToSample1)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), goToSample1.getHeading())
                .addPath(new BezierLine(new Point(goToSample1), new Point(pushSampleIn1)))
                .setLinearHeadingInterpolation(goToSample1.getHeading(), pushSampleIn1.getHeading())
                .setZeroPowerAccelerationMultiplier(13)
                .build();

        samples2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushSampleIn1), new Point(goToSample2_controlPoint), new Point(goToSample2)))
                .setLinearHeadingInterpolation(pushSampleIn1.getHeading(), goToSample2.getHeading())
                .addPath(new BezierLine(new Point(goToSample2), new Point(pushSampleIn2)))
                .setLinearHeadingInterpolation(goToSample2.getHeading(), pushSampleIn2.getHeading())
                .setZeroPowerAccelerationMultiplier(13)
                .build();

        samples3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushSampleIn2), new Point(goToSample3_controlPoint), new Point(goToSample3)))
                .setLinearHeadingInterpolation(pushSampleIn2.getHeading(), goToSample3.getHeading())
                .addPath(new BezierLine(new Point(goToSample3), new Point(pushSampleIn3)))
                .setLinearHeadingInterpolation(goToSample3.getHeading(), pushSampleIn3.getHeading())
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        spec2 = new Path(new BezierCurve(new Point(pushSampleIn3), new Point(scoreSpec2_controlPoint), new Point(scoreSpec2)));
        spec2.setLinearHeadingInterpolation(pushSampleIn3.getHeading(), scoreSpec2.getHeading());
        spec2.setPathEndTimeoutConstraint(100);

        wait = new Path(new BezierLine(new Point(scoreSpec2), new Point(waitPose1)));
        wait.setLinearHeadingInterpolation(scoreSpec2.getHeading(), waitPose1.getHeading());

        grabSpec3 = new Path(new BezierLine(new Point(waitPose1), new Point(pickUpPose)));
        grabSpec3.setLinearHeadingInterpolation(waitPose1.getHeading(), pickUpPose.getHeading());

        spec3 = new Path(new BezierLine(new Point(pickUpPose), new Point(scoreSpec3)));
        spec3.setLinearHeadingInterpolation(pickUpPose.getHeading(), scoreSpec3.getHeading());
        spec3.setPathEndTimeoutConstraint(100);

        wait2 = new Path(new BezierLine(new Point(scoreSpec3), new Point(waitPose2)));
        wait2.setLinearHeadingInterpolation(scoreSpec3.getHeading(), waitPose2.getHeading());

        grabSpec4 = new Path(new BezierLine(new Point(waitPose2), new Point(pickUpPose)));
        grabSpec4.setLinearHeadingInterpolation(waitPose2.getHeading(), pickUpPose.getHeading());

        spec4 = new Path(new BezierLine(new Point(pickUpPose), new Point(scoreSpec4)));
        spec4.setLinearHeadingInterpolation(pickUpPose.getHeading(), scoreSpec4.getHeading());
        spec4.setPathEndTimeoutConstraint(100);

        wait3 = new Path(new BezierLine(new Point(scoreSpec4), new Point(waitPose3)));
        wait3.setLinearHeadingInterpolation(scoreSpec4.getHeading(), waitPose3.getHeading());

        grabSpec5 = new Path(new BezierLine(new Point(waitPose3), new Point(pickUpPose)));
        grabSpec5.setLinearHeadingInterpolation(waitPose3.getHeading(), pickUpPose.getHeading());

        spec5 = new Path(new BezierLine(new Point(pickUpPose), new Point(scoreSpec5)));
        spec5.setLinearHeadingInterpolation(pickUpPose.getHeading(), scoreSpec5.getHeading());
        spec5.setPathEndTimeoutConstraint(100);

        wait4 = new Path(new BezierLine(new Point(scoreSpec5), new Point(waitPose4)));
        wait4.setLinearHeadingInterpolation(scoreSpec5.getHeading(), waitPose4.getHeading());

        park = new Path(new BezierLine(new Point(waitPose4), new Point(pickUpPose)));
        park.setLinearHeadingInterpolation(waitPose4.getHeading(), pickUpPose.getHeading());

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        pickUpSpec = new Path(new BezierLine(new Point(chamberPose), new Point(pickUpPose)));
        pickUpSpec.setLinearHeadingInterpolation(chamberPose.getHeading(), pickUpPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                if(!follower.isBusy()) {
                    score();
                    upClawTwist();
                    claw.setPosition(0);
                    setPathState(0);
                }
                break;
            case 0:
                if(!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    setPathState(111);
                }
                break;
            case 111:
                if(!follower.isBusy()) {
                    score();
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    claw.setPosition(0.25);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    claw.setPosition(0.25);
                    armDown();
                    downClawTwist();
                    follower.followPath(samples,true);
                    setPathState(222);
                }
                break;
            case 222:
                if(!follower.isBusy()) {
                    claw.setPosition(0.25);
                    armDown();
                    follower.followPath(samples2,true);
                    setPathState(223);
                }
                break;
            case 223:
                if(!follower.isBusy()) {
                    claw.setPosition(0.25);
                    armDown();
                    follower.followPath(samples3,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    closeClaw();
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    upClawTwist();
                    armUp();
                    follower.followPath(spec2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    score();
                    follower.followPath(grabSpec3, true);
                    openClaw();
                    armDown();
                    downClawTwist();
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    closeClaw();
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(spec3, true);
                    armUpMore();
                    upClawTwist();
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    score();
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(grabSpec4, true);
                    openClaw();
                    armDown();
                    downClawTwist();
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    closeClaw();
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(spec4, true);
                    armUpMore();
                    upClawTwist();
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    score();
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(grabSpec5, true);
                    openClaw();
                    armDown();
                    downClawTwist();
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    closeClaw();
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    follower.followPath(spec5, true);
                    armUpMore();
                    upClawTwist();
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    score();
                    follower.followPath(wait4, true);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    follower.followPath(park, true);
                    openClaw();
                    armDown();
                    downClawTwist();
                    setPathState(-999);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
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
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        elapsedTime = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(-1);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {

    }

    private void armDown(){
        //bigPivot.setPosition(0.78);
        //smallPivot.setPosition(0.27);
        float startTime = elapsedTime.getElapsedTime();
        while(elapsedTime.getElapsedTime() - startTime < 1.5){
            bigPivot.setPosition(.99); //1 for resetting skipping gear, 0.74 for right position
            smallPivot.setPosition(0.16); //0.1 good
            crSmallPivot.setPower(0.018);
        }
               // if (smallPivot.getPosition() >= 0.23){crSmallPivot.setPower(0.018);}
    }

    private void downClawTwist(){
        clawTwist.setPosition(0.3);
    }
    private void armUp(){
        //bigPivot.setPosition(0.38);
        //smallPivot.setPosition(0.9);
        float startTime = elapsedTime.getElapsedTime();
        while(elapsedTime.getElapsedTime() - startTime < .5){
            telemetry.addData("ElapsedTime :", elapsedTime.getElapsedTime());
            bigPivot.setPosition(0.59); //.63
            smallPivot.setPosition(0.55); //.55
            crSmallPivot.setPower(-0.6);
        }
               // (smallPivot.getPosition() <= 0.9){crSmallPivot.setPower(-0.2);}
    }
    private void armUpMore(){
        //bigPivot.setPosition(0.38);
        //smallPivot.setPosition(0.9);
        float startTime = elapsedTime.getElapsedTime();
        while(elapsedTime.getElapsedTime() - startTime < .5){
            telemetry.addData("ElapsedTime :", elapsedTime.getElapsedTime());
            bigPivot.setPosition(0.57); //.63
            smallPivot.setPosition(0.55); //.55
            crSmallPivot.setPower(-0.6);
        }
        // (smallPivot.getPosition() <= 0.9){crSmallPivot.setPower(-0.2);}
    }

    private void armHalfUp(){
        //bigPivot.setPosition(0.38);
        //smallPivot.setPosition(0.9);
        float startTime = elapsedTime.getElapsedTime();
        while(elapsedTime.getElapsedTime() - startTime < 1){
            bigPivot.setPosition(0.51);
            smallPivot.setPosition(0.2);
            crSmallPivot.setPower(-0.1);
        }
        // (smallPivot.getPosition() <= 0.9){crSmallPivot.setPower(-0.2);}
    }

    private void upClawTwist(){
        clawTwist.setPosition(1);
    }
    private void score(){
        //bigPivot.setPosition(0.38);
        //smallPivot.setPosition(0.5);
        float startTime = elapsedTime.getElapsedTime();
        while(elapsedTime.getElapsedTime() - startTime < 300){
            bigPivot.setPosition(0.68);
            smallPivot.setPosition(0.8);//.63
            crSmallPivot.setPower(-1);
        }
        //if  (smallPivot.getPosition() <= 0.3){crSmallPivot.setPower(1);}
    }
    private void scoreAfterPre(){
        //bigPivot.setPosition(0.38);
        //smallPivot.setPosition(0.5);
        float startTime = elapsedTime.getElapsedTime();
        while(elapsedTime.getElapsedTime() - startTime < 300){
            bigPivot.setPosition(0.7);
            smallPivot.setPosition(0.8);//.63
            crSmallPivot.setPower(-1);
        }
        //if  (smallPivot.getPosition() <= 0.3){crSmallPivot.setPower(1);}
    }

    private void fastScore(){
        //bigPivot.setPosition(0.38);
        //smallPivot.setPosition(0.5);
        float startTime = 0;
        if (speedToggle){
            startTime = elapsedTime.getElapsedTime();
        }
        while((elapsedTime.getElapsedTime() - startTime > 1) && (elapsedTime.getElapsedTime() - startTime < 2)){
            bigPivot.setPosition(0.38);
            smallPivot.setPosition(0.5);
            crSmallPivot.setPower(1);
        }
        //if  (smallPivot.getPosition() <= 0.3){crSmallPivot.setPower(1);}
    }

    private void openClaw(){
        float startTime = elapsedTime.getElapsedTime();
        while(elapsedTime.getElapsedTime() - startTime < 1){
            claw.setPosition(0.25);
        }
    }

    private void closeClaw(){
        float startTime = elapsedTime.getElapsedTime();
        while(elapsedTime.getElapsedTime() - startTime < .5){
            claw.setPosition(0);
        }
    }
}

