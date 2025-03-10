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
    private final Pose chamberPose = new Pose(35, 65, Math.toRadians(0));
    private final Pose pickUpPose = new Pose(14, 30, Math.toRadians(0));

    private final Pose goToSample1 = new Pose(56, 30, Math.toRadians(0));
    private final Pose goToSample1_controlPoint = new Pose(27, 38, Math.toRadians(0));
    private final Pose pushSampleIn1 = new Pose(20, 30, Math.toRadians(0));
    private Path scorePreload, pickUpSpec;
    private PathChain samples;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        bigPivot = hardwareMap.get(Servo.class,"bigPivot");
        smallPivot = hardwareMap.get(Servo.class,"smallPivot");
        crSmallPivot = hardwareMap.get(CRServo.class,"crSmallPivot");


        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(chamberPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading());

        samples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(chamberPose), new Point(goToSample1_controlPoint), new Point(goToSample1)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), goToSample1.getHeading())
                .addPath(new BezierLine(new Point(goToSample1), new Point(pushSampleIn1)))
                .setLinearHeadingInterpolation(goToSample1.getHeading(), pushSampleIn1.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        pickUpSpec = new Path(new BezierLine(new Point(chamberPose), new Point(pickUpPose)));
        pickUpSpec.setLinearHeadingInterpolation(chamberPose.getHeading(), pickUpPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                armUp();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(samples,true);
                    setPathState(-1);
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
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    private void armDown(){
        bigPivot.setPosition(0.78);
        smallPivot.setPosition(0.23);
        if (smallPivot.getPosition() >= 0.23){crSmallPivot.setPower(0.018);}
    }
    private void armUp(){
        float startTime = elapsedTime.getElapsedTime();
        while(elapsedTime.getElapsedTime() - startTime < 3){
            bigPivot.setPosition(0.38);
            smallPivot.setPosition(0.9);
            crSmallPivot.setPower(-0.2);
        }
               // (smallPivot.getPosition() <= 0.9){crSmallPivot.setPower(-0.2);}
    }
    private void score(){
        bigPivot.setPosition(0.38);
        smallPivot.setPosition(0.3);
        if  (smallPivot.getPosition() <= 0.3){crSmallPivot.setPower(1);}
    }
}

