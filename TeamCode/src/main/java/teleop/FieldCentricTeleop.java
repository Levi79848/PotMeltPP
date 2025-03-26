package teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "FieldCentric", group = "TeleOp")
public class FieldCentricTeleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);


    private CRServo intake = null;
    private CRServo sub_extender = null;
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private Servo flip = null;
    private Servo claw = null;
    private DcMotor leftHang = null;
    private DcMotor rightHang = null;
    private Servo bigPivot = null;
    private Servo smallPivot = null;
    private CRServo crSmallPivot = null;
    private Timer runtime = new Timer();



    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        intake = hardwareMap.get(CRServo.class, "intake");
        sub_extender = hardwareMap.get(CRServo.class, "sub_extender");
        flip = hardwareMap.get(Servo.class, "flip");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        claw = hardwareMap.get(Servo.class, "claw");
        leftHang = hardwareMap.get(DcMotor.class, "leftHang");
        rightHang = hardwareMap.get(DcMotor.class, "rightHang");
        bigPivot = hardwareMap.get(Servo.class, "bigPivot");
        smallPivot = hardwareMap.get(Servo.class, "smallPivot");
        crSmallPivot = hardwareMap.get(CRServo.class, "crSmallPivot");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
       rightHang.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        rightHang.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
       if (gamepad1.a) {
                claw.setPosition(0.25);
            } else if (gamepad1.x) {
                claw.setPosition(0);
            }

            if (gamepad1.b) {
                bigPivot.setPosition(0.78); //1 for resetting skipping gear, 0.74 for right position
                smallPivot.setPosition(0.29); //0.1 good
                if (smallPivot.getPosition() >= 0.23){crSmallPivot.setPower(0.018);}
            } else if (gamepad1.y) {
                bigPivot.setPosition(0.38); //0 for resetting skipping gear, 0.2 for right position
                smallPivot.setPosition(0.95); //0.8 good
                if (smallPivot.getPosition() <= 0.9){crSmallPivot.setPower(-0.2);}
            } else if (gamepad1.right_bumper) {
                crSmallPivot.setPower(-1);
                smallPivot.setPosition(smallPivot.getPosition()+0.01);
            } else if (gamepad1.left_bumper) {
                crSmallPivot.setPower(1);
                smallPivot.setPosition(smallPivot.getPosition()-0.01);
            } else {
                crSmallPivot.setPower(0);
            }

        /*
        if (gamepad1.y) {
            bigPivot.setPosition(1);
            smallPivot.setPosition(1);
        } else if (gamepad1.b) {
            bigPivot.setPosition(1);
            smallPivot.setPosition(1);
        }*/

//gamepad 2 --------------------------------------

            if (gamepad2.left_bumper) {
                leftHang.setPower(-1);
                rightHang.setPower(-1);
            } else if(gamepad2.right_bumper) {
                leftHang.setPower(1);
                rightHang.setPower(1);
            } else {
                leftHang.setPower(0);
                rightHang.setPower(0);
            }

            //Intake Stuff ----------------------------------

            if (gamepad2.dpad_left) {
                sub_extender.setPower(1);
            } else if (gamepad2.dpad_right) {
                sub_extender.setPower(-1);
            } else {
                sub_extender.setPower(0);
            }

            if (gamepad2.a) {
                intake.setPower(-0.5);
            }  else if (gamepad2.b) {
                intake.setPower(0.5);
            }  else {
                intake.setPower(0);
            }

            if (gamepad2.dpad_up) {
                flip.setPosition(0);
            } else if (gamepad2.dpad_down) {
                flip.setPosition(0.54);
            }
        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}