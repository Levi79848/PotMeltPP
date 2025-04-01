package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp
public class OldOPMode extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
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
    private Servo clawTwist = null;


    /*
     * Rate limit gamepad button presses to every 500ms.
     */
  /*
   private final static int GAMEPAD_LOCKOUT = 1;

    Telemetry.Item patternName;
    Telemetry.Item display;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }*/

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
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
        clawTwist = hardwareMap.get(Servo.class, "clawTwist");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHang.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftHang.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightHang.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.5);
            double leftFrontpower = (y + x + rx) / denominator;
            double leftBackpower = (y - x + rx) / denominator;
            double rightFrontpower = (y - x - rx) / denominator;
            double rightBackpower = (y + x - rx) / denominator;

            leftFront.setPower(leftFrontpower);
            leftBack.setPower(leftBackpower);
            rightFront.setPower(rightFrontpower);
            rightBack.setPower(rightBackpower);

            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("smallPivotPosition: ", smallPivot.getPosition());
            telemetry.addData("bigPivotPosition: ", bigPivot.getPosition());
            telemetry.update();



//gamepad 1 ---------------------------------------

        /*
        if (gamepad1.left_bumper) {
            leftSlide.setPower(-0.5);
            rightSlide.setPower(-0.5);
        } else if (gamepad1.right_bumper) {
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        } else {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
        }
        */


            if (gamepad1.a) {
                claw.setPosition(0.25);
            } else if (gamepad1.x) {
                claw.setPosition(0);
            }

            if (gamepad1.b) {
                bigPivot.setPosition(1); //1 for resetting skipping gear, 0.74 for right position
                smallPivot.setPosition(0.2); //0.1 good
                clawTwist.setPosition(0.3);
                crSmallPivot.setPower(0.018);
            } else if (gamepad1.y) {
                bigPivot.setPosition(0.48); //0 for resetting skipping gear, 0.2 for right position
                smallPivot.setPosition(0.7); //0.8 good
                clawTwist.setPosition(1);
                crSmallPivot.setPower(-0.1);
            } else if (gamepad1.dpad_right) {
                crSmallPivot.setPower(-1);
                smallPivot.setPosition(smallPivot.getPosition()+0.01);
            } else if (gamepad1.dpad_left) {
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
        }
    }
}