package org.firstinspires.ftc.teamcode.ICE;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//BLUE APRILTAG LIMELIGHT
@TeleOp (group = "AAA", name = "icyTele")
public class icyTele extends LinearOpMode {
    private Follower follower;
    public ElapsedTime runtime = new ElapsedTime();
    boolean popSequenceActive = false;
    boolean popSequenceComplete = true;
    long sequenceStartTime = 0;
    int popSequenceStep = 0;
    public ElapsedTime runtime2 = new ElapsedTime();
    boolean turretSequenceActive = false;
    boolean turretSequenceComplete = true;
    long sequence2StartTime = 0;
    int turretSequenceStep = 0;
    int holdSequenceStep = 0;
    boolean holdSequenceActive = false;
    int afterHoldStep = 0;
    boolean afterHoldActive = false;
    boolean afterHoldComplete = true;
    boolean holdSequenceComplete = true;
    private final Pose startPose = new Pose(0, 0, 0);

    Servo turnTurret, angleTurret0, angleTurret1, popUp;
    CRServo spinny;
    DcMotorEx turret, intake, transferR, transferL, frontRight, frontLeft, backRight, backLeft;
    boolean xLast, bLast, lbumpLast, bPressable, xPressable, lbumpPressable, aLast, aPressable, rbumpLast, rbumpPressable, b1Last, b1Pressable, x1Last, x1Pressable;

    @Override
    public void runOpMode() throws InterruptedException {
        //drive motor init
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Follower after constants are set
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        //other motor init
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transferR = hardwareMap.get(DcMotorEx.class, "transferR");
        transferL = hardwareMap.get(DcMotorEx.class, "transferL");
        turret=hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servo init
        popUp = hardwareMap.get(Servo.class, "popup");
        popUp.setPosition(1);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.99);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.01);
        spinny = hardwareMap.get(CRServo.class, "spinny");
        turnTurret = hardwareMap.get(Servo.class, "turnTurret");
        turnTurret.setPosition(0.48);
        turnTurret.scaleRange(0.21, 0.7); //hard limits, 0.7 left, 0.21 right

        waitForStart();
        follower.startTeleopDrive();
        runtime.reset();
        while (opModeIsActive()) {
            //drive
            double y = -gamepad2.left_stick_y; // Remember, this is reversed!
            double x = -gamepad2.left_stick_x; // this is strafing
            double rx = gamepad2.right_stick_x; // rotate (inverted)
            boolean aimAssist = false;

            // Drive with (possibly) overridden rx
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            // Note: x is negated to reverse the strafing direction
            double leftFrontPower = (y - x + rx) / denominator;
            double leftRearPower = (y + x + rx) / denominator;
            double rightFrontPower = (y + x - rx) / denominator;
            double rightRearPower = (y - x - rx) / denominator;

            frontLeft.setPower(leftFrontPower);
            backLeft.setPower(leftRearPower);
            frontRight.setPower(rightFrontPower);
            backRight.setPower(rightRearPower);

            //BOTH INTAKE AND TRANSFER
            if (gamepad1.x && !xLast) {
                xPressable = !xPressable;
            }
            if (xPressable) {
                intake.setPower(1);
                transferR.setPower(0.7);
                transferL.setPower(0.7);
                spinny.setPower(1);
            }
            else {
                intake.setPower(0);
                transferR.setPower(0);
                transferL.setPower(0);
                spinny.setPower(0);
            }
            xLast = gamepad1.x;

            //JUST intake
            if (gamepad1.b && !bLast) {
                bPressable = !bPressable;
            }
            if (bPressable) {
                intake.setPower(1);
                telemetry.addData("Intake Power", intake.getPower());
            }
            else {
                intake.setPower(0);
            }
            bLast = gamepad1.b;

            //TURRET
            if (gamepad1.a && !turretSequenceActive) {
                turretSequenceActive = true;
                turretSequenceComplete = false;
                sequence2StartTime = System.currentTimeMillis();
            }

            if (turretSequenceActive && !turretSequenceComplete) {
                long elapsedTime2 = System.currentTimeMillis() - sequence2StartTime;
                switch (turretSequenceStep) {
                    case 0:
                        turret.setPower(1);
                        if (elapsedTime2 >= 1000) {
                            turretSequenceStep++;
                            sequence2StartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        intake.setPower(1);
                        transferR.setPower(0.7);
                        transferL.setPower(0.7);
                        spinny.setPower(1);
                        if (elapsedTime2 >= 3500) {
                            turretSequenceStep++;
                            sequence2StartTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        intake.setPower(1);
                        transferR.setPower(0.7);
                        transferL.setPower(0.7);
                        spinny.setPower(1);
                        turret.setPower(0);
                        turretSequenceComplete = true;
                        turretSequenceActive = false;
                        sequence2StartTime = 0;
                        turretSequenceStep = 0;
                        break;
                }
            }

            //POP UP
            if (gamepad1.y && !popSequenceActive) {
                popSequenceActive = true;
                popSequenceComplete = false;
                sequenceStartTime = System.currentTimeMillis();
            }
//hi
            if (popSequenceActive && !popSequenceComplete) {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                switch (popSequenceStep) {
                    case 0:
                        turret.setPower(1);
                        if (elapsedTime >= 1000) {
                            popSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        popUp.setPosition(0.98);
                        if (elapsedTime >= 500) {
                            popSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        popUp.setPosition(1);
                        popSequenceComplete = true;
                        popSequenceActive = false;
                        sequenceStartTime = 0;
                        popSequenceStep = 0;
                        break;
                }
            }

            //ANGLE TURRET
            if (gamepad1.dpad_down) {
                angleTurret0.setPosition(0.99);
                telemetry.addData("Servo Position (1): ", angleTurret0.getPosition());
                angleTurret1.setPosition(0.01);
                telemetry.addData("Servo Position (1): ", angleTurret1.getPosition());
                telemetry.update();
            }

            else if (gamepad1.dpad_up) {
                angleTurret0.setPosition(0.91);
                telemetry.addData("Servo Position (1): ", angleTurret0.getPosition());
                angleTurret1.setPosition(0.09);
                telemetry.addData("Servo Position (2): ", angleTurret1.getPosition());
                telemetry.update();
            }

            if (gamepad1.guide)
            {
                turret.setPower(0);
                spinny.setPower(0);
                intake.setPower(0);
                transferL.setPower(0);
                transferR.setPower(0);
            }

            //try to hold balls
            /*if (gamepad1.b && !bLast) {
                bPressable = !bPressable;
            }
            if (bPressable) {
                spinny.setPower(0.3);
                intake.setPower(1);
                transferR.setPower(0.5);
                transferL.setPower(0.5);
                telemetry.addData("Spinny Power", intake.getPower());
            }
            else {
                spinny.setPower(0);
                intake.setPower(0);
                transferR.setPower(0);
                transferL.setPower(0);
            }
            bLast = gamepad1.b;*/

            //EXPERIMENTAL MACROS
            /*
            //B, dpad Right, Y, dpad Left
            if (gamepad1.b && !holdSequenceActive)
            {
                holdSequenceActive = true;
                holdSequenceComplete = false;
                sequenceStartTime = System.currentTimeMillis();
            }
            if (holdSequenceActive && !holdSequenceComplete) {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                switch (holdSequenceStep) {
                    case 0:
                        intake.setPower(1);
                        transferL.setPower(0.5);
                        transferR.setPower(0.5);
                        spinny.setPower(0.3);
                        if (elapsedTime >= 3000) {
                            holdSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        intake.setPower(0);
                        transferL.setPower(0);
                        transferR.setPower(0);
                        spinny.setPower(0);
                        holdSequenceComplete = true;
                        holdSequenceActive = false;
                        sequenceStartTime = 0;
                        holdSequenceStep = 0;
                        break;
                }
            }

            //shoot after hold
            if (gamepad1.dpad_right && !afterHoldActive)
            {
                afterHoldActive = true;
                afterHoldComplete = false;
                sequenceStartTime = System.currentTimeMillis();

            }
            if (afterHoldActive && !afterHoldComplete)
            {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                switch (afterHoldStep) {
                    case 0:
                        turret.setPower(1);
                        if (elapsedTime >= 1000) {
                            afterHoldStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        telemetry.addData("case 0", "turret only");
                        telemetry.update();
                        break;
                    case 1:
                        turret.setPower(1);
                        spinny.setPower(1);
                        transferL.setPower(0.5);
                        transferR.setPower(0.5);
                        if (elapsedTime>=4000)
                        {
                            afterHoldStep++;
                        }


                        telemetry.addData("case 1", "everything");
                        telemetry.update();
                        break;
                    case 2:
                        turret.setPower(0);
                        spinny.setPower(0);
                        transferL.setPower(0);
                        transferR.setPower(0);
                        afterHoldComplete = true;
                        afterHoldActive = false;
                        sequenceStartTime = 0;
                        telemetry.addData("case 2", "stop");
                        telemetry.update();
                        break;

                    //then popup
                }
            }
             */
            telemetry.update();
        }
    }
}
