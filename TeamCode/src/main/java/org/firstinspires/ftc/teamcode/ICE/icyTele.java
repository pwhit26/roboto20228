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
        popUp.setPosition(0.14);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.135);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.865);
        spinny = hardwareMap.get(CRServo.class, "spinny");

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


            // Handle X/B edge presses every loop (even if no valid LL result)
            boolean xPressed = gamepad2.x;
            boolean bPressed = gamepad2.b;
            boolean yPressed = gamepad2.y;


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


            if (gamepad1.dpad_left)
            {
                turret.setPower(0);
                spinny.setPower(0);
                intake.setPower(0);
                transferL.setPower(0);
                transferR.setPower(0);
            }


            //JUST intake
            if (gamepad1.right_bumper && !rbumpLast) {
                rbumpPressable = !rbumpPressable;
            }
            if (rbumpPressable) {
                intake.setPower(0.5);
                telemetry.addData("Intake Power", intake.getPower());
            }
            else {
                intake.setPower(0);
            }
            rbumpLast = gamepad1.right_bumper;


            //JUST TRANSFER
            if (gamepad1.left_bumper && !lbumpLast) {
                lbumpPressable = !lbumpPressable;
            }
            if (lbumpPressable) {
                transferL.setPower(1);
                transferR.setPower(1);

            }
            else {
                transferL.setPower(0);
                transferR.setPower(0);
            }
            lbumpLast = gamepad1.left_bumper;


            //BOTH INTAKE AND TRANSFER
            if (gamepad1.x && !xLast) {
                xPressable = !xPressable;
            }
            if (xPressable) {
                intake.setPower(1);
                transferR.setPower(0.5);
                transferL.setPower(0.5);
                turret.setPower(1);
                spinny.setPower(1);
            }
            else {
                intake.setPower(0);
                transferR.setPower(0);
                transferL.setPower(0);
                turret.setPower(0);
                spinny.setPower(0);
            }
            xLast = gamepad1.x;


            //POP UP
            if (gamepad1.y && !popSequenceActive) {
                popSequenceActive = true;
                popSequenceComplete = false;
                sequenceStartTime = System.currentTimeMillis();
            }

            if (popSequenceActive && !popSequenceComplete) {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                switch (popSequenceStep) {
                    case 0:
                        popUp.setPosition(0.105);
                        if (elapsedTime >= 500) {
                            popSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        popUp.setPosition(0.14);
                        popSequenceComplete = true;
                        popSequenceActive = false;
                        sequenceStartTime = 0;
                        popSequenceStep = 0;
                        break;
                }
            }


            //TURRET
            if (gamepad1.a && !aLast) {
                aPressable = !aPressable;
            }
            if (aPressable) {
                //spinny.setPower(1);
                turret.setPower(1);
                //telemetry.addData("Spinny Power", spinny.getPower());
                telemetry.addData("Turret Power", turret.getPower());
            }
            else {
                //spinny.setPower(0);
                turret.setPower(0);
            }
            aLast = gamepad1.a;
            telemetry.update();


            //ANGLE TURRET
            if (gamepad1.dpad_down) {
                angleTurret0.setPosition(0.135);
                telemetry.addData("Servo Position (1): ", angleTurret0.getPosition());
                angleTurret1.setPosition(0.865);
                telemetry.addData("Servo Position (1): ", angleTurret1.getPosition());
                telemetry.update();
            }

            else if (gamepad1.dpad_up) {
                angleTurret0.setPosition(0.09);
                telemetry.addData("Servo Position (1): ", angleTurret0.getPosition());
                angleTurret1.setPosition(0.91);
                telemetry.addData("Servo Position (2): ", angleTurret1.getPosition());
                telemetry.update();
            }

            telemetry.update();
        }
    }
}
