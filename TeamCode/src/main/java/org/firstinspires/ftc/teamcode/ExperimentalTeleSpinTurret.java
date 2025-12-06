package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
@TeleOp (group = "AAA", name = "experiment")
public class ExperimentalTeleSpinTurret extends LinearOpMode {
    private boolean aprilTagTracking = false;
    boolean aimActive = false;
    int aimSettleCount = 0;
    long aimStartMs = 0;
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
    private double currentServoPos;
    private double lastPos;
    private Limelight3A limelight;
    //hi
    Servo turnTurret, angleTurret0, angleTurret1, popUp;
    CRServo spinny;
    DcMotorEx turret, intake, transferR, transferL, frontRight, frontLeft, backRight, backLeft;
    boolean xLast, bLast, lbumpLast, bPressable, xPressable, lbumpPressable, aLast, aPressable, rbumpLast, rbumpPressable, b1Last, b1Pressable, x1Last, x1Pressable;

    @Override
    public void runOpMode() throws InterruptedException {
        //drive motor init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servo init
        popUp = hardwareMap.get(Servo.class, "popup");
        popUp.scaleRange(0.19, 0.24); //0.19 is up, 0.24 is down
        popUp.setPosition(0.8);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.5);
        angleTurret0.scaleRange(0.44, 0.58);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.5);
        angleTurret1.scaleRange(0.42, 0.56);
        spinny = hardwareMap.get(CRServo.class, "spinny");
        turnTurret = hardwareMap.get(Servo.class, "turnTurret");
        turnTurret.scaleRange(0.28, 0.7); //hard limits, 0.7 left, 0.21 right
        turnTurret.setPosition(0.5);
        currentServoPos = turnTurret.getPosition();
        if (limelight != null) {
            limelight.pipelineSwitch(1);  // AprilTag pipeline (usually 0)
            limelight.start();
        }

        waitForStart();
        follower.startTeleopDrive();
        runtime.reset();
        while (opModeIsActive()) {
            //drive
            double y = -gamepad2.left_stick_y; // Remember, this is reversed!
            double x = -gamepad2.left_stick_x; // this is strafing
            double rx = gamepad2.right_stick_x; // rotate (inverted)
            boolean aimAssist = false;




            if (gamepad2.b) {
                if (limelight != null) {
                    limelight.pipelineSwitch(1);
                     LLResult llResult = limelight.getLatestResult();
                }
                aimActive = false;
                aimSettleCount = 0;
            }
            if (limelight != null) {
                LLResult ll = limelight.getLatestResult();
                double txDeg = 0.0;
                double tyDeg = 0.0;
                double ta = 0.0;
                boolean llValid = false;
                if (ll != null) {
                    txDeg = ll.getTx();
                    tyDeg = ll.getTy();
                    ta = ll.getTa();
                    llValid = ll.isValid();
                }
                if (llValid) {
                    if (gamepad2.b) {
                        telemetry.addData("LL","x - aprilTag");
                        double epsStartDeg = 1.0;
                        aimActive = Math.abs(txDeg) > epsStartDeg;
                        if (aimActive) {
                            aimSettleCount = 0;
                            aimStartMs = System.currentTimeMillis();
                        }
                    }

                    if (aimActive) {
                        double kP = 0.015;
                        double minPower = 0.12;
                        double rxAuto;
                        double epsDriveDeg = 1.0;
                        if (Math.abs(txDeg) > epsDriveDeg) {
                            rxAuto = kP * txDeg;
                            if (Math.abs(rxAuto) < minPower) {
                                rxAuto = Math.copySign(minPower, rxAuto);
                            }
                        } else {
                            rxAuto = 0.0;
                        }
                        if (rxAuto > 0.6) rxAuto = 0.6;
                        if (rxAuto < -0.6) rxAuto = -0.6;
                        rx = rxAuto;

                        double epsDeg = 1.0;
                        if (Math.abs(txDeg) <= epsDeg) {
                            aimSettleCount++;
                        } else {
                            aimSettleCount = 0;
                        }
                        if (aimSettleCount >= 5) {
                            aimActive = false;
                        }
                        telemetry.addData("AimAssist", String.format("TAG ACTIVE rx=%.3f tx=%.2f", rxAuto, txDeg));
                    } else {
                        telemetry.addData("AimAssist", "READY (press X)");
                    }
                } else {
                    telemetry.addData("LL", "no valid tag target");
                }

            }

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




        //GRAB 3 BALLS AND SHOOT SEQUENCE: X for intaking 2 balls, X to stop, B to get third Ball, B to stop, A for turret to shoot 2 balls, Y for popup to shoot 3rd ball

        //BOTH INTAKE AND TRANSFER
        if (gamepad1.x && !xLast) {
            xPressable = !xPressable;
        }
        if (xPressable) {
            intake.setPower(1);
            transferR.setPower(0.7);
            transferL.setPower(0.7);
            spinny.setPower(1);
        } else {
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
        } else {
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
                    transferR.setPower(0.9);
                    transferL.setPower(0.9);
                    spinny.setPower(1);
                    if (elapsedTime2 >= 3500) {
                        turretSequenceStep++;
                        sequence2StartTime = System.currentTimeMillis();
                    }
                    break;
                case 2:
                    intake.setPower(1);
                    transferR.setPower(0.9);
                    transferL.setPower(0.9);
                    spinny.setPower(1);
                    turret.setPower(0);
                    turretSequenceComplete = true;
                    turretSequenceActive = false;
                    sequence2StartTime = 0;
                    turretSequenceStep = 0;
                    break;
            }
        }

        //POP UP --> shoots one ball
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
                    popUp.setPosition(0.1);
                    if (elapsedTime >= 500) {
                        popSequenceStep++;
                        sequenceStartTime = System.currentTimeMillis();
                    }
                    break;
                case 2:
                    popUp.setPosition(0.9);
                    turret.setPower(0);

                    popSequenceComplete = true;
                    popSequenceActive = false;
                    sequenceStartTime = 0;
                    popSequenceStep = 0;
                    break;
            }
        }

        //ANGLE TURRET
        if (gamepad1.dpad_down) {
            angleTurret0.setPosition(0.8);
            telemetry.addData("Servo Position (1): ", angleTurret0.getPosition());
            angleTurret1.setPosition(0.2);
            telemetry.addData("Servo Position (1): ", angleTurret1.getPosition());
            telemetry.update();
        } else if (gamepad1.dpad_up) {
            angleTurret0.setPosition(0.2);
            telemetry.addData("Servo Position (1): ", angleTurret0.getPosition());
            angleTurret1.setPosition(0.8);
            telemetry.addData("Servo Position (2): ", angleTurret1.getPosition());
            telemetry.update();
        }
        if (gamepad1.right_bumper) {
            intake.setPower(-1);
            transferR.setPower(-1);
            transferL.setPower(-1);
            spinny.setPower(-1);
        }

        if (gamepad1.guide) {
            turret.setPower(0);
            spinny.setPower(0);
            intake.setPower(0);
            transferL.setPower(0);
            transferR.setPower(0);
        }

        telemetry.update();
    }
}}

