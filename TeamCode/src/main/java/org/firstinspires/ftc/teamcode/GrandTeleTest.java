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
//hi
@TeleOp(name="GrandTele")
public class GrandTeleTest extends LinearOpMode{
    private Follower follower;

    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotorEx frontRight, frontLeft, backRight, backLeft, intake, turret;
    Servo turnTurret, angleTurret0, angleTurret1, popUp;
    CRServo upperTransferL, upperTransferR, lowerTransferL, lowerTransferR;
    public static double turnTurretLowerBound = 0;
    public static double turnTurretUpperBound = 0.8;
    public ElapsedTime runtime = new ElapsedTime();
    boolean b2Last;
    double drivePower;
    public static double pos1=0.174;//smaller numbers make it go up
    public static double pos2= 0.95;
    boolean popSequenceActive = false;
    boolean popSequenceComplete = true;
    long sequenceStartTime = 0;
    int popSequenceStep = 0;

    // Aim-assist button/state
    boolean xLast, bLast, yLast, bPressable, yPressable, aLast, aPressable, rbumpLast, rbumpPressable, lbumpLast, lbumpPressable;

    boolean aimActive = false;
    int aimSettleCount = 0;
    long aimStartMs = 0;
    int currentPipeline = 1; // track current pipeline
    // Vision mode display/state: "X" for AprilTags, "B" for green balls, "" for none
    private String visionMode = "";
    // Limelight
    private Limelight3A limelight;
    // Tunables for geometry-based distance
    private double cameraHeightM = 0.27;      // set your camera height (m)
    private double tagHeightM = 0.80;         // set your tag center height (m)
    private double cameraMountPitchDeg = 0.0; // camera tilt up (+deg)


    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Follower after constants are set
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addData("LL", "initialized");
        } else {
            telemetry.addData("LL", "not found");
        }
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //turnTurret = hardwareMap.get(Servo.class, "turnTurret");
        //turnTurret.scaleRange(turnTurretLowerBound, turnTurretUpperBound);
        //turnTurret.setPosition(0.75);
        //telemetry.addData("Turret Rotation Position", turnTurret.getPosition());
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.25);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.75);
        upperTransferL=hardwareMap.get(CRServo.class, "upperTransferL");
        upperTransferR=hardwareMap.get(CRServo.class, "upperTransferR");
        lowerTransferL=hardwareMap.get(CRServo.class, "lowerTransferL");
        lowerTransferR=hardwareMap.get(CRServo.class, "lowerTransferR");
        popUp=hardwareMap.get(Servo.class, "popUp");
        popUp.setPosition(0.14);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        yPressable = false;
        yLast = false;
        waitForStart();
        follower.startTeleopDrive();
        runtime.reset();
        while (opModeIsActive()) {
//ALWAYSSS
            //drive
            double y = gamepad2.left_stick_y; // Remember, this is reversed!
            double x = gamepad2.left_stick_x; // this is strafing
            double rx = gamepad2.right_stick_x; // rotate
            boolean aimAssist = false;

            // Handle X/B edge presses every loop (even if no valid LL result)
            boolean xPressed = gamepad1.x;
            boolean bPressed = gamepad1.b;
            boolean yPressed = gamepad1.y;
            boolean xEdge = xPressed && !xLast;
            boolean bEdge = bPressed && !bLast;

            if (yPressed && !yLast) {
                yPressable = !yPressable;
            }
            if (yPressable) {
                intake.setPower(0.5);
                telemetry.addData("Intake Power", intake.getPower());
            }
            else {
                intake.setPower(0);
            }
            yLast = gamepad1.y;

            //TURRET CONTROLS

            if (gamepad1.dpad_right) {
                angleTurret0.setPosition(0.1);
                telemetry.addData("Servo Position (1): ", angleTurret0.getPosition());
                angleTurret1.setPosition(0.9);
                telemetry.addData("Servo Position (1): ", angleTurret1.getPosition());
                telemetry.update();
            }

            else if (gamepad1.dpad_left) {
                angleTurret0.setPosition(0.28);
                telemetry.addData("Servo Position (1): ", angleTurret0.getPosition());
                angleTurret1.setPosition(0.72);
                telemetry.addData("Servo Position (2): ", angleTurret1.getPosition());
                telemetry.update();
            }

            if (gamepad1.right_bumper && !rbumpLast) {
                rbumpPressable = !rbumpPressable;
            }
            if (rbumpPressable) {
                upperTransferL.setPower(1);
                upperTransferR.setPower(-1);
                lowerTransferL.setPower(1);
                lowerTransferR.setPower(-1);
            }
            else {
                upperTransferL.setPower(0);
                upperTransferR.setPower(0);
                lowerTransferL.setPower(0);
                lowerTransferR.setPower(0);
            }
            rbumpLast = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !popSequenceActive) {
                popSequenceActive = true;
                popSequenceComplete = false;
                sequenceStartTime = System.currentTimeMillis();
            }

            if (popSequenceActive && !popSequenceComplete) {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                switch (popSequenceStep) {
                    case 0:
                        popUp.setPosition(0.09);
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

            if (gamepad1.a && !aLast) {
                aPressable = !aPressable;
            }
            if (aPressable) {
                turret.setPower(1);
                telemetry.addData("Turret Power", turret.getPower());
            }
            else {
                turret.setPower(0);
            }
            aLast = gamepad1.a;

            telemetry.update();

            // X: first press selects AprilTag mode; subsequent presses toggle aim correction
            if (xEdge) {
                if (!"X".equals(visionMode)) {
                    visionMode = "X";
                    if (limelight != null) {
                        limelight.pipelineSwitch(1);
                        currentPipeline = 1;
                    }
                    aimActive = false;
                    aimSettleCount = 0;
                } else {
                    // Toggle aim while in X mode
                    aimActive = !aimActive;
                    aimSettleCount = 0;
                    aimStartMs = System.currentTimeMillis();
                }
            }

            // B: toggle between green (B) and purple (Y) ball detection on each press
            if (bEdge) {
                if (!"B".equals(visionMode) && !"Y".equals(visionMode)) {
                    visionMode = "B";
                    if (limelight != null) {
                        limelight.pipelineSwitch(2);
                        currentPipeline = 2;
                    }
                } else if ("B".equals(visionMode)) {
                    visionMode = "Y";
                    if (limelight != null) {
                        limelight.pipelineSwitch(3);
                        currentPipeline = 3;
                    }
                } else { // currently Y
                    visionMode = "B";
                    if (limelight != null) {
                        limelight.pipelineSwitch(2);
                        currentPipeline = 2;
                    }
                }
                // Entering ball vision cancels any tag-aim
                aimActive = false;
                aimSettleCount = 0;
            }

            // Update edge trackers
            xLast = xPressed;
            bLast = bPressed;

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            if ("X".equals(visionMode)) {
                telemetry.addData("Mode", "X mode: looking for AprilTags");
            } else if ("B".equals(visionMode)) {
                telemetry.addData("Mode", "B mode: looking for green balls");
            }
            else if ("Y".equals(visionMode))
            {
                telemetry.addData("Mode", "Y mode: looking for purple balls");
            }

            telemetry.update();

            // Limelight telemetry: angles and distance
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

                telemetry.addData("Pipeline", currentPipeline);

                if ("X".equals(visionMode))
                {
                    telemetry.addData("LL valid", llValid);
                    telemetry.addData("LL tx (deg)", txDeg);
                    telemetry.addData("LL ty (deg)", tyDeg);

                }
                if ("B".equals(visionMode) || "Y".equals(visionMode)) {
                    telemetry.addData("Ball tx (deg)", txDeg);
                    telemetry.addData("Ball ty (deg)", tyDeg);
                }

                // Distance estimate (uses tag height constants; mainly useful in X mode)
                double thetaV = Math.toRadians(cameraMountPitchDeg + tyDeg);
                if (Math.abs(Math.cos(thetaV)) > 1e-3 && "X".equals(visionMode)) {
                    double forwardZ = (tagHeightM - cameraHeightM) / Math.tan(thetaV);
                    double thetaH = Math.toRadians(txDeg);
                    double lateralX = forwardZ * Math.tan(thetaH);
                    double verticalY = (tagHeightM - cameraHeightM);
                    double euclid = Math.sqrt(lateralX*lateralX + verticalY*verticalY + forwardZ*forwardZ);
                    telemetry.addData("LL forwardZ (m)", forwardZ);
                    telemetry.addData("LL distance (m)", euclid);
                } else {
                    telemetry.addData("LL forwardZ (m)", "undefined angle");
                }

                // Mode-specific aim-control
                if ("B".equals(visionMode)) {
                    if (ta >= 0.2){
                        double kPBall_g = 0.02;   // tune
                        double minPowerBall = 0.10;
                        double epsDriveDegBall = 0.5;
                        double rxAuto;
                        if (Math.abs(txDeg) > epsDriveDegBall) {
                            rxAuto = kPBall_g * txDeg;
                            if (Math.abs(rxAuto) < minPowerBall) {
                                rxAuto = Math.copySign(minPowerBall, rxAuto);
                            }
                        } else {
                            rxAuto = 0.0;
                        }
                        if (rxAuto > 0.7) rxAuto = 0.7;
                        if (rxAuto < -0.7) rxAuto = -0.7;
                        rx = rxAuto;
                        aimActive = true;
                        telemetry.addData("AimAssist", String.format("BALL ACTIVE rx=%.3f tx=%.2f ty=%.2f", rxAuto, txDeg, tyDeg));
                    }
                    else{

                    }
                }
                else if ("X".equals(visionMode)) {
                    // AprilTag mode uses llValid gating
                    if (llValid) {
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
                else if ("Y".equals(visionMode))
                {
                    if (ta >= 0.2)
                    {
                        double kPBall_p = 0.02;   // tune
                        double minPowerBall = 0.10;
                        double epsDriveDegBall = 0.5;
                        double rxAuto;
                        if (Math.abs(txDeg) > epsDriveDegBall) {
                            rxAuto = kPBall_p * txDeg;
                            if (Math.abs(rxAuto) < minPowerBall) {
                                rxAuto = Math.copySign(minPowerBall, rxAuto);
                            }
                        } else {
                            rxAuto = 0.0;
                        }
                        if (rxAuto > 0.7) rxAuto = 0.7;
                        if (rxAuto < -0.7) rxAuto = -0.7;
                        rx = rxAuto;
                        aimActive = true;
                        telemetry.addData("AimAssist", String.format("BALL ACTIVE rx=%.3f tx=%.2f ty=%.2f", rxAuto, txDeg, tyDeg));
                    }
                    else{

                    }
                }
            }
            // Drive with (possibly) overridden rx
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftRearPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightRearPower = (y + x - rx) / denominator;

            frontLeft.setPower(leftFrontPower);
            backLeft.setPower(leftRearPower);
            frontRight.setPower(rightFrontPower);
            backRight.setPower(rightRearPower);
        }

    }
}