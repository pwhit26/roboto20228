package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp
public class DriveTest extends LinearOpMode {
    private Follower follower;

    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    boolean halfSpeed;
    public ElapsedTime runtime = new ElapsedTime();
    boolean b2Last;
    double drivePower;
    // Aim-assist button/state
    boolean xLast = false;
    boolean bLast = false;
    boolean yLast = false;
    boolean aimActive = false;
    int aimSettleCount = 0;
    long aimStartMs = 0;
    int currentPipeline = 1; // track current pipeline
    // Vision mode display/state: "X" for AprilTags, "B" for green balls, "" for none
    private String visionMode = "";
    // Limelight
    private Limelight3A limelight;
    // Tunables for geometry-based distance
    private double cameraHeightM = 0.25;      // set your camera height (m)
    private double tagHeightM = 0.80;         // set your tag center height (m)
    private double cameraMountPitchDeg = 0.0; // camera tilt up (+deg)

    @Override
    public void runOpMode() throws InterruptedException {
        // Set constants BEFORE constructing pose/follower utilities


        halfSpeed = false;
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight = hardwareMap.get(DcMotorEx.class, "rightRear");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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


        waitForStart();
        follower.startTeleopDrive();
        runtime.reset();
        while (opModeIsActive()) {
//ALWAYS
            //drive
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // this is strafing
            double rx = gamepad1.right_stick_x; // rotate
            boolean aimAssist = false;

            // Handle X/B edge presses every loop (even if no valid LL result)
            boolean xPressed = gamepad1.x && !xLast;
            boolean bPressed = gamepad1.b && !bLast;
            boolean yPressed = gamepad1.y && !yLast;

            if (xPressed) {
                visionMode = "X";
                if (limelight != null) {
                    limelight.pipelineSwitch(1);
                    currentPipeline = 1;
                }
                aimActive = false;
                aimSettleCount = 0;
            }
            if (bPressed) {
                visionMode = "B";
                if (limelight != null) {
                    limelight.pipelineSwitch(2);
                    currentPipeline = 2;
                }
                aimActive = false;
                aimSettleCount = 0;
            }
            xLast = gamepad1.x;
            bLast = gamepad1.b;
            if (yPressed)
            {
                visionMode = "Y";
                if (limelight!=null)
                {
                    limelight.pipelineSwitch(3);
                    currentPipeline=3;
                }
            }

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            if ("X".equals(visionMode)) {
                telemetry.addData("Mode", "X mode: looking for AprilTags");
            } else if ("B".equals(visionMode)) {
                telemetry.addData("Mode", "B mode: looking for green balls");
            }
            else if ("X".equals(visionMode))
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
                    boolean checked = false;
                    // Continuous ball tracking based on tx even if llValid is false
                    if (ta < 0.2 && !checked)
                    {
                        frontRight.setPower(0.75);
                        backRight.setPower(0.75);
                        frontLeft.setPower(-0.75);
                        backLeft.setPower(-0.75);
                        checked = true;

                    }
                    else {
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

                } else if ("Y".equals(visionMode))
                {
                    boolean checked = false;
                    // Continuous ball tracking based on tx even if llValid is false
                    if (ta < 0.2 && !checked)
                    {
                        frontRight.setPower(0.75);
                        backRight.setPower(0.75);
                        frontLeft.setPower(-0.75);
                        backLeft.setPower(-0.75);
                        checked = true;

                    }

                    else {
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

                } else {
                    // AprilTag mode uses llValid gating
                    if (llValid) {
                        if (xPressed) {
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
