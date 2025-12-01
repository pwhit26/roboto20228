package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp
public class TurretSpinny extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    Servo turnTurret;
    DcMotorEx turret;//, frontLeft, frontRight, backLeft, backRight;
    //public Follower follower;
    boolean b2Last;
    double servoPos;
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

        //follower=Constants.createFollower(hardwareMap);
        //frontLeft=hardwareMap.get(DcMotorEx.class, "leftFront");
        //frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        //backLeft=hardwareMap.get(DcMotorEx.class, "leftBack");
        //backRight= hardwareMap.get(DcMotorEx.class, "rightBack");
        // Set constants BEFORE constructing pose/follower utilities
        turnTurret=hardwareMap.get(Servo.class, "turnTurret");
        turnTurret.scaleRange(0, 1);
        turnTurret.setPosition(0);
        turret=hardwareMap.get(DcMotorEx.class, "turret");
        servoPos = turnTurret.getPosition() * 360;


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
        //follower.startTeleopDrive();
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

            if ("X".equals(visionMode)) {
                telemetry.addData("Mode", "X mode: looking for AprilTags");
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
                            //double kP = 0.015;
                            //double minPower = 0.12;
                            //double rxAuto;
                            double epsDriveDeg = 1.0;
                            if (Math.abs(txDeg) > epsDriveDeg) {
                                //rxAuto = kP * txDeg;
                                servoPos = servoPos + txDeg;
                                turnTurret.setPosition(servoPos/360);
                                //if (Math.abs(rxAuto) < minPower) {
                                    //rxAuto = Math.copySign(minPower, rxAuto);
                                //}
                            } //else {
                                //rxAuto = 0.0;
                            //}
                            //if (rxAuto > 0.6) rxAuto = 0.6;
                            //if (rxAuto < -0.6) rxAuto = -0.6;
                            //rx = rxAuto;

                            double epsDeg = 1.0;
                            if (Math.abs(txDeg) <= epsDeg) {
                                aimSettleCount++;
                            } else {
                                aimSettleCount = 0;
                            }
                            if (aimSettleCount >= 5) {
                                aimActive = false;
                            }
                            telemetry.addData("AimAssist", String.format("TAG ACTIVE rx=%.3f tx=%.2f", txDeg));
                        } else {
                            telemetry.addData("AimAssist", "READY (press X)");
                        }
                    } else {
                        telemetry.addData("LL", "no valid tag target");
                    }

            }


            //Drive with (possibly) overridden rx
            /*double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftRearPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightRearPower = (y + x - rx) / denominator;*/


        }


    }
}
