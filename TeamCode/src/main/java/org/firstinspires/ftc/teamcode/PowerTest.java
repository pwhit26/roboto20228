package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
public class PowerTest extends LinearOpMode {
    private DcMotorEx turret;
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    private Limelight3A limelight;
    private double cameraHeightM = 0.25;      // set your camera height (m)
    private double tagHeightM = 0.80;         // set your tag center height (m)
    private double cameraMountPitchDeg = 25.0;
    private double velocity;
    private boolean rBumpLast, lBumpLast;
    double power;

    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret= hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addData("LL", "initialized");
        } else {
            telemetry.addData("LL", "not found");
        }

        waitForStart();
        while (opModeIsActive())
        {
            double y = gamepad1.right_stick_y; // Remember, this is reversed!
            double x = gamepad1.right_stick_x; // this is strafing
            double rx = gamepad1.left_stick_x; // rotate (inverted)
            boolean aimAssist = false;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y - x + rx) / denominator;
            double leftRearPower = (y + x + rx) / denominator;
            double rightFrontPower = (y + x - rx) / denominator;
            double rightRearPower = (y - x - rx) / denominator;

            frontLeft.setPower(leftFrontPower);
            backLeft.setPower(leftRearPower);
            frontRight.setPower(rightFrontPower);
            backRight.setPower(rightRearPower);

            // Handle X/B edge presses every loop (even if no valid LL result)
            boolean xPressed = gamepad1.x;
            boolean bPressed = gamepad1.b;
            boolean yPressed = gamepad1.y;

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

                double thetaV = Math.toRadians(cameraMountPitchDeg + tyDeg);
                if (Math.abs(Math.cos(thetaV)) > 1e-3) {
                    double forwardZ = (tagHeightM - cameraHeightM) / Math.tan(thetaV);
                    double thetaH = Math.toRadians(txDeg);
                    double lateralX = forwardZ * Math.tan(thetaH);
                    double verticalY = (tagHeightM - cameraHeightM);
                    double euclid = Math.sqrt(lateralX * lateralX + verticalY * verticalY + forwardZ * forwardZ);
                    /*if (forwardZ<2)
                    {
                        forwardZ = forwardZ - 0.295;
                        euclid=euclid-0.295;
                        //power = forwardZ;
                    }
                    else if (forwardZ>=2)
                    {
                        forwardZ = forwardZ - 0.42;
                        euclid = euclid-0.42;
                        //power = forwardZ;
                    }*/
                    power=forwardZ;
                    telemetry.addData("LL forwardZ (m)", forwardZ);
                    telemetry.addData("LL distance (m)", euclid);
                    telemetry.addData("LL valid", llValid);
                    telemetry.addData("LL tx (deg)", txDeg);
                    telemetry.addData("LL ty (deg)", tyDeg);
                    telemetry.update();
                } else {
                    telemetry.addData("LL forwardZ (m)", "undefined angle");
                    //telemetry.update();
                }
                //telemetry.update();


                if (gamepad1.right_bumper && !rBumpLast)
                {
                    velocity = -58.21*(power*power) + 550.8*power + 1264.5;
                    turret.setVelocity((int)Math.round(velocity));
                    telemetry.addData("velocity", (int)Math.round(velocity));
                    telemetry.update();



                }
                rBumpLast = gamepad1.right_bumper;



            }




        }

    }
}
