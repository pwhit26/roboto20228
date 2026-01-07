package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TestRPM extends LinearOpMode {
    private Follower follower;
    private DcMotorEx turret;
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    private Limelight3A limelight;
    private double cameraHeightM = 0.25;      // set your camera height (m)
    private double tagHeightM = 0.80;         // set your tag center height (m)
    private double cameraMountPitchDeg = 20.0;
    private double ticks = 28;
    private double rpm = 0;
    private double velocity;
    private boolean rBumpLast, lBumpLast;
    private double current;
    Servo angleTurret0, angleTurret1;

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
        // Follower after constants are set
        follower = Constants.createFollower(hardwareMap);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.06);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.94);

        turret= hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
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
        while (opModeIsActive()){


            double y = -gamepad2.left_stick_y; // Remember, this is reversed!
            double x = -gamepad2.left_stick_x; // this is strafing
            double rx = gamepad2.right_stick_x; // rotate (inverted)
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

        if (gamepad2.right_bumper && !rBumpLast)
        {
            rpm = rpm + 10;
            velocity = (rpm/60.0)*ticks;
            turret.setVelocity((int)Math.round(velocity));
            current = turret.getCurrentPosition() / 28.0;


        }
            rBumpLast = gamepad1.right_bumper;
        if (gamepad2.left_bumper && !lBumpLast)
        {
            rpm = rpm - 10;
            velocity = (rpm/60.0)*ticks;
            turret.setVelocity((int)Math.round(velocity));


        }
            lBumpLast = gamepad1.left_bumper;

        //turret.setVelocity((int)Math.round(velocity));
            telemetry.addData("target rpm", rpm);
            telemetry.addData("actual rpm", (int)current);
            telemetry.addData("target velocity", (int)Math.round(velocity));
            telemetry.addData("turret velocity", (int)turret.getVelocity());

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
                double dist=calculateDistance(txDeg, tyDeg);
                setTurretAngle(dist);
            }


        double thetaV = Math.toRadians(cameraMountPitchDeg + tyDeg);
        if (Math.abs(Math.cos(thetaV)) > 1e-3) {
            double forwardZ = (tagHeightM - cameraHeightM) / Math.tan(thetaV);
            double thetaH = Math.toRadians(txDeg);
            double lateralX = forwardZ * Math.tan(thetaH);
            double verticalY = (tagHeightM - cameraHeightM);
            double euclid = Math.sqrt(lateralX * lateralX + verticalY * verticalY + forwardZ * forwardZ);
            telemetry.addData("LL forwardZ (m)", forwardZ);
            telemetry.addData("LL distance (m)", euclid);
        }

    }


        telemetry.update();



        }

}
    private void setTurretAngle(double dist)
    {
        if (dist>2)
        {
            angleTurret0.setPosition(0.03);
            angleTurret1.setPosition(0.97);
        }
        else if (dist>1.5)
        {
            angleTurret0.setPosition(0.035);
            angleTurret1.setPosition(0.965);
        }
        else if (dist>1)
        {
            angleTurret0.setPosition(0.06);
            angleTurret1.setPosition(0.94);
        }
        else if (dist>0.75)
        {
            angleTurret0.setPosition(0.09);
            angleTurret1.setPosition(0.91);
        }
        else if (dist<=0.5){
            angleTurret0.setPosition(0.1);
            angleTurret1.setPosition(0.9);
        }
        else {
            angleTurret0.setPosition(0.06);
            angleTurret1.setPosition(0.94);
        }
    }
    private double calculateDistance(double ty, double tx) {
        // Camera configuration (adjust these values)
        double cameraHeightM = 0.3;      // Height of camera from ground in meters
        double tagHeightM = 0.75;         // Heig   ht of AprilTag from ground
        double cameraMountPitchDeg = 20.0; // Camera angle from horizontal

        double thetaV = Math.toRadians(cameraMountPitchDeg + ty);
        if (Math.abs(Math.cos(thetaV)) > 1e-3) {
            double forwardZ = (tagHeightM - cameraHeightM) / Math.tan(thetaV);
            double thetaH = Math.toRadians(tx);
            double lateralX = forwardZ * Math.tan(thetaH);
            double verticalY = (tagHeightM - cameraHeightM);
            double euclid = Math.sqrt(lateralX * lateralX + verticalY * verticalY + forwardZ * forwardZ);

            // Calculate distance using trigonometry
            return forwardZ;

        }
        else {
            return 0;
        }

    }}
