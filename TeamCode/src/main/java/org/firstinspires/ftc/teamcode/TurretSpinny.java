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


@TeleOp(name = "Turret Spinny")
public class TurretSpinny extends LinearOpMode {
    // Hardware
    private Servo turnTurret;
    private DcMotorEx turretMotor,frontRight, frontLeft, backRight, backLeft;
    private Limelight3A limelight;
    private Follower follower;

    // PID constants for turret control
    private static final double KP = 0.01;  // Proportional gain (adjust as needed)
    private static final double DEADZONE = 0.5;  // Degrees of error to ignore

    // Servo limits (adjust based on your servo's range)
    //private static final double MIN_SERVO_POS = 0.28;
    //private static final double MAX_SERVO_POS = 0.7;
    private double currentServoPos;
    private double lastPos;// Start in the middle

    // Camera configuration
    private static final double CAMERA_ANGLE_OFFSET_DEG = 0.0; // Any mounting angle offset

    @Override
    public void runOpMode() {
        // Initialize hardware
        turnTurret = hardwareMap.get(Servo.class, "turnTurret");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Follower after constants are set
        follower = Constants.createFollower(hardwareMap);

        // Configure motor
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure Limelight for AprilTag detection
        if (limelight != null) {
            limelight.pipelineSwitch(1);  // AprilTag pipeline (usually 0)
            limelight.start();
        }

        // Set initial servo position
        turnTurret.scaleRange(0.28, 0.7);
        turnTurret.setPosition(0.5);
        currentServoPos= turnTurret.getPosition();


        telemetry.addData("Status", "Initialized. Press Start to begin tracking.");
        telemetry.update();



            // Get Limelight results
            if (limelight != null) {
                limelight.pipelineSwitch(1);  // Changed to pipeline 0 for AprilTag
                limelight.start();
                telemetry.addData("Limelight", "Initialized - Pipeline: %d", 1);
            } else {
                telemetry.addData("Limelight", "Not found in hardware map!");
            }

            // [Rest of initialization...]

            waitForStart();
            follower.startTeleopDrive();

            while (opModeIsActive()) {
                // [Drive code remains the same until Limelight section...]
                //drive
                double y = gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x; // this is strafing
                double rx = gamepad1.right_stick_x; // rotate
                boolean aimAssist = false;

                // Handle X/B edge presses every loop (even if no valid LL result)
                boolean xPressed = gamepad1.x;
                boolean bPressed = gamepad1.b;
                boolean yPressed = gamepad1.y;

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

                // Get Limelight results
                if (limelight != null) {
                    LLResult ll = limelight.getLatestResult();
                    telemetry.addData("Limelight", "Got result: %s", ll != null ? "Valid" : "Null");

                    if (ll != null) {
                        boolean isValid = ll.isValid();
                        double tx = ll.getTx();
                        double ty = ll.getTy();
                        double ta = ll.getTa();

                            if (tx > 2)
                            {
                                currentServoPos = currentServoPos - 0.005;
                                turnTurret.setPosition(currentServoPos);
                                tx = ll.getTx();
                                lastPos=currentServoPos;

                            }
                            if (tx<-2)
                            {
                                currentServoPos = currentServoPos + 0.005;
                                turnTurret.setPosition(currentServoPos);
                                tx = ll.getTx();
                                lastPos=currentServoPos;
                            }
                            if (tx<4 && tx>-4)
                            {
                                turnTurret.setPosition(lastPos);
                                tx = ll.getTx();
                            }


                        //currentServoPos = currentServoPos + (tx);


                        telemetry.addData("LL Valid", isValid);
                        //telemetry.addData("AprilTag ID", tid);
                        telemetry.addData("TX/TY/TA", "%.2f / %.2f / %.2f", tx, ty, ta);

                        if (isValid) {
                            // [Rest of your tracking code...]
                        } else {
                            turnTurret.setPosition(0.5);
                            telemetry.addData("Status", "No AprilTag detected - Check pipeline and tag visibility");
                        }
                    } else {
                        turnTurret.setPosition(0.5);
                        telemetry.addData("Status", "LL Result is null - Check Limelight connection");
                    }
                } else {
                    turnTurret.setPosition(0.5);
                    telemetry.addData("Status", "Limelight not initialized");
                }

                telemetry.update();
                sleep(20);
            }
        }


    // Helper method to calculate distance to target
    private double calculateDistance(double ty) {
        // Camera configuration (adjust these values)
        double cameraHeightM = 0.25;      // Height of camera from ground in meters
        double tagHeightM = 0.80;         // Height of AprilTag from ground
        double cameraMountPitchDeg = 25.0; // Camera angle from horizontal

        // Calculate distance using trigonometry
        double angleToTargetRadians = Math.toRadians(cameraMountPitchDeg + ty);
        return (tagHeightM - cameraHeightM) / Math.tan(angleToTargetRadians);
    }

    }