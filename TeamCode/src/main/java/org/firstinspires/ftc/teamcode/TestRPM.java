package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TestRPM extends LinearOpMode {
    private Follower follower;
    private DcMotorEx turret, intake, turnTurret;
    private DcMotorEx frontRight, frontLeft, backRight, backLeft, spindexer;
    private Limelight3A limelight;
    private double cameraHeightM = 0.25;      // set your camera height (m)
    private double tagHeightM = 0.80;         // set your tag center height (m)
    private double cameraMountPitchDeg = 20.0;
    private double ticks = 28;
    private double rpm = 0;
    private double velocity;
    private boolean rBumpLast, lBumpLast;
    private double current;
    Servo angleTurret0, angleTurret1, popUp;
    long sequenceStartTime = 0;
    int popSequenceStep = 0;
    int shootStep=0;
    boolean shootSequenceActive = false;
    boolean shootSequenceComplete = true;
    private RevColorSensorV3 colorBack, color0, color1, colorFront;
    private boolean wasColorDetected = false;
    private AnalogInput encoder;
    static final int numIntakeSlots = 3;
    int[] intakeSlotPositions = {110, 230, 355};
    int[] shootSlotPositions = {65, 190, 305};
    static final double kP = 0.0155; // Start small
    static final double kD = 0.0008; // Helps prevent overshoot
    static final double kI = 0.0;    // Usually not needed for a spindexer
    double lastError = 0;
    ElapsedTime pidTimer = new ElapsedTime();
    double PositionToleranceDeg = 8;
    static final double max_spin_power = 0.5;
    double integral = 0;
    int currentSlot = 0;
    boolean spindexerMoving = false;
    int currentShootSlot = 0;
    private VoltageSensor batteryVoltageSensor;
    // Add this variable at the top of your class with the others
    private int emptySlotCounter = 0;
    private static final int EMPTY_CONFIRM_THRESHOLD = 6; // How many loops to wait
    boolean pastCheck;
    private Follower poseUpdater;
    private Follower drive;
    private final Pose GOAL_POSE = new Pose(72, 36);
    private double lastTurretAngleDeg = 0;
    String[] scanResults = {"open", "open", "open"};
    boolean needScan = true;
    double sLastError = 0;
    private int intakeConfirmCounter = 0;
    private static final int INTAKE_CONFIRM_THRESHOLD = 4;
    private boolean slot0Valid = false;
    private boolean slot1Valid = false;
    private boolean slot2Valid = false;
    String targetColorMode = "all";
    private boolean goShoot = false;
    private int intakeStep = 0;
    private double txDeg, tyDeg;

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
        turnTurret = hardwareMap.get(DcMotorEx.class, "turnTurret");
        turnTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnTurret.setVelocityPIDFCoefficients(0.05, 0, 0.001, 12.1);
        // Follower after constants are set
        follower = Constants.createFollower(hardwareMap);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.06);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.94);
        popUp=hardwareMap.get(Servo.class, "popup");
        popUp.setPosition(0);
        spindexer=hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        colorBack = hardwareMap.get(RevColorSensorV3.class, "colorBack");
        color0 = hardwareMap.get(RevColorSensorV3.class, "color0");
        //color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        colorFront=hardwareMap.get(RevColorSensorV3.class, "colorFront");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        intake = hardwareMap.get(DcMotorEx.class,"intake");

        colorBack = hardwareMap.get(RevColorSensorV3.class, "colorBack");
        color0 = hardwareMap.get(RevColorSensorV3.class, "color0");
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        colorFront = hardwareMap.get(RevColorSensorV3.class, "colorFront");

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
            rpm = rpm + 100;
            //velocity = (rpm/60.0)*ticks;
            turret.setVelocity((int)Math.round(rpm));
            current = turret.getCurrentPosition() / 28.0;


        }
            rBumpLast = gamepad2.right_bumper;
        if (gamepad2.left_bumper && !lBumpLast)
        {
            rpm = rpm - 100;
            //velocity = (rpm/60.0)*ticks;
            turret.setVelocity((int)Math.round(rpm));


        }
            lBumpLast = gamepad2.left_bumper;

            if (gamepad1.y) {
                double currentPos = getSpindexerAngleDeg();
                double targetPos = intakeSlotPositions[currentSlot];
                long stepTime = System.currentTimeMillis() - sequenceStartTime;


                // 1. Calculate Forward Distance (to ensure it only spins one way)
                double error = targetPos - currentPos;
                if (error < 0) {
                    error += 360; // Force the motor to find the target by spinning forward
                }

                switch (intakeStep) {
                    case -1: // INITIAL DECISION (Lazy Logic)
                        currentSlot = getClosestForwardSlot(); // Use the helper method
                        intakeStep = 0;
                        sequenceStartTime = System.currentTimeMillis();
                        lastError = 0;
                        break;
                    case 0: // ALIGNING
                        // 2. PID Calculation
                        double dt = pidTimer.seconds();
                        pidTimer.reset();

                        // 1. Get current voltage
                        double currentVoltage = batteryVoltageSensor.getVoltage();

// 2. Calculate the compensation factor (Target 12V / Current)
// We use Math.max to prevent division by zero just in case
                        double voltageComp = 12.0 / Math.max(currentVoltage, 1.0);

                        double derivative = (error - lastError) / dt;
                        double power = (error * kP) + (derivative * kD);
                        lastError = error;

                        // Cap the power so it doesn't go crazy
                        power = Math.max(-0.5, Math.min(0.5, power));
                        double minPower = 0.15; // Minimum power to overcome friction

                        if (error > PositionToleranceDeg) {
                            double finalPower = Math.max(Math.abs(power), minPower) * Math.signum(power);
                            spindexer.setPower(finalPower * voltageComp);
                            sequenceStartTime = System.currentTimeMillis(); // Reset timer because we aren't there yet
                        } else {
                            spindexer.setPower(0);
                            // Wait for settle before turning on the intake motor
                            if (stepTime >= 40) {
                                intakeStep = 1;
                                sequenceStartTime = System.currentTimeMillis();
                            }
                        }
                        break;
                    case 1:
                        if (!isSpotTaken()) {
                            intake.setPower(0.85);
                            intakeConfirmCounter = 0;
                        } else {
                            intakeConfirmCounter++;

                            if (intakeConfirmCounter >= INTAKE_CONFIRM_THRESHOLD) {
                                intake.setPower(0);
                                currentSlot = (currentSlot + 1) % intakeSlotPositions.length;
                                intakeStep = 0;
                                sequenceStartTime = System.currentTimeMillis();
                                intakeConfirmCounter = 0;
                            }
                        }
                        break;
                }
            } else if (!gamepad1.y && !gamepad1.right_bumper && !gamepad1.dpad_left && !gamepad1.dpad_right){
                // Reset logic when button is released
                spindexer.setPower(0);
                intake.setPower(0);
                intakeStep = -1;
                lastError = 0;
            }
            telemetry.addData("intake", intakeStep);
            //telemetry.update();

            if (gamepad1.right_bumper)
            {
                targetColorMode = "all";
                goShoot = true;

            }
            else if (gamepad1.dpad_left)
            {
                targetColorMode = "green";
                goShoot = true;
            }
            else if (gamepad1.dpad_right)
            {
                targetColorMode = "purple";
                goShoot = true;
            }
            else {
                goShoot = false;
            }


            if (goShoot)
            {
                double currentSPos = getSpindexerAngleDeg();
                long stepTime = System.currentTimeMillis() - sequenceStartTime;

                switch (shootStep) {
                    case -1: // NEW: INITIAL DECISION
                        currentShootSlot = getClosestShootSlot();
                        shootStep = 0;
                        sequenceStartTime = System.currentTimeMillis();
                        break;

                    case 0: // ALIGNING
                        double targetSPos = shootSlotPositions[currentShootSlot];
                        double error = targetSPos - currentSPos;
                        if (error < 0) error += 360;

                        double dt = pidTimer.seconds();
                        pidTimer.reset();
                        double voltageComp = 12.0 / Math.max(batteryVoltageSensor.getVoltage(), 1.0);
                        double derivative = (error - lastError) / dt;
                        double power = (error * kP) + (derivative * kD);
                        lastError = error;

                        power = Math.max(-0.5, Math.min(0.5, power));

                        if (needScan)
                        {
                            if (error > PositionToleranceDeg) {
                                spindexer.setPower(power * voltageComp);
                                sequenceStartTime = System.currentTimeMillis();
                            } else {
                                spindexer.setPower(0);
                                scan();
                                if (stepTime >= 50) {
                                    shootStep = 1;
                                    sequenceStartTime = System.currentTimeMillis();
                                    emptySlotCounter = 0;
                                }
                            }
                        }
                        else {
                            shootStep = 1;
                            sequenceStartTime = System.currentTimeMillis();
                        }


                        break;

                    case 1: // CHECK FOR BALL
                        slot0Valid = false;
                        slot1Valid = false;
                        slot2Valid = false;

                        if (targetColorMode.equals("all"))
                        {
                            slot2Valid = scanResults[2].equals("green") || scanResults[2].equals("purple");
                            slot0Valid = scanResults[0].equals("green") || scanResults[0].equals("purple");
                            slot1Valid = scanResults[1].equals("green") || scanResults[1].equals("purple");
                        }
                        else {
                            slot2Valid = scanResults[2].equals(targetColorMode);
                            slot0Valid = scanResults[0].equals(targetColorMode);
                            slot1Valid = scanResults[1].equals(targetColorMode);
                        }
                        double tPos = -1;
                        if (slot2Valid)
                        {

                            tPos = shootSlotPositions[currentShootSlot];
                        }
                        else if (slot0Valid)
                        {
                            tPos = shootSlotPositions[(currentShootSlot + 1) % shootSlotPositions.length];

                        }
                        else if (slot1Valid)
                        {
                            tPos = shootSlotPositions[(currentShootSlot + 2) % shootSlotPositions.length];
                        }
                        else {
                            //all slots open
                            shootStep = 0;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        if (tPos !=- 1)
                        {
                            double err = tPos - currentSPos;
                            if (err < 0) err += 360;

                            double timeee = pidTimer.seconds();
                            pidTimer.reset();
                            double voltage = 12.0 / Math.max(batteryVoltageSensor.getVoltage(), 1.0);
                            double deriv = (err - sLastError) / timeee;
                            double pow = (err * kP) + (deriv * kD);
                            sLastError = err;
                            pow = Math.max(-0.5, Math.min(0.5, pow));

                            if (err > PositionToleranceDeg) {
                                spindexer.setPower(pow * voltage);
                                sequenceStartTime = System.currentTimeMillis();
                            } else {
                                spindexer.setPower(0);
                                if (stepTime >= 40) { // Wait for settle
                                    shootStep = 2;
                                    sequenceStartTime = System.currentTimeMillis();
                                    emptySlotCounter = 0;
                                }
                            }
                        }
                        break;

                    case 2: // POP UP
                        popUp.setPosition(0.4);
                        if (stepTime >= 300) {
                            shootStep = 3;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 3: // RETRACT & PREPARE NEXT
                        needScan = false;
                        popUp.setPosition(0);
                        // START MOVING TO NEXT SLOT WHILE RETRACTING
                        if (stepTime >= 150) {
                            currentShootSlot = (currentShootSlot + 1) % shootSlotPositions.length;
                            shootStep = 0;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                }
            }
            else if (!gamepad1.right_bumper && !gamepad1.y && !gamepad1.dpad_left && !gamepad1.dpad_right){
                spindexer.setPower(0);
                shootStep = -1;
                lastError = 0;
                popUp.setPosition(0);
                needScan = true;
            }

        //turret.setVelocity((int)Math.round(velocity));
            //telemetry.addData("target rpm", rpm);
            //telemetry.addData("actual rpm", (int)current);
            telemetry.addData("target velocity", (int)Math.round(rpm));
            telemetry.addData("turret velocity", (int)turret.getVelocity());

            if (limelight!=null)
            {
                LLResult ll = limelight.getLatestResult();
                telemetry.addData("Limelight", "Got result: %s", ll != null ? "Valid" : "Null");

                if (ll != null) {
                    boolean isValid = ll.isValid();
                    double tx = ll.getTx();
                    double ty = ll.getTy();
                    double ta = ll.getTa();
                    txDeg=tx;
                    tyDeg=ty;

                    angleAdjust(tx);
                    double dist=calculateDistance(ty, tx);
                    setTurretAngle(dist);

                    telemetry.addData("LL Valid", isValid);
                    //telemetry.addData("AprilTag ID", tid);
                    telemetry.addData("TX/TY/TA", "%.2f / %.2f / %.2f", tx, ty, ta);
                    telemetry.addData("Distance from Apriltag/Angle 0/Angle1:", "%.2f / %.2f / %.2f", dist, angleTurret0.getPosition(), angleTurret1.getPosition());
                }
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




        telemetry.update();



        }

}
    private void setTurretAngle(double dist)
    {
        if (dist>2)
        {
            angleTurret0.setPosition(0.02);
            angleTurret1.setPosition(0.98);
        }
        else if (dist>1.5)
        {
            angleTurret0.setPosition(0.04);
            angleTurret1.setPosition(0.96);
        }
        else if (dist>1)
        {
            angleTurret0.setPosition(0.07);
            angleTurret1.setPosition(0.93);
        }
        else if (dist>0.75)
        {
            angleTurret0.setPosition(0.11);
            angleTurret1.setPosition(0.89);
        }
        else if (dist<=0.75){
            angleTurret0.setPosition(0.13);
            angleTurret1.setPosition(0.87);
        }
        else {
            angleTurret0.setPosition(0.09);
            angleTurret1.setPosition(0.91);
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

    }

    private boolean isTargetColorDetected() {
        // Get raw color values
        int red = colorBack.red();
        int green = colorBack.green();
        int blue = colorBack.blue();
        NormalizedRGBA colors = colorBack.getNormalizedColors();
        if ((colors.blue)> colors.green && colors.blue>0.0013)
        {
            telemetry.addData("Color seen:", "purple");
            telemetry.addData("Color seen:", colors.blue);
            telemetry.update();
            return true;

        } else if(colors.green>(colors.blue) && colors.green>0.0013) {

            telemetry.addData("Color seen:", "green");
            telemetry.addData("Color seen:", colors.green);
            telemetry.update();
            return true;
        }
        telemetry.addData("Color seen:", "No Color");
        telemetry.update();
        return false;

    }

    double getSpindexerAngleDeg() {
        // REV Analog encoder stays between 0 and 3.3V usually
        double voltage = encoder.getVoltage();
        double maxVoltage = 3.3; // Check if your bot is 3.2 or 3.3
        double degrees = (voltage / maxVoltage) * 360.0;

        // Apply your specific mechanical offset
        double offset = -123.4;
        double finalAngle = (degrees + offset) % 360;
        if (finalAngle < 0) finalAngle += 360;

        return finalAngle;
    }
    private int getClosestShootSlot() {
        double currentPos = getSpindexerAngleDeg();
        int bestSlot = 0;
        double minForwardDistance = 400;

        for (int i = 0; i < shootSlotPositions.length; i++) {
            double distance = shootSlotPositions[i] - currentPos;
            if (distance < 0) distance += 360; // Forward only

            if (distance < minForwardDistance) {
                minForwardDistance = distance;
                bestSlot = i;
            }
        }
        return bestSlot;
    }
    private int getClosestForwardSlot() {
        double currentPos = getSpindexerAngleDeg();
        int bestSlot = currentSlot;
        double minForwardDistance = 400; // Larger than 360

        for (int i = 0; i < intakeSlotPositions.length; i++) {
            double distance = intakeSlotPositions[i] - currentPos;

            // If the distance is negative, it means the slot is "behind" us.
            // We add 360 to find the distance to reach it by spinning forward.
            if (distance < 0) {
                distance += 360;
            }

            if (distance < minForwardDistance) {
                minForwardDistance = distance;
                bestSlot = i;
            }
        }
        return bestSlot;
    }
    private boolean isSpotTaken() {
        // Get raw color values
        int red = colorFront.red();
        int green = colorFront.green();
        int blue = colorFront.blue();
        NormalizedRGBA colors = colorFront.getNormalizedColors();

        if ((colors.blue)> colors.green && colors.blue>0.0015)
        {
            telemetry.addData("Color seen:", "purple");
            telemetry.addData("Color seen:", colors.blue);
            telemetry.update();
            return true;

        } else if(colors.green>(colors.blue) && colors.green>0.0015) {

            telemetry.addData("Color seen:", "green");
            telemetry.addData("Color seen:", colors.green);
            telemetry.update();
            return true;
        }
        telemetry.addData("Color seen:", "No Color");
        telemetry.update();
        return false;

    }

    public void scan()
    {
        NormalizedRGBA in2Pos = colorBack.getNormalizedColors();
        NormalizedRGBA in0Pos = color0.getNormalizedColors();
        NormalizedRGBA in1Pos = color1.getNormalizedColors();

        if (in0Pos.blue > 0.001 && in0Pos.blue > in0Pos.green)
        {
            scanResults[0] = "purple";
        }
        else if (in0Pos.green>0.0013)
        {
            scanResults[0] = "green";
        }
        else {
            scanResults[0] = "open";
        }

        if (in1Pos.blue > 0.001 && in1Pos.blue > in1Pos.green)
        {
            scanResults[1] = "purple";
        }
        else if (in1Pos.green > 0.0013)
        {
            scanResults[1] = "green";
        }
        else {
            scanResults[1] = "open";
        }

        if (in2Pos.blue > 0.0012 && in2Pos.blue > in2Pos.green)
        {
            scanResults[2] = "purple";
        }
        else if (in2Pos.green > 0.0013)
        {
            scanResults[2] = "green";
        }
        else {
            scanResults[2] = "open";
        }


        telemetry.addData("scan results: ",scanResults[0] + ", " + scanResults[1] + ", " + scanResults[2]);
        telemetry.update();


    }
private void angleAdjust(double tx)
{
    if (tx>3)
    {
        turnTurret.setPower(0.18);
    }
    else if (tx<-3)
    {
        turnTurret.setPower(-0.18);
    }
    else
    {
        turnTurret.setPower(0);
    }
}}
