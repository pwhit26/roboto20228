package org.firstinspires.ftc.teamcode.ICE;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

//RED APRILTAG LIMELIGHT
//Unchanged
//JULIANNA
@TeleOp(name ="DecodingByEncoding")
public class DecodingByEncoding extends LinearOpMode {
    private Follower follower;
    private static final double MIN_COLOR_THRESHOLD = 0.5; // 50% of total area
    private boolean wasColorDetected = false;
    private boolean intakeSequenceActive= false;
    private boolean intakeSequenceComplete = true;
    private int ballcount=0;
    //private boolean intakeSlow=false;
    private int intakeStep=0;
    private Limelight3A limelight;
    private int spinPos;
    private boolean on = true;
    public ElapsedTime runtime = new ElapsedTime();
    boolean popSequenceActive = false;
    boolean popSequenceComplete = true;
    long sequenceStartTime = 0;
    long popStartTime = 0;
    int popSequenceStep = 0;
    int shootStep=0;
    boolean shootSequenceActive = false;
    boolean shootSequenceComplete = true;
    private final Pose startPose = new Pose(0, 0, 0);
    private RevColorSensorV3 colorBack, color0, color1, colorFront;
    private double txDeg, tyDeg;
    private double v;
    private int initialPos=0;
    long Id;
    boolean unstuckActive = false;
    long unstuckStartTime = 0;
    int colorConfirmCount = 0;
    static final int COLOR_CONFIRM_THRESHOLD = 3;
    boolean shootingActive = false;
    long shootStepStartTime = 0;
    boolean ballLatched = false;
    boolean readyForPopup = false;
    int intakeCount = 0;
    int shootCount = 0;
    boolean dpadLlast, dpadRlast, RbumpLast = false;



    Servo angleTurret0, angleTurret1, popUp;
    DcMotorEx turret, intake, frontRight, frontLeft, backRight, backLeft, spindexer, turnTurret;
    boolean xLast, bLast, yLast, bPressable, yPressable, aLast, aPressable, rbumpLast, rbumpPressable, b1Last, b1Pressable, x1Last, x1Pressable;


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
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Follower after constants are set
        follower = Constants.createFollower(hardwareMap); //Haolin was here lol
        follower.setStartingPose(startPose);

        //other motor init
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        turret.setVelocityPIDFCoefficients(0.05, 0, 0.001, 12.1);
        turnTurret = hardwareMap.get(DcMotorEx.class, "turnTurret");
        turnTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        int spindexStart = spindexer.getCurrentPosition();
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(0.5);

        colorBack = hardwareMap.get(RevColorSensorV3.class, "colorBack");
        color0 = hardwareMap.get(RevColorSensorV3.class, "color0");
        //color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        colorFront=hardwareMap.get(RevColorSensorV3.class, "colorFront");


        //servo init
        popUp = hardwareMap.get(Servo.class, "popup");
        popUp.setPosition(0);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.11);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.89);

        if (limelight != null) {
            limelight.pipelineSwitch(1);  // Changed to pipeline 0 for AprilTag
            limelight.start();
            telemetry.addData("Limelight", "Initialized - Pipeline: %d", 1);
        } else {
            telemetry.addData("Limelight", "Not found in hardware map!");
        }

        waitForStart();
        follower.startTeleopDrive();
        runtime.reset();
        while (opModeIsActive()) {
            //BASE TELE W RED LIMELIGHT
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

            if (limelight!=null)
            {
                LLResult ll = limelight.getLatestResult();
                telemetry.addData("Limelight", "Got result: %s", ll != null ? "Valid" : "Null");
                LLResultTypes.FiducialResult targetTag = null;

                if (ll != null) {
                    List<LLResultTypes.FiducialResult> fiducials = ll.getFiducialResults();

                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        long tagId = fr.getFiducialId();
                        telemetry.addData("Detected Tag", tagId);

                        if (tagId == 20) {
                            Id = tagId;
                            targetTag = fr;
                            break; // stop once we found red goal
                        }
                    }
                    if (targetTag != null && Id == 20)
                    {
                        boolean isValid = ll.isValid();
                        double tx = ll.getTx();
                        double ty = ll.getTy();
                        double ta = ll.getTa();
                        txDeg=tx;
                        tyDeg=ty;


                        double dist=calculateDistance(ty, tx);
                        angleAdjust(tx, dist);
                        setTurretAngle(dist);
                        if (gamepad1.b)
                        {
                            setTurretVelocity(dist); //not sure if i didnt fuck this up sorry
                        }
                        else
                        {
                            turret.setVelocity(800);
                        }

                        telemetry.addData("LL Valid", isValid);
                        //telemetry.addData("AprilTag ID", tid);
                        telemetry.addData("TX/TY/TA", "%.2f / %.2f / %.2f", tx, ty, ta);
                        telemetry.addData("Distance from Apriltag/Angle 0/Angle1:", "%.2f / %.2f / %.2f", dist, angleTurret0.getPosition(), angleTurret1.getPosition());
                    }
                    else {
                        if (gamepad1.b)
                        {
                            turret.setVelocity(1500);
                        }
                        else {
                            turret.setVelocity(800);
                        }
                    }


                }
            }



            //Shoot macro
            // ================= SHOOTING SEQUENCE =================

// Start shooting sequence
            if (gamepad1.right_bumper && !shootingActive) {
                shootingActive = true;
                shootStep = 0;
                shootStepStartTime = System.currentTimeMillis();
                colorConfirmCount = 0;
                ballLatched = false;
                readyForPopup = false;
                spindexer.setPower(0.35);
                shootCount++;
                intakeCount++;
                spindexer.setTargetPosition(shootCount * 180);
                RbumpLast = false;

            }

// Default safety
            /*if (!shootingActive && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.x && !gamepad1.y && !gamepad2.a && !gamepad2.b) {

            }*/
            // HARD CANCEL: release button aborts shooting
            if (!gamepad1.right_bumper && shootingActive && shootStep == 0) {
                shootingActive = false;
                shootStep = 0;
                //spindexer.setPower(0);
                popUp.setPosition(0);
            }

// Run sequence
            if (shootingActive) {
                long elapsed = System.currentTimeMillis() - shootStepStartTime;

                switch (shootStep) {

                    // ------------------------------------------------
                    // STEP 0: Spin spindexer fast toward shooter
                    // ------------------------------------------------
                    case 0:
                        spindexer.setTargetPosition(0);  // FAST FEED

                        if (greenDetect() || purpleDetect()) {
                            colorConfirmCount++;
                        } else {
                            colorConfirmCount = 0;
                        }

                        // Once color is confirmed, slow down
                        if (colorConfirmCount >= COLOR_CONFIRM_THRESHOLD) {
                            shootStep = 1;
                            shootStepStartTime = System.currentTimeMillis();
                        }
                        break;

                    // ------------------------------------------------
                    // STEP 1: Slow crawl + latch detection
                    // ------------------------------------------------
                    case 1:
                        //spindexer.setPower(0.025); // CRAWL SPEED

                        if (!ballLatched) {
                            ballLatched = true;
                            shootStepStartTime = System.currentTimeMillis();
                        }

                        // Micro-advance to center ball
                        if (elapsed >= 10) {
                            //spindexer.setPower(0);
                            shootStep = 2;
                            shootStepStartTime = System.currentTimeMillis();
                        }
                        break;

                    // ------------------------------------------------
                    // STEP 2: Popup up (spindexer fully stopped)
                    // ------------------------------------------------
                    case 2:
                        //spindexer.setPower(0);
                        popUp.setPosition(0.45); // POPUP UP

                        if (elapsed >= 450) {
                            shootStep = 3;
                            shootStepStartTime = System.currentTimeMillis();
                        }
                        break;

                    // ------------------------------------------------
                    // STEP 3: Popup down & reset
                    // ------------------------------------------------
                    case 3:
                        popUp.setPosition(0.0); // POPUP DOWN

                        if (elapsed >= 200) {
                            shootingActive = false;
                            shootStep = 0;
                            colorConfirmCount = 0;
                            ballLatched = false;
                            RbumpLast = true;
                        }
                        break;
                }
            }


            else if (gamepad1.dpad_left) //Just green
            {
                spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                telemetry.addData("Shoot Order:", "Green");
                shootCount++;
                intakeCount++;
                dpadLlast = false;
                switch (shootStep)
                {
                    case 0:
                        spindexer.setTargetPosition(shootCount*180);
                        boolean isGreenDetected = greenDetect();
                        if (isGreenDetected && !wasColorDetected)
                        {
                            spindexer.setPower(0.35);
                            wasColorDetected = true;
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        else if (!isGreenDetected)
                        {
                            shootCount++;
                            intakeCount++;
                            //spindexer.setPower(0.25);
                            wasColorDetected=false;
                        }
                        else if (elapsedTime >= 2000) {
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        if (elapsedTime>=200)
                        {
                            popUp.setPosition(0.45); //ALL THE WAY UP
                            ballcount--;
                        }

                        if (elapsedTime >= 450) {
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        popUp.setPosition(0);
                        //turret.setPower(0);
                        if (elapsedTime >= 500) {
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        shootSequenceComplete = true;
                        shootSequenceActive = false;
                        sequenceStartTime = 0;
                        shootStep = 0;
                        dpadLlast = true;
                        break;
                }
            }
            else if (gamepad1.dpad_right) //just purple
            {
                spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                telemetry.addData("Shoot Order:", "Purple");
                dpadRlast = false;
                switch (shootStep)
                {
                    case 0:
                        spindexer.setTargetPosition(shootCount*180);
                        boolean isPurpleDetected = purpleDetect();
                        if (isPurpleDetected && !wasColorDetected)
                        {
                            spindexer.setPower(0.35);
                            wasColorDetected = true;
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        else if (!isPurpleDetected)
                        {
                            shootCount++;
                            intakeCount++;

                            //spindexer.setPower(0.25);
                            wasColorDetected=false;
                        }
                        else if (elapsedTime >= 2000) {
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        if (elapsedTime>=200)
                        {
                            popUp.setPosition(0.45); //ALL THE WAY UP 0.51
                        }

                        if (elapsedTime >= 450) {
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        popUp.setPosition(0);
                        //turret.setPower(0);
                        if (elapsedTime >= 500) {
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        shootSequenceComplete = true;
                        shootSequenceActive = false;
                        sequenceStartTime = 0;
                        shootStep = 0;
                        dpadRlast = true;
                        break;
                }

            }
            else if (!gamepad1.right_bumper && !gamepad1.y && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad2.b && !gamepad1.x && !gamepad1.dpad_up && !gamepad2.a){
                wasColorDetected = false;
                // Stop spindexer when bumper is released
                //spindexer.setPower(0);
                shootStep=0;
            }
            else {
                wasColorDetected = false;
            }



            //Turret Power
            if (gamepad1.b && (limelight==null || limelight.getLatestResult()==null))
            {
                turret.setPower(0.6);
                telemetry.addData("turret power:", turret.getPower());
                //telemetry.update();
            }
            else {
                turret.setPower(0);
            }








            //intake

           /* if (gamepad1.y)
            {
                spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spindexer.setTargetPosition(initialPos);
                spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spindexer.setPower(0.3);
                initialPos=initialPos+175;
                intake.setPower(0.6);

            }*/
            /*
            if (gamepad1.y)
            {
                intakeSequenceComplete = false;
                intakeSequenceActive = true;
                //spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //spindexer.setTargetPosition(initialPos);
                //spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //spindexer.setPower(0.3);
                //spindexer.setTargetPosition(0);
                //spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //spindexer.setPower(0.3);


                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                switch (intakeStep)
                {
                    case 0:

                        //setInitialPos();
                        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        spindexer.setTargetPosition(initialPos);
                        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        spindexer.setPower(0.3);

                        if (elapsedTime>=300)
                        {
                            intakeStep++;
                            sequenceStartTime=System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        if (!isSpotTaken())
                        {
                            intake.setPower(0.63);
                            telemetry.addData("Intake Power", intake.getPower());
                            if (elapsedTime>=800)
                            {
                                intakeStep++;
                                sequenceStartTime=System.currentTimeMillis();
                            }
                        }
                        if (isSpotTaken())
                        {
                          //  intake.setPower(0);
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        intake.setPower(0);
                        //initialPos=initialPos+175;
                        spindexer.setTargetPosition(initialPos + 190);
                        initialPos = spindexer.getCurrentPosition();
                        if (elapsedTime>=300)
                        {
                            intakeStep++;
                            sequenceStartTime=System.currentTimeMillis();
                        }
                        break;

                    case 3:
                        intakeSequenceComplete = true;
                        intakeSequenceActive = false;
                        sequenceStartTime = 0;

                        //intakeStep = 0;
                        initialPos = 0;
                        break;


                }
            }


             */
/*
            if (gamepad1.y) {

                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                //ballcount=0;
                switch (intakeStep)
                {
                    case 0:
                        intake.setPower(0);
                        spindexer.setPower(0.1775);
                        if (elapsedTime >= 50) {
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        boolean intakeGo = intakeTimingDetection();
                        if (intakeGo && !isSpotTaken())
                        {
                            intake.setPower(0.63);
                            spindexer.setPower(0);
                            wasColorDetected = true;
                            telemetry.addData("Intake Power", intake.getPower());
                            //telemetry.update();
                        }
                        if (isSpotTaken())
                        {
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }

                        //else if (elapsedTime >= 400) {
                            //intakeStep++;
                            //sequenceStartTime = System.currentTimeMillis();
                            //}


                            //break;


                    case 2:
                        if (isTargetColorDetected())
                        {
                            ballcount++;

                        }
                        telemetry.addData("Intake Power", intake.getPower());
                        telemetry.addData("Ball Count:", ballcount);
                        //telemetry.update();
                        spindexer.setPower(0.173);
                        intake.setPower(0);
                        intakeStep++;
                        sequenceStartTime = System.currentTimeMillis();
                        break;
                    case 3:
                        intakeSequenceComplete = true;
                        intakeSequenceActive = false;
                        sequenceStartTime = 0;
                        intakeStep = 0;
                        break;
                }

            }
*/

            //intake
            if (gamepad1.y) {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                int spinnn = spindexer.getCurrentPosition();
                //intakeCount++;
                shootCount++;
                //ballcount=0;
                switch (intakeStep)
                {
                    case 0:
                        if (rbumpLast || dpadLlast || dpadRlast)
                        {
                            spindexer.setTargetPosition(spinnn + 90);
                        }

                        if (!spindexer.isBusy())
                        {
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        intake.setPower(0);
                        //spindexer.setPower(0.3);
                        spindexer.setTargetPosition(175*intakeCount);
                        if (!spindexer.isBusy()) {
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        //boolean intakeGo = intakeTimingDetection();
                        if (!isSpotTaken())
                        {
                            intake.setPower(0.63);
                            //spindexer.setPower(0);
                            telemetry.addData("Intake Power", intake.getPower());
                            //telemetry.update();
                        }
                        else if (isSpotTaken())
                        {
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        /*
                        else if (elapsedTime >= 400) {
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                            }

                         */
                        break;

                    case 3:
                        telemetry.addData("Intake Power", intake.getPower());
                        //telemetry.update();
                        intake.setPower(0);
                        intakeCount++;
                        //shootCount++;
                        //spindexer.setTargetPosition(95 + (180*intakeCount));
                        if (elapsedTime>=200)
                        {
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        intakeSequenceComplete = true;
                        intakeSequenceActive = false;
                        sequenceStartTime = 0;
                        intakeStep = 0;
                        rbumpLast=false;
                        dpadRlast=false;
                        dpadLlast=false;
                        break;
                }}





            /*if (gamepad1.y)
            {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                switch (intakeStep)
                {
                    case 0:
                        spindexer.setVelocity(400);
                        intake.setPower(0);
                        if (elapsedTime>=150)
                        {
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        if (intakeTimingDetection() && !isSpotTaken())
                        {
                            spindexer.setVelocity(0);
                            intake.setPower(0.6);
                            if (elapsedTime>=800)
                            {
                                intakeStep++;
                                sequenceStartTime = System.currentTimeMillis();
                            }
                        }
                        else if (isSpotTaken())
                        {
                            intakeStep++;
                        }
                        break;
                    case 2:
                        intake.setPower(0);
                        spindexer.setVelocity(400);
                        if (elapsedTime>=150)
                        {
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        sequenceStartTime = 0;
                        intakeStep = 0;
                        break;
                }




            */
            else if (gamepad1.a && !gamepad1.left_bumper)
            {
                intake.setPower(-0.6);
            }
            else if (gamepad1.left_bumper)
            {
                intake.setPower(0.6);
            }
            else if (!gamepad1.y && !gamepad1.right_bumper && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.a && !gamepad1.left_bumper && !gamepad1.x && !gamepad1.dpad_up){
                intake.setPower(0);
            }
            else {
                intake.setPower(0);
                spindexer.setPower(0);
            }
            //spindexer.setTargetPosition(initialPos);
            //yLast = gamepad1.y;
            if (gamepad1.x && !gamepad1.y)
            {
                unstuck();
            }

            if (gamepad2.y)
            {
                intakeTimingDetection();
            }
            if (gamepad2.x)
            {
                isSpotTaken();
            }
            if ((gamepad2.a || gamepad1.dpad_up) && !popSequenceActive) {
                popSequenceActive = true;
                popSequenceStep = 0;
                popStartTime = System.currentTimeMillis();
            }
            if (popSequenceActive) {
                long elapsedTime = System.currentTimeMillis() - popStartTime;

                switch (popSequenceStep) {
                    case 0:
                        spindexer.setPower(0.2);
                        spindexer.setTargetPosition(spindexer.getCurrentPosition() - 35);
                        telemetry.addLine("Case 0");
                        if (elapsedTime >= 250) {
                            popSequenceStep++;
                            popStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 1:
                        //spindexer.setPower(0);
                        popUp.setPosition(0.45);
                        telemetry.addLine("Case 1");
                        if (elapsedTime >= 600) {
                            popSequenceStep++;
                            popStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 2:
                        popUp.setPosition(0);
                        telemetry.addLine("Case 2");
                        if (elapsedTime >= 200) {
                            popSequenceStep++;
                            popStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 3:
                        telemetry.addLine("Case 3");
                        popSequenceActive = false;
                        popSequenceStep = 0;
                        break;
                }
            }
            if (gamepad2.b)
            {
                purpleDetect();
            }

            if (gamepad2.left_bumper)
            {
                turnTurret.setPower(-0.25);
            }
            else if (gamepad2.right_bumper)
            {
                turnTurret.setPower(0.25);
            }
            else {
                //hi

            }
            if (gamepad2.b)
            {
                spindexer.setTargetPosition(spindexer.getCurrentPosition() - 35);
            }
            else {
            }

            telemetry.update();



        }
    }
    private boolean isTargetColorDetected() {
        // Get raw color values
        int red = color0.red();
        int green = color0.green();
        int blue = color0.blue();
        NormalizedRGBA colors = color0.getNormalizedColors();
        if ((colors.blue)> colors.green && colors.blue>0.0017)
        {
            telemetry.addData("Color seen:", "purple");
            telemetry.addData("Color seen:", colors.blue);
            telemetry.update();
            return true;

        } else if(colors.green>(colors.blue) && colors.green>0.00195) {

            telemetry.addData("Color seen:", "green");
            telemetry.addData("Color seen:", colors.green);
            telemetry.update();
            return true;
        }
        telemetry.addData("Color seen:", "No Color");
        telemetry.update();
        return false;

    }
    private boolean intakeTimingDetection()
    {
        NormalizedRGBA intakeColors = colorBack.getNormalizedColors();
        if (intakeColors.red>0.001 && !(intakeColors.green>0.002) && !(intakeColors.blue>0.002))//(intakeColors.red>intakeColors.green && intakeColors.red>intakeColors.blue)
        {
            telemetry.addData("Color seen:", "red");
            telemetry.addData("Color seen:", intakeColors.red);
            telemetry.update();
            return true;
        }
        telemetry.addData("Color seen:", "No Color");
        telemetry.update();
        return false;
    }
    private void angleAdjust(double tx, double dist)
    {
        if (Id == 20)
        {
            if (dist > 2.2 && dist<2.9)
            {
                if (tx>4)
                {
                    turnTurret.setPower(0.173);
                }
                else if (tx<-2)
                {
                    turnTurret.setPower(-0.173);
                }
                else
                {
                    turnTurret.setPower(0);
                }
            }
            else {
                if (tx>5)
                {
                    turnTurret.setPower(0.173);
                }
                else if (tx<-1)
                {
                    turnTurret.setPower(-0.173);
                }
                else
                {
                    turnTurret.setPower(0);
                }
            }
        }
    }


    private void setTurretAngle(double dist)
    {
        if (dist>=2.9)
        {
            angleTurret0.setPosition(0.01);
            angleTurret1.setPosition(0.99);
        }
        else if (dist>2.2)
        {
            angleTurret0.setPosition(0.02);
            angleTurret1.setPosition(0.98);
        }
        else if (dist>1.5)
        {
            angleTurret0.setPosition(0.05);
            angleTurret1.setPosition(0.95);
        }
        else if (dist>1)
        {
            angleTurret0.setPosition(0.07);
            angleTurret1.setPosition(0.93);
        }
        else if (dist>0.75)
        {
            angleTurret0.setPosition(0.1);
            angleTurret1.setPosition(0.9);
        }
        else if (dist<=0.75){
            angleTurret0.setPosition(0.12);
            angleTurret1.setPosition(0.88);
        }
        else {
            angleTurret0.setPosition(0.03);
            angleTurret1.setPosition(0.97);
        }
    }

    private void setTurretVelocity(double dist)
    {



        //double velocity = (-58.21*(dist*dist)) + (550.8*dist) + 820; OLD EQUATION
        double velocity = 1367.6*(Math.pow(dist, 0.19183)) + 30;//0.173 original
        if (dist<=1.2)
        {
            velocity = velocity +35;
        }
        if (dist >1.2 && dist <1.5)
        {
            velocity = velocity + 20;
        }

        if (dist >=1.5 && dist <= 1.8)
        {
            velocity = velocity - 20;
        }
        if (dist > 1.8 && dist < 2.2)
        {
            velocity = velocity -50;
        }
        if (dist >=2.2 && dist<2.9)
        {
            velocity = velocity + 5;
        }
        if (dist>3)
        {
            velocity = velocity - 5;
        }

        if (gamepad1.right_bumper)
        {
            velocity = velocity + 75;
        }






        v = velocity;
        turret.setVelocity(velocity);
        telemetry.addData("Target velocity", (int)Math.round(velocity));
        telemetry.addData("Real velocity: ", (int)turret.getVelocity());
        telemetry.update();
    }
    /* int tolerance = (int)(turret.getVelocity()-velocity);

        if (tolerance > 15)
        {
            turret.setVelocity((int)Math.round(velocity));
            turret.setVelocity(turret.getVelocity() - tolerance); //velocity = velocity - tolerance;
        }
        else if (tolerance < -15)
        {
            turret.setVelocity((int)Math.round(velocity));
            turret.setVelocity(turret.getVelocity() - tolerance);//velocity = velocity - tolerance;
        }
        else {
            turret.setVelocity((int)Math.round(velocity));
        }*/

    private boolean greenDetect()
    {
        NormalizedRGBA colors = colorBack.getNormalizedColors();
        if(colors.green>(colors.blue) && colors.green>colors.red && colors.green>0.0025) {

            telemetry.addData("Color seen:", "green");
            telemetry.addData("Color seen:", colors.green);
            telemetry.update();
            return true;
        }
        telemetry.addData("Color seen:", "No Color");
        telemetry.update();
        return false;

    }
    private boolean purpleDetect()
    {
        NormalizedRGBA colors = colorBack.getNormalizedColors();

        if ((colors.blue)> colors.green && colors.blue>0.0025)
        {
            telemetry.addData("Color seen:", "purple");
            telemetry.addData("Color seen:", colors.blue);
            telemetry.update();
            return true;

        }
        telemetry.addData("Color seen:", "No Color");
        telemetry.update();
        return false;
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
    private double calculateDistance(double ty, double tx) {
        // Camera configuration (adjust these values)
        double cameraHeightM = 0.3;      // Height of camera from ground in meters
        double tagHeightM = 0.75;         // Height of AprilTag from ground
        double cameraMountPitchDeg = 16.0; // Camera angle from horizontal

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
    private boolean shooterReady(double velocity){
        double rpmTolerance = 25;
        return (Math.abs(turret.getVelocity() - velocity) < rpmTolerance);
    }
    private void unstuck()
    {
        popUp.setPosition(0);
        spindexer.setTargetPosition(spindexer.getCurrentPosition() - 40);

    }
    private void setInitialPos()
    {
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setPower(0.2);
        if (intakeTimingDetection())
        {
            spindexer.setPower(0);
        }
    }
}
