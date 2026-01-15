package org.firstinspires.ftc.teamcode.LAVA;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
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

//RED APRILTAG LIMELIGHT
//Unchanged
//JULIANNA
@TeleOp(group = "AAA", name ="lavaTele")
public class lavaTele extends LinearOpMode {
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
    int popSequenceStep = 0;
    int shootStep=0;
    boolean shootSequenceActive = false;
    boolean shootSequenceComplete = true;
    private final Pose startPose = new Pose(0, 0, 0);
    private RevColorSensorV3 colorBack, color0, color1, colorFront;
    private double txDeg, tyDeg;
    private double v;

    Servo angleTurret0, angleTurret1, popUp;
    DcMotorEx turret, intake, frontRight, frontLeft, backRight, backLeft, spindexer, turnTurret;
    boolean xLast, bLast, yLast, bPressable, yPressable, aLast, aPressable, rbumpLast, rbumpPressable, b1Last, b1Pressable, x1Last, x1Pressable;

    @Override
    public void runOpMode() throws InterruptedException {
        //drive motor init
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Follower after constants are set
        follower = Constants.createFollower(hardwareMap); //Haolin was here lol
        follower.setStartingPose(startPose);

        //other motor init
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turnTurret = hardwareMap.get(DcMotorEx.class, "turnTurret");
        turnTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
                    if (gamepad1.b)
                    {
                        setTurretVelocity(dist); //not sure if i didnt fuck this up sorry
                    }
                    else
                    {
                        turret.setVelocity(0);
                    }

                    telemetry.addData("LL Valid", isValid);
                    //telemetry.addData("AprilTag ID", tid);
                    telemetry.addData("TX/TY/TA", "%.2f / %.2f / %.2f", tx, ty, ta);
                    telemetry.addData("Distance from Apriltag/Angle 0/Angle1:", "%.2f / %.2f / %.2f", dist, angleTurret0.getPosition(), angleTurret1.getPosition());
                }
            }

            if (gamepad1.left_bumper && (limelight==null || limelight.getLatestResult()==null))
            {
                turret.setTargetPosition(0);
            }

            //Shoot macro
            if (gamepad1.right_bumper) {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                telemetry.addData("Shoot Order:", "General Shoot");
                switch (shootStep) {
                    case 0:
                        //turret.setPower(0.6);
                        if (elapsedTime >= 300) {
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        boolean isColorDetected = isTargetColorDetected();
                        if (isColorDetected && !wasColorDetected) {
                            // Color just detected, stop the spindexer
                            spindexer.setPower(0);
                            wasColorDetected = true;
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        } else if (!isColorDetected) {
                            // No color detected, keep spinning
                            spindexer.setPower(0.25); // Adjust power as needed
                            wasColorDetected = false;
                        }
                        else if (elapsedTime >= 5000) {
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        if (elapsedTime>=200)
                        {
                            popUp.setPosition(0.51); //ALL THE WAY UP
                            ballcount--;
                        }
                        if (elapsedTime >= 450) {
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        popUp.setPosition(0);
                        //turret.setPower(0);
                        if (elapsedTime >= 500) {
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        shootSequenceComplete = true;
                        shootSequenceActive = false;
                        sequenceStartTime = 0;
                        shootStep = 0;
                        break;

                }
            }

            else if (gamepad1.dpad_left) //Just green
            {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                telemetry.addData("Shoot Order:", "Green");
                switch (shootStep)
                {
                    case 0:
                        boolean isGreenDetected = greenDetect();
                        if (isGreenDetected && !wasColorDetected)
                        {
                            spindexer.setPower(0);
                            wasColorDetected = true;
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        else if (!isGreenDetected)
                        {
                            spindexer.setPower(0.25);
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
                            popUp.setPosition(0.51); //ALL THE WAY UP
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
                        break;
                }
            }
            else if (gamepad1.dpad_right) //just purple
            {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                telemetry.addData("Shoot Order:", "Purple");
                switch (shootStep)
                {
                    case 0:
                        boolean isPurpleDetected = purpleDetect();
                        if (isPurpleDetected && !wasColorDetected)
                        {
                            spindexer.setPower(0);
                            wasColorDetected = true;
                            shootStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        else if (!isPurpleDetected)
                        {
                            spindexer.setPower(0.25);
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
                            popUp.setPosition(0.51); //ALL THE WAY UP
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
                        break;
                }

            }
            else if (!gamepad1.right_bumper && !gamepad1.y && !gamepad1.dpad_left && !gamepad1.dpad_right){
                wasColorDetected = false;
                // Stop spindexer when bumper is released
                spindexer.setPower(0);
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
                        /*
                        else if (elapsedTime >= 400) {
                            intakeStep++;
                            sequenceStartTime = System.currentTimeMillis();
                            }

                         */
                            break;

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
            else if (gamepad1.a)
            {
                intake.setPower(-0.6);
            }
            else if (!gamepad1.y && !gamepad1.right_bumper && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.a){
                intake.setPower(0);
                spindexer.setPower(0);
            }
            else {
                intake.setPower(0);
            }
            if (gamepad1.x)
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
        if (intakeColors.red>0.001 && !(intakeColors.green>0.0016) && !(intakeColors.blue>0.0015))//(intakeColors.red>intakeColors.green && intakeColors.red>intakeColors.blue)
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
            angleTurret0.setPosition(0.025);
            angleTurret1.setPosition(0.975);
        }
        else if (dist>1)
        {
            angleTurret0.setPosition(0.045);
            angleTurret1.setPosition(0.955);
        }
        else if (dist>0.75)
        {
            angleTurret0.setPosition(0.09);
            angleTurret1.setPosition(0.91);
        }
        else if (dist<=0.75){
            angleTurret0.setPosition(0.11);
            angleTurret1.setPosition(0.89);
        }
        else {
            angleTurret0.setPosition(0.02);
            angleTurret1.setPosition(0.98);
        }
    }

    private void setTurretVelocity(double dist)
    {



        //double velocity = (-58.21*(dist*dist)) + (550.8*dist) + 820; OLD EQUATION
        double velocity = 1392.5*(Math.pow(dist, 0.173));//0.173 original
        if (dist >1.2 && dist <1.5)
        {
            velocity = velocity -60;
        }

        if (dist >=1.5 && dist <= 1.8)
        {
            velocity = velocity -40;
        }
        if (dist > 1.8 && dist < 2.75)
        {
            velocity = velocity - 100;
        }
        if (dist>3)
        {
            velocity = velocity + 50;
        }
        int tolerance = (int)(turret.getVelocity()-velocity);

        if (tolerance > 10)
        {
            velocity = velocity - tolerance;
        }
        else if (tolerance < -10)
        {
            velocity = velocity - tolerance;
        }



        v = velocity;
        turret.setVelocity((int)Math.round(velocity));
        telemetry.addData("velocity", (int)Math.round(velocity));
        telemetry.update();
    }

    private boolean greenDetect()
    {
        NormalizedRGBA colors = color0.getNormalizedColors();
        if(colors.green>(colors.blue) && colors.green>colors.red && colors.green>0.00195) {

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
        NormalizedRGBA colors = color0.getNormalizedColors();

        if ((colors.blue)> colors.green && colors.blue>0.0016)
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
        spindexer.setPower(0.2);

    }
}
