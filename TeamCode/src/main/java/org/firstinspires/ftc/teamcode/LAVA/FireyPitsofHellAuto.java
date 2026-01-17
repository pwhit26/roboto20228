package org.firstinspires.ftc.teamcode.LAVA;
// shoot far, grab preset
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//REAL CLOSE BLUE AUTO
//GOOD AUTO USE THIS ONE!!!!!!
//hellooooo
@Autonomous
public class FireyPitsofHellAuto extends OpMode {
    private Follower follower;
    private Pose start, shoot, shoot2, preScoop1, scoop1, preScoop2, scoop2, preScoop3, scoop3, parky, shootAgain;
    private PathChain startShoot, shootPre1, preSco1,sco1Sho,shootPre2, park, preSco2,sco2Sho, intake2, intake3;
    String pathState="";
    long startTime = 0;
    int pathStage = 0; // 0 = not started, 1 = first path, 2 = second path, 3 = done
    public ElapsedTime runtime = new ElapsedTime();
    int pos = 0;
    int stage=0;
    long startT=0;
    ElapsedTime time = new ElapsedTime();
    long elapsed = System.currentTimeMillis() - startT;
    boolean shootSequenceActive = false;
    boolean shootSequenceComplete = true;
    boolean preScoStarted=false;
    //helloooo

    private double v;
    private double txDeg, tyDeg;
    //hi
    Limelight3A limelight;
    RevColorSensorV3 colorBack, colorFront, color0;
    Servo angleTurret0, angleTurret1, popUp;
    DcMotorEx turret, intake, spindexer, frontRight, frontLeft, backRight, backLeft, turnTurret;
    boolean xLast, bLast, lbumpLast, bPressable, xPressable, lbumpPressable, aLast, aPressable, rbumpLast, rbumpPressable, b1Last, b1Pressable, x1Last, x1Pressable;


    @Override
    public void init() {
        //drive motor init
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

        //other motor init
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turret=hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //servo init
        popUp = hardwareMap.get(Servo.class, "popup");
        popUp.setPosition(0);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.08);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.92);

        turnTurret = hardwareMap.get(DcMotorEx.class, "turnTurret");
        turnTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTurret.setTargetPosition(0);
        turnTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnTurret.setPower(0.18);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(0.5);
        colorBack = hardwareMap.get(RevColorSensorV3.class, "colorBack");
        color0 = hardwareMap.get(RevColorSensorV3.class, "color0");
        //color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        colorFront=hardwareMap.get(RevColorSensorV3.class, "colorFront");

        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize poses - adjust these values to match your field setup
        start = new Pose(0, 0, Math.toRadians(0));
        shoot = new Pose(9, 0, Math.toRadians(-23.5));
        shoot2 = new Pose(9, 0, Math.toRadians(-24));
        preScoop1 = new Pose(24.5, -10, Math.toRadians(-90));
        scoop1 = new Pose(25,-44, Math.toRadians(-90));
        scoop2 = new Pose(25, -45, Math.toRadians(-90));
        scoop3 = new Pose(25, -48, Math.toRadians(-90));
        parky = new Pose(23, 0, Math.toRadians(0));


        /*shootAgain = new Pose (80, -10, Math.toRadians(24));
        preScoop2 = new Pose(40, 0, Math.toRadians(90));
        scoop2 = new Pose(40,-32, Math.toRadians(90));
        preScoop3 = new Pose(17, 0, Math.toRadians(90));
        scoop3 = new Pose(17,-32, Math.toRadians(90));*/





        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);

        // Build paths
        startShoot = follower.pathBuilder()
                .addPath(new BezierLine(start, shoot))
                .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
                .build();

        shootPre1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot, preScoop1))
                .setLinearHeadingInterpolation(shoot.getHeading(), preScoop1.getHeading())
                .build();

        preSco1 = follower.pathBuilder()
                .addPath(new BezierLine(preScoop1, scoop1))
                .setLinearHeadingInterpolation(preScoop1.getHeading(), scoop1.getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(scoop1, scoop2))
                .setLinearHeadingInterpolation(scoop1.getHeading(), scoop2.getHeading())
                .build();
        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(scoop2, scoop3))
                .setLinearHeadingInterpolation(scoop2.getHeading(), scoop3.getHeading())
                .build();

        shootPre2 = follower.pathBuilder()
                .addPath(new BezierLine(scoop3, shoot2))
                .setLinearHeadingInterpolation(scoop3.getHeading(), shoot2.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(shoot, parky))
                .setLinearHeadingInterpolation(shoot.getHeading(), parky.getHeading())
                .build();

        /*preSco2 = follower.pathBuilder()
                .addPath(new BezierLine(preScoop2, scoop2))
                .setLinearHeadingInterpolation(preScoop2.getHeading(), scoop2.getHeading())
                .build();

        sco2Sho = follower.pathBuilder()
                .addPath(new BezierLine(scoop2, shoot))
                .setLinearHeadingInterpolation(scoop2.getHeading(), shoot.getHeading())
                .build();*/

        turnTurret.setTargetPosition(0);
        turnTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnTurret.setTargetPosition(0);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Starting...");
        telemetry.update();
    }

    @Override
    public void loop() {
        long elapsedTime = System.currentTimeMillis() - startTime;
        switch (pathStage) {
            case 0: //little baby first move

                follower.followPath(startShoot);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Finished first path");
                break;

            case 1: // start turret

                if (!follower.isBusy()) {
                    turret.setVelocity(1480); //ball 1
                    angleTurret0.setPosition(0.015);
                    angleTurret1.setPosition(0.985);
                }
                if (elapsedTime >= 900) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting to shoot");
                break;

            case 2: //spin to first spot
                if (!follower.isBusy()) {
                    spindexer.setTargetPosition(95);
                }
                if (elapsedTime>=1600)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 3: //pop up shoot
                popUp.setPosition(0.51);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 4: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1645); //ball 2
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 5: //spindex to next spot
                if (!follower.isBusy()) {

                    spindexer.setTargetPosition(275);
                }
                if (elapsedTime>=600)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 6: //pop up shoot
                popUp.setPosition(0.51);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 7: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1640); //ball 3
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;


            case 8: //spindex to next spot
                if (!follower.isBusy()) {

                    spindexer.setTargetPosition(445);
                }
                if (elapsedTime>=600)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 9: //pop up shoot
                popUp.setPosition(0.51);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 10: //pop up down
                popUp.setPosition(0);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    turret.setVelocity(0);
                    //  follower.followPath(shootPre1);
                }
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 12:
                follower.followPath(shootPre1);
                pathStage++;
                startTime = System.currentTimeMillis();
                /*if (elapsedTime >= 1000) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }*/
                telemetry.addData("Status", "Starting second path");
                break;
            case 13:
                if (!follower.isBusy()) {
                    pathStage++;
                }
                break;
            case 14:

                // Start the path ONCE
                if (!preScoStarted) {
                    intake.setPower(0.4);
                    spindexer.setTargetPosition(525);
                    follower.followPath(preSco1);
                    startTime = System.currentTimeMillis();
                    preScoStarted = true;
                }

                elapsedTime = System.currentTimeMillis() - startTime;

                // ---- MECHANISM TIMING (always runs) ----
                if (elapsedTime >= 1200) {
                    intake.setPower(0);
                    spindexer.setTargetPosition(700);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                    preScoStarted = false;
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    intake.setPower(0.4);
                    spindexer.setTargetPosition(700);
                    follower.followPath(intake2);
                }
                if (elapsedTime >= 1250) {
                    intake.setPower(0);
                    spindexer.setTargetPosition(875);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    spindexer.setTargetPosition(880);
                    intake.setPower(0.4);
                    follower.followPath(intake3);
                }
                if (elapsedTime>=1950)
                {
                    turret.setVelocity(1460);
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 17:
                follower.followPath(shootPre2);
                pathStage++;
                startTime = System.currentTimeMillis();
                /*if (elapsedTime >= 1000) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }*/
                telemetry.addData("Status", "Starting second path");
                break;

            case 18: // start turret
                if (!follower.isBusy()) {
                    turret.setVelocity(1430); //ball 1
                    angleTurret0.setPosition(0.015);
                    angleTurret1.setPosition(0.985);
                }
                if (elapsedTime >= 900) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting to shoot");
                break;

            case 19: //spin to first spot
                if (!follower.isBusy()) {
                    spindexer.setTargetPosition(980);
                }
                if (elapsedTime>=1600)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 20: //pop up shoot
                popUp.setPosition(0.51);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 21: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1630); //ball 2
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 22: //spindex to next spot
                if (!follower.isBusy()) {

                    spindexer.setTargetPosition(1150);
                }
                if (elapsedTime>=600)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 23: //pop up shoot
                popUp.setPosition(0.51);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 24: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1670); //ball 3
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;


            case 25: //spindex to next spot
                if (!follower.isBusy()) {

                    spindexer.setTargetPosition(1330);
                }
                if (elapsedTime>=600)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 26: //pop up shoot
                popUp.setPosition(0.51);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 27: //pop up down
                popUp.setPosition(0);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 28:
                if (!follower.isBusy()) {
                    turret.setVelocity(0);
                    //  follower.followPath(shootPre1);
                }
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 29:
                follower.followPath(park);
                pathStage++;
                startTime = System.currentTimeMillis();
                /*if (elapsedTime >= 1000) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }*/
                telemetry.addData("Status", "move move");
                break;

            case 30: //pop up down
                popUp.setPosition(0);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;


            case 31: // All paths complete
                // Robot is stopped, do nothing
                terminateOpModeNow();
                return;
        }

        // Update the follower to move the robot
        follower.update();

        // Add telemetry for debugging
        telemetry.addData("Path Stage", pathStage);
        telemetry.addData("Current Pose", follower.getPose());
        telemetry.update();
    }




    public void setPathState (String pState){
        pathState = pState;
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

    }private void angleAdjust(double tx)
    {
        int pos=0;
        if (tx>3)
        {
            pos = pos+10;
        }

        else if (tx<-3)
        {
            pos=pos-10;
        }
        else
        {
            pos = turnTurret.getCurrentPosition();
        }
        turnTurret.setTargetPosition(pos);
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
            angleTurret0.setPosition(0.09);
            angleTurret1.setPosition(0.91);
        }
        else if (dist<=0.5){
            angleTurret0.setPosition(0.12);
            angleTurret1.setPosition(0.88);
        }
        else {
            angleTurret0.setPosition(0.08);
            angleTurret1.setPosition(0.92);
        }
    }
    private void setTurretVelocity(double dist)
    {
        //double velocity = (-58.21*(dist*dist)) + (550.8*dist) + 820; OLD EQUATION
        double velocity = 271*(dist) + 1050;
        if (dist>2)
        {
            velocity = velocity - 150;
        }
        else if (dist>1.5)
        {
            velocity = velocity + 20;
        }
        v = velocity;
        turret.setVelocity((int)Math.round(velocity));
        telemetry.addData("velocity", (int)Math.round(velocity));
        telemetry.update();
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
    private boolean isSpotTaken() {
        // Get raw color values
        int red = colorFront.red();
        int green = colorFront.green();
        int blue = colorFront.blue();
        NormalizedRGBA colors = colorFront.getNormalizedColors();

        if ((colors.blue)> colors.green && colors.blue>0.00135)
        {
            telemetry.addData("Color seen:", "purple");
            telemetry.addData("Color seen:", colors.blue);
            telemetry.update();
            return true;

        } else if(colors.green>(colors.blue) && colors.green>0.00135) {

            telemetry.addData("Color seen:", "green");
            telemetry.addData("Color seen:", colors.green);
            telemetry.update();
            return true;
        }
        telemetry.addData("Color seen:", "No Color");
        telemetry.update();
        return false;

    }
    private boolean greenDetect()
    {
        NormalizedRGBA colors = colorBack.getNormalizedColors();
        if(colors.green>(colors.blue) && colors.green>0.0013) {

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

        if ((colors.blue)> colors.green && colors.blue>0.0013)
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
    private void limelightWorky()
    {
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
        else {
            telemetry.addData("Limelight: ", "Not seen");
        }
    }

}
