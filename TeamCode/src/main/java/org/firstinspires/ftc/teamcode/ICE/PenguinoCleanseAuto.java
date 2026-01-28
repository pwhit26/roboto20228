package org.firstinspires.ftc.teamcode.ICE;
// shoot far, grab preset
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

import java.util.List;

//REAL CLOSE BLUE AUTO
//GOOD AUTO USE THIS ONE!!!!!!
//hellooooo
@Autonomous
public class PenguinoCleanseAuto extends OpMode {
    private Follower follower;
    private Pose start, shoot, shoot2, preScoop1, scoop1, preScoop2, scoop2, preScoop3, scoop3, parky, shootAgain, jiggle;
    private PathChain startShoot, shootPre1, preSco1,sco1Sho,shootPre2, park, preSco2,sco2Sho, intake2, intake3, postJiggle;
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
    long Id, IdGame;
    boolean status21, status22, status23 = false;
    int sortStep = 0;
    boolean wantGreen, wantPurple;
    int slotAlreadyChecked=0;
    int secondSlotChecked = 0;
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
        spindexer.setPower(0.45);
        colorBack = hardwareMap.get(RevColorSensorV3.class, "colorBack");
        color0 = hardwareMap.get(RevColorSensorV3.class, "color0");
        //color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        colorFront=hardwareMap.get(RevColorSensorV3.class, "colorFront");

        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize poses - adjust these values to match your field setup
        start = new Pose(0, 0, Math.toRadians(0));

        shoot = new Pose(9, 0, Math.toRadians(24.5));
        jiggle = new Pose (5, 0, Math.toRadians(24.5));
        shoot2 = new Pose(9, 0, Math.toRadians(24.5));
        preScoop1 = new Pose(24.5, 10, Math.toRadians(90));
        scoop1 = new Pose(24.5,43, Math.toRadians(90));
        scoop2 = new Pose(24.5, 45, Math.toRadians(90));
        scoop3 = new Pose(24.5, 48, Math.toRadians(90));
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
                .addPath(new BezierLine(scoop3, jiggle))
                .setLinearHeadingInterpolation(scoop3.getHeading(), jiggle.getHeading())
                .build();
        postJiggle = follower.pathBuilder()
                .addPath(new BezierLine(jiggle, shoot2))
                .setLinearHeadingInterpolation(jiggle.getHeading(), shoot2.getHeading())
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

            telemetry.addData("Status", "Initialized");

            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            if (limelight != null) {
                limelight.pipelineSwitch(1);
                limelight.start();
                telemetry.addData("Limelight", "Initialized - Pipeline: %d", 1);
            } else {
                telemetry.addData("Limelight", "Not found in hardware map!");
            }

            telemetry.update();

    }

    public void init_loop() {

        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            telemetry.addData("Limelight Result", ll != null ? "Valid" : "Null");

            if (ll != null) {
                List<LLResultTypes.FiducialResult> fiducials = ll.getFiducialResults();
                telemetry.addData("Tag Count", fiducials.size());

                for (LLResultTypes.FiducialResult fr : fiducials) {
                    long tagId = fr.getFiducialId();

                    // Ignore tag 20, only accept 21–23
                    if (tagId == 21 || tagId == 22 || tagId == 23) {
                        Id = tagId;  // store valid tag
                        telemetry.addData("Detected Tag ID", tagId);
                    }
                }
            }
        }

        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        if (limelight != null) {
            limelight.start();
        }
        startTime = System.currentTimeMillis();  // <-- reset timer HERE
        pathStage = 0;



    }

    @Override
    public void loop() {


        long elapsedTime = System.currentTimeMillis() - startTime;


        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            if (ll != null) {
                List<LLResultTypes.FiducialResult> fiducials = ll.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    long tagId = fr.getFiducialId();
                    if (tagId == 21 || tagId == 22 || tagId == 23) {
                        Id = tagId;  // keep Id current
                    }
                }
            }
        }

        switch (pathStage) {

            case 0:
                telemetry.addData("Current Id", Id);
                IdGame = Id; //this may not work but doing this to set a variable for each game to reference as the motif Id in case it gets messed up

                if (Id == 21) {
                    telemetry.addData("Motif", "Green Purple Purple");
                    status21 = true;
                    status22 = false;
                    status23 = false;
                } else if (Id == 22) {
                    telemetry.addData("Motif", "Purple Green Purple");
                    status21 = false;
                    status22 = true;
                    status23 = false;
                } else if (Id == 23) {
                    telemetry.addData("Motif", "Purple Purple Green");
                    status21 = false;
                    status22 = false;
                    status23 = true;
                } else {
                    telemetry.addData("Motif", "No valid tag yet");
                    status21 = false;
                    status22 = false;
                    status23 = false;
                   // telemetry.update();
                    break; // stay here until a valid tag
                }



                // turned this time up to see the telemetry
                if (elapsedTime >= 500) {
                    pathStage++;
                    startTime = System.currentTimeMillis(); // reset timer for next stage
                }
                break;

            case 1: //little baby first move
                turret.setVelocity(1530); //ball 1
                follower.followPath(startShoot);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Finished first path");
                break;

            case 2: // start turret

                if (!follower.isBusy()) {

                    angleTurret0.setPosition(0.005);
                    angleTurret1.setPosition(0.995);
                }
                if (elapsedTime >= 600) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting to shoot");
                break;
            case 3:
                if (!follower.isBusy())
                {
                    pathStage++;
                    sortStep=0;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 4:  // Sort first ball cleanly

                wantGreen = status21;
                wantPurple = (status22 || status23);

                // ---- Choose target based on sortStep ----
                if (sortStep == 0) {
                    spindexer.setTargetPosition(95);
                }
                else if (sortStep == 1) {
                    spindexer.setTargetPosition(275);
                }
                else if (sortStep == 2) {
                    spindexer.setTargetPosition(445);
                }

                // ---- Check if correct color found ----
                if ((wantGreen && greenDetect()) || (wantPurple && purpleDetect())) {
                    slotAlreadyChecked = sortStep;
                    pathStage++;
                    startTime = System.currentTimeMillis();
                    sortStep = 0;  // reset for next cycle
                    break;
                }

                // ---- If time passed, advance to next slot ----
                if (elapsedTime > 400) {
                    startTime = System.currentTimeMillis(); // reset timer for next slot
                    sortStep++;

                    // If we've checked all 3 slots, give up and move on
                    if (sortStep > 2) {
                        pathStage++;
                        sortStep = 0;
                        slotAlreadyChecked = 0;
                    }
                }

                break;



            case 5: //pop up shoot
                angleTurret0.setPosition(0.015);
                angleTurret1.setPosition(0.985);
                popUp.setPosition(0.47);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 6: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1605); //ball 2
                if (elapsedTime >= 500) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 7: //spindex to next spot
                wantGreen = status22;
                wantPurple = (status21 || status23);

                // ---- Choose target based on sortStep ----
                if (sortStep == 0 && slotAlreadyChecked != 0) {
                    spindexer.setTargetPosition(95);
                }
                else if (sortStep == 1 && slotAlreadyChecked != 1) {
                    spindexer.setTargetPosition(275);
                }
                else if (sortStep == 2 && slotAlreadyChecked != 2) {
                    spindexer.setTargetPosition(445);
                }

                // ---- Check if correct color found ----
                if ((wantGreen && greenDetect()) || (wantPurple && purpleDetect())) {
                    secondSlotChecked = sortStep;
                    pathStage++;
                    startTime = System.currentTimeMillis();
                    sortStep = 0;  // reset for next cycle
                    break;
                }

                // ---- If time passed, advance to next slot ----
                if (elapsedTime > 400) {
                    startTime = System.currentTimeMillis(); // reset timer for next slot
                    sortStep++;

                    // If we've checked all 3 slots, give up and move on
                    if (sortStep > 2) {
                        pathStage++;
                        sortStep = 0;
                    }
                }

                break;

            case 8: //pop up shoot
                popUp.setPosition(0.47);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 9: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1640); //ball 3
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;


            case 10: //spindex to next spot
                if (!follower.isBusy()) {
                    if (slotAlreadyChecked != 0 && secondSlotChecked != 0)
                    {
                        spindexer.setTargetPosition(95);
                    }
                    else if (slotAlreadyChecked != 1 && secondSlotChecked != 1)
                    {
                        spindexer.setTargetPosition(275);
                    }
                    else
                    {
                        spindexer.setTargetPosition(445);
                    }
                }
                if (elapsedTime>=400)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                    slotAlreadyChecked = 0;
                    secondSlotChecked = 0;
                    sortStep = 0;
                }
                break;

            case 11: //pop up shoot
                popUp.setPosition(0.47);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 12: //pop up down
                popUp.setPosition(0);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    turret.setVelocity(0);
                    //  follower.followPath(shootPre1);
                }
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 14:
                follower.followPath(shootPre1);
                spindexer.setPower(0.6);
                pathStage++;
                startTime = System.currentTimeMillis();
                /*if (elapsedTime >= 1000) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }*/
                telemetry.addData("Status", "Starting second path");
                break;
            case 15:
                if (!follower.isBusy()) {
                    pathStage++;
                }
                break;
            case 16:

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
                if (elapsedTime >= 1450) {
                    intake.setPower(0);
                    spindexer.setTargetPosition(700);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                    preScoStarted = false;
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    intake.setPower(0.4);
                    spindexer.setTargetPosition(700);
                    follower.followPath(intake2);
                }
                if (elapsedTime >= 1470) {
                    intake.setPower(0);
                    spindexer.setTargetPosition(875);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    spindexer.setTargetPosition(880);
                    intake.setPower(0.4);
                    follower.followPath(intake3);
                }
                if (elapsedTime>=1850)
                {
                    turret.setVelocity(1460);
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 19:
                follower.followPath(shootPre2);
                turret.setVelocity(1540);
                //startTime = System.currentTimeMillis();
                if (elapsedTime >= 600) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting second path");
                break;
            case 20:
                spindexer.setTargetPosition(990);
                pathStage++;
                startTime = System.currentTimeMillis();
                break;

            case 21:
                if (!follower.isBusy()) {
                    angleTurret0.setPosition(0.015);
                    angleTurret1.setPosition(0.985);
                }

                if (elapsedTime >= 900) {
                    // --- Begin physical re-home of spindexer ---
                    spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    spindexer.setPower(0.15);

                    // spin slowly until a ball is seen → this is real slot zero
                    if (greenDetect() || purpleDetect()) {
                        spindexer.setPower(0);

                        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        spindexer.setTargetPosition(0);
                        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        spindexer.setPower(0.43);

                        sortStep = 0;
                        slotAlreadyChecked = -1;
                        secondSlotChecked = -1;

                        pathStage++;
                        startTime = System.currentTimeMillis();
                    }
                }
                break;



            case 22: // second-half: find first correct ball
                elapsedTime = System.currentTimeMillis() - startTime;

                wantGreen = status21;
                wantPurple = (status22 || status23);

                if (sortStep == 0) spindexer.setTargetPosition(0);
                else if (sortStep == 1) spindexer.setTargetPosition(185);
                else if (sortStep == 2) spindexer.setTargetPosition(360);

                // Only check color once spindexer finished moving
                if (!spindexer.isBusy()) {
                    if ((wantGreen && greenDetect()) || (wantPurple && purpleDetect())) {
                        slotAlreadyChecked = sortStep;
                        sortStep = 0;
                        pathStage++;
                        startTime = System.currentTimeMillis();
                        break;
                    }
                }

                if (elapsedTime > 500) {
                    startTime = System.currentTimeMillis();
                    sortStep++;

                    if (sortStep > 2) {
                        sortStep = 0;
                        slotAlreadyChecked = -1;
                        pathStage++;
                    }
                }
                break;
            case 23:
                turnTurret.setTargetPosition(0);
                if (elapsedTime>=300)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 24: // pop up shoot
                popUp.setPosition(0.48);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;



            case 25: // pop down + spin shooter
                popUp.setPosition(0);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 26: // move from jiggle to shoot2
                follower.followPath(postJiggle);
                turret.setVelocity(1685);
                if (elapsedTime>= 400)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;



            case 27: // second slot of second-half sort
                elapsedTime = System.currentTimeMillis() - startTime;

                wantGreen = status22;
                wantPurple = (status21 || status23);

                if (sortStep == 0 && slotAlreadyChecked != 0) spindexer.setTargetPosition(0);
                else if (sortStep == 1 && slotAlreadyChecked != 1) spindexer.setTargetPosition(185);
                else if (sortStep == 2 && slotAlreadyChecked != 2) spindexer.setTargetPosition(360);

                if (!spindexer.isBusy()) {
                    if ((wantGreen && greenDetect()) || (wantPurple && purpleDetect())) {
                        secondSlotChecked = sortStep;
                        sortStep = 0;
                        pathStage++;
                        startTime = System.currentTimeMillis();
                        break;
                    }
                }

                if (elapsedTime > 500) {
                    startTime = System.currentTimeMillis();
                    sortStep++;

                    if (sortStep > 2) {
                        sortStep = 0;
                        pathStage++;
                    }
                }
                break;


            case 28: //pop up shoot
                popUp.setPosition(0.48);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 29: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1670); //ball 3
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;


            case 30: //spindex to next spot
                if (!follower.isBusy()) {
                    if (slotAlreadyChecked != 0 && secondSlotChecked != 0)
                    {
                        spindexer.setTargetPosition(0);
                    }
                    else if (slotAlreadyChecked != 1 && secondSlotChecked != 1)
                    {
                        spindexer.setTargetPosition(185);
                    }
                    else
                    {
                        spindexer.setTargetPosition(360);
                    }
                }
                if (elapsedTime>=400)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                    slotAlreadyChecked = 0;
                    secondSlotChecked = 0;
                    sortStep = 0;
                }
                break;

            case 31: //pop up shoot
                popUp.setPosition(0.47);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 32: //pop up down
                popUp.setPosition(0);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    turret.setVelocity(0);
                    //  follower.followPath(shootPre1);
                }
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 34:
                follower.followPath(park);
                pathStage++;
                startTime = System.currentTimeMillis();
                /*if (elapsedTime >= 1000) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }*/
                telemetry.addData("Status", "move move");
                break;

            case 35: //pop up down
                popUp.setPosition(0);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;


            case 36: // All paths complete
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
        NormalizedRGBA colors = color0.getNormalizedColors();
        if(colors.green>(colors.blue) && colors.green>colors.red && colors.green>0.003) {

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

        if ((colors.blue)> colors.green && colors.blue>0.002)
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
