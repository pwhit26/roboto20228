package org.firstinspires.ftc.teamcode;
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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous
public class spinnySpinnyTest extends OpMode {
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

    private AnalogInput encoder;
    static final int numIntakeSlots = 3;
    int[] intakeSlotPositions = {110, 230, 355};
    int[] shootSlotPositions = {65, 190, 305};
    static final double kP = 0.0155; // Start small
    static final double kD = 0.0008; // Helps prevent overshoot
    static final double kI = 0.0;    // Usually not needed for a spindexer
    double lastError = 0;
    ElapsedTime pidTimer = new ElapsedTime();
    double PositionToleranceDeg = 10;
    static final double max_spin_power = 0.5;
    double integral = 0;
    int currentSlot = 0;
    boolean spindexerMoving = false;
    int currentShootSlot = 0;
    private VoltageSensor batteryVoltageSensor;
    double sLastError = 0;
    private int intakeConfirmCounter = 0;
    private static final int INTAKE_CONFIRM_THRESHOLD = 4;
    //hi
    Limelight3A limelight;
    RevColorSensorV3 colorBack, colorFront, color0, color1;
    Servo angleTurret0, angleTurret1, popUp;
    DcMotorEx turret, intake, spindexer, frontRight, frontLeft, backRight, backLeft, turnTurret;
    boolean xLast, bLast, lbumpLast, bPressable, xPressable, lbumpPressable, aLast, aPressable, rbumpLast, rbumpPressable, b1Last, b1Pressable, x1Last, x1Pressable;
    long sequenceStartTime = 0;
    int shootStep = 0;
    boolean needScan = true;
    String[] scanResults = {"open", "open", "open"};
    private int emptySlotCounter = 0;
    private boolean slot0Valid = false;
    private boolean slot1Valid = false;
    private boolean slot2Valid = false;
    String targetColorMode = "purple";

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

        encoder = hardwareMap.get(AnalogInput.class, "encoder");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


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
        //spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //spindexer.setPower(0.3);
        colorBack = hardwareMap.get(RevColorSensorV3.class, "colorBack");
        color0 = hardwareMap.get(RevColorSensorV3.class, "color0");
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
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
        double currentSPos = getSpindexerAngleDeg();



        /*if (limelight != null) {
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
        }*/

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
                turret.setVelocity(800); //ball 1
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
                popUp.setPosition(0.4);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 6: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(600); //ball 2
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
                popUp.setPosition(0.4);
                if (elapsedTime >= 700) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 9: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(300); //ball 3
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
                popUp.setPosition(0.4);
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
                /*
            case 14:
                //   follower.followPath(shootPre1);
                //spindexer.setPower(0.6);
                intakeAlign(2);
                if (spindexerAtTarget(intakeSlotPositions[2]))
                {

                    telemetry.addData("position", " intake slot 2 - " + getSpindexerAngleDeg());

                }
                //intake.setPower(0);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting second path");
                break;
            case 15:
                if (!follower.isBusy()) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 16:

                // Start the path
                if (!preScoStarted) {
                    intake.setPower(0.8);
                    //  follower.followPath(preSco1);

                    preScoStarted = true;
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 17:


                //elapsedTime = System.currentTimeMillis() - startTime;

                // ---- MECHANISM TIMING (always runs) ----
                if (elapsedTime >= 1200) { //1450
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 18:
                if (!intake.isBusy())
                {
                    intakeAlign(0);
                }
                if (spindexerAtTarget(intakeSlotPositions[2]))
                {

                    telemetry.addData("position", " intake slot 0 - " + getSpindexerAngleDeg());

                }
                //intake.setPower(0);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    intake.setPower(0.8);
                    //   follower.followPath(intake2);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 20:
                if (elapsedTime >= 1200) { //1470
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 21:
                if (!intake.isBusy())
                {
                    intakeAlign(1);
                }
                if (spindexerAtTarget(intakeSlotPositions[2]))
                {

                    telemetry.addData("position", " intake slot 1 - " + getSpindexerAngleDeg());

                }
                //intake.setPower(0);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    intake.setPower(0.8);
                    //    follower.followPath(intake3);
                }
                if (elapsedTime>=1000) //1850
                {
                    turret.setVelocity(800);
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

           / case 18:
                if (!spindexer.isBusy() && !intake.isBusy()) {
                    intakeAlign(0);
                    telemetry.addData("position", " intake slot 0 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 19:
                if (elapsedTime>=800)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                    preScoStarted = false;
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    intake.setPower(0.8);
                 //   follower.followPath(intake2);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 21:
                if (elapsedTime >= 1200) { //1470
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 22:
                if (!spindexer.isBusy() && !intake.isBusy())
                {
                    intakeAlign(1);
                    telemetry.addData("position", " intake slot 1 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 23:
                if (elapsedTime>=800)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }

                break;
            case 24:
                if (!follower.isBusy()) {
                    intake.setPower(0.8);
                //    follower.followPath(intake3);
                }
                if (elapsedTime>=1850) //1850
                {
                    turret.setVelocity(800);
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            //case 28:
            // break;

            case 23: //little baby first move
                turret.setVelocity(500); //ball 1
                angleTurret0.setPosition(0.005);
                angleTurret1.setPosition(0.995);
                // follower.followPath(startShoot);
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Finished first path");
                break;

            case 24: // start turret

                if (!follower.isBusy()) {
                    pathStage = 4;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting to shoot");
                break;

            case 25:

                shootAlign(0);

                if (spindexerAtTarget(shootSlotPositions[0]))
                {

                    telemetry.addData("position", "slot 0 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }




                break;



            case 26: //pop up shoot
                angleTurret0.setPosition(0.015);
                angleTurret1.setPosition(0.985);
                popUp.setPosition(0.4);
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 27: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(500); //ball 2
                if (popUp.getPosition() == 0) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 28: //spindex to next spot

                shootAlign(1);

                if (spindexerAtTarget(shootSlotPositions[1]))
                {

                    telemetry.addData("position", "slot 1 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();

                }

                break;

            case 29: //pop up shoot
                popUp.setPosition(0.4);
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 30: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(300); //ball 3
                if (popUp.getPosition() == 0) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 31:
                break;



            /*case 19:
             //   follower.followPath(shootPre2);
                turret.setVelocity(800);
                //startTime = System.currentTimeMillis();
                if (elapsedTime >= 600) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting second path");
                break;
            case 20:
                if (!follower.isBusy())
                {
                    shootAlign(1);
                    telemetry.addData("position", "slot 1 - " + getSpindexerAngleDeg());
                    pathStage=23;
                    startTime = System.currentTimeMillis();
                }
                break;


            case 23:
                turnTurret.setTargetPosition(-5);
                if (elapsedTime>=800)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 24: // pop up shoot
                popUp.setPosition(0.4);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;



            case 25: // pop down + spin shooter
                popUp.setPosition(0);
                if (elapsedTime >= 800) {
                    turnTurret.setTargetPosition(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 26: // move from jiggle to shoot2
            //    follower.followPath(postJiggle);
                turret.setVelocity(800);
                if (elapsedTime>= 800)
                {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;



            case 27: // second slot of second-half sort
                if (!follower.isBusy())
                {
                    shootAlign(2);
                    telemetry.addData("position", "slot 2 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;


            case 28: //pop up shoot
                popUp.setPosition(0.4);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 29: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(800); //ball 3
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;


            case 30: //spindex to next spot
                shootAlign(0);
                telemetry.addData("position", "slot 0 - " + getSpindexerAngleDeg());
                pathStage++;
                startTime = System.currentTimeMillis();
                break;

            case 31: //pop up shoot
                popUp.setPosition(0.4);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 32: //pop up down
                popUp.setPosition(0);
                if (elapsedTime >= 800) {
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
              //  follower.followPath(park);
                pathStage++;
                startTime = System.currentTimeMillis();
                /*if (elapsedTime >= 1000) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "move move");
                break;

            case 35: //pop up down
                popUp.setPosition(0);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;*/


            case 37: // All paths complete
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
    public void shootAlign(int currentSlot)
    {
        double targetSPos = shootSlotPositions[currentSlot];
        double error = targetSPos - getSpindexerAngleDeg();
        if (error < 0) error += 360;
        double dt = Math.max(pidTimer.seconds(), 0.02);
        pidTimer.reset();
        double voltageComp = 12.0 / Math.max(batteryVoltageSensor.getVoltage(), 1.0);
        double derivative = (error - lastError) / dt;
        double power = (error * kP) + (derivative * kD);
        lastError = error;

        power = Math.max(-0.3, Math.min(0.3, power));
        if (error > PositionToleranceDeg) {
            spindexer.setPower(power * voltageComp);
        } else {
            spindexer.setPower(0);
        }

    }

    public void intakeAlign(int slot)
    {
        double targetPos = intakeSlotPositions[slot];


        // 1. Calculate Forward Distance (to ensure it only spins one way)
        double error = targetPos - getSpindexerAngleDeg();
        if (error < 0) {
            error += 360; // Force the motor to find the target by spinning forward
        }
        double dt = Math.max(pidTimer.seconds(), 0.02);
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
        power = Math.max(-0.3, Math.min(0.3, power));
        double minPower = 0.15; // Minimum power to overcome friction

        if (error > PositionToleranceDeg) {
            double finalPower = Math.max(Math.abs(power), minPower) * Math.signum(power);
            spindexer.setPower(finalPower * voltageComp);
        } else {
            spindexer.setPower(0);
        }
    }

    boolean spindexerAtTarget(double target) {
        double error = target - getSpindexerAngleDeg();
        if (error < 0) error += 360;
        return error < PositionToleranceDeg;
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
    private void goShoot()
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

                    slot2Valid = scanResults[2].equals(targetColorMode);
                    slot0Valid = scanResults[0].equals(targetColorMode);
                    slot1Valid = scanResults[1].equals(targetColorMode);
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

}
