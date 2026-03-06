package org.firstinspires.ftc.teamcode.ICE;
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
//no motif

@Autonomous
public class SpinOnUrOwn extends OpMode {
    private Follower follower;
    private Pose start, shoot, shoot2, preScoop1, scoop1, farShoot, preScoop2, scoop2, preScoop3, scoop3, parky, shootAgain, jiggle, scoop4, scoop5, preScoop4, preScoop5;
    private PathChain startShoot, shootPre1, preSco1,sco1Sho,shootPre2, park, preSco2,sco2Sho, intake2, intake3, postJiggle, intake4, intake5, preSco3, preSco4, preSco5;
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
    static final double kP = 0.018; // Start small
    static final double kD = 0.0008; // Helps prevent overshoot
    static final double kI = 0.0;    // Usually not needed for a spindexer
    double lastError = 0;
    ElapsedTime pidTimer = new ElapsedTime();
    double PositionToleranceDeg = 4.5;
    int spindexerSettleCounter = 0;
    static final double max_spin_power = 0.5;
    double integral = 0;
    int currentSlot = 0;
    boolean spindexerMoving = false;
    int currentShootSlot = 0;
    private VoltageSensor batteryVoltageSensor;
    double sLastError = 0;
    private int intakeConfirmCounter = 0;
    private static final int INTAKE_CONFIRM_THRESHOLD = 4;
    double shootLastError = 0;
    ElapsedTime shootPidTimer = new ElapsedTime();

    double intakeLastError = 0;
    ElapsedTime intakePidTimer = new ElapsedTime();
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
        //color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        colorFront=hardwareMap.get(RevColorSensorV3.class, "colorFront");

        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize poses - adjust these values to match your field setup
        start = new Pose(-66, 12, Math.toRadians(0));
        farShoot = new Pose(-58, 12, Math.toRadians(24));
        preScoop1 = new Pose(-32, 34, Math.toRadians(90));
        scoop1 = new Pose(-32,43, Math.toRadians(90));
        scoop2 = new Pose(-29, 49, Math.toRadians(90));
        scoop3 = new Pose(-32, 68, Math.toRadians(90));
        scoop4 = new Pose(-29, 66, Math.toRadians(90));
        scoop5 = new Pose (-28, 68.5, Math.toRadians(90));
        preScoop2 = new Pose (-52, 67, Math.toRadians(90));
        preScoop3 = new Pose (-52, 65, Math.toRadians(120));
        preScoop4 = new Pose (-53, 66, Math.toRadians(130));
        preScoop5 = new Pose (-48, 64, Math.toRadians(160));




        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);

        // Build paths
        startShoot = follower.pathBuilder()
                .addPath(new BezierLine(start, farShoot))
                .setLinearHeadingInterpolation(start.getHeading(), farShoot.getHeading())
                .build();

        shootPre1 = follower.pathBuilder()
                .addPath(new BezierLine(farShoot, preScoop1))
                .setLinearHeadingInterpolation(farShoot.getHeading(), preScoop1.getHeading())
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
        intake4 = follower.pathBuilder()
                .addPath(new BezierLine(scoop3, scoop4))
                .setLinearHeadingInterpolation(scoop3.getHeading(), scoop4.getHeading())
                .build();
        intake5 = follower.pathBuilder()
                .addPath(new BezierLine(scoop4, scoop5))
                .setLinearHeadingInterpolation(scoop4.getHeading(), scoop5.getHeading())
                .build();
        sco1Sho = follower.pathBuilder()
                .addPath(new BezierLine(scoop5, farShoot))
                .setLinearHeadingInterpolation(scoop5.getHeading(), farShoot.getHeading())
                .build();
        preSco2 = follower.pathBuilder()
                .addPath(new BezierLine(farShoot, preScoop2))
                .setLinearHeadingInterpolation(farShoot.getHeading(), preScoop2.getHeading())
                .build();
        preSco3 = follower.pathBuilder()
                .addPath(new BezierLine(preScoop2, preScoop3))
                .setLinearHeadingInterpolation(preScoop2.getHeading(), preScoop3.getHeading())
                .build();
        preSco4 = follower.pathBuilder()
                .addPath(new BezierLine(preScoop3, preScoop4))
                .setLinearHeadingInterpolation(preScoop3.getHeading(), preScoop4.getHeading())
                .build();
        preSco5 = follower.pathBuilder()
                .addPath(new BezierLine(preScoop4, preScoop5))
                .setLinearHeadingInterpolation(preScoop4.getHeading(), preScoop5.getHeading())
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


                /*telemetry.addData("Current Id", Id);
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
                }*/



                // turned this time up to see the telemetry
           /* case -1:
                if (!follower.isBusy()) {   // ensures it only starts once
                    follower.followPath(startShoot);
                    pathStage = 1;
                }
                break;*/

            case 0:
                turret.setVelocity(1530); //ball 1
                if (!follower.isBusy()) {   // ensures it only starts once
                    follower.followPath(startShoot);
                    pathStage = 1;
                }
                break;

            case 1: //little baby first move
               // follower.followPath(shootPre1);


                angleTurret0.setPosition(0.005);
                angleTurret1.setPosition(0.995);
                // follower.followPath(startShoot);
                turret.setVelocity(1530); //ball 1
                if (elapsedTime >= 2000) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Finished first path");
                break;

            case 2: // start turret

                if (!follower.isBusy()) {
                    pathStage = 4;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting to shoot");
                break;

            case 4:

                shootAlign(0);

                if (spindexerAtTarget(shootSlotPositions[0]))
                {

                    telemetry.addData("position", "slot 0 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }




                break;



            case 5: //pop up shoot
                turret.setVelocity(1530); //ball 1
                if (elapsedTime>=900)
                {
                    angleTurret0.setPosition(0.015);
                    angleTurret1.setPosition(0.985);
                    popUp.setPosition(0.4);
                }
                if (elapsedTime >= 1200) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 6: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1560); //ball 2
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 7: //spindex to next spot

                shootAlign(1);

                if (spindexerAtTarget(shootSlotPositions[1]))
                {

                    telemetry.addData("position", "slot 1 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();

                }

                break;

            case 8: //pop up shoot
                popUp.setPosition(0.4);
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 9: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1560); //ball 3
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;


            case 10: //spindex to next spot

                shootAlign(2);

                if (spindexerAtTarget(shootSlotPositions[2]))
                {

                    telemetry.addData("position", "slot 2 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();

                }
                break;

            case 11: //pop up shoot
                popUp.setPosition(0.43);
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 12: //pop up down
                popUp.setPosition(0);

                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(shootPre1);
                    turret.setVelocity(800);
                }
                if (elapsedTime >= 500) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                if (!follower.isBusy()) {   // ensures it only starts once
                    follower.followPath(preSco1);
                    pathStage = 14;
                }
                break;
            case 14:
             //   follower.followPath(shootPre1);
                //spindexer.setPower(0.6);
                intakeAlign(2);
                if (spindexerAtTarget(intakeSlotPositions[2]))
                {

                    telemetry.addData("position", " intake slot 2 - " + getSpindexerAngleDeg());

                }
                //intake.setPower(0);
                if (elapsedTime >= 500) {
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
                if (!preScoStarted) { //preScoStarted is actually for intake2 i was just too lazy to change the name
                    intake.setPower(0.8);
                    follower.followPath(intake2);

                    preScoStarted = true;
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 17:


                //elapsedTime = System.currentTimeMillis() - startTime;

                // ---- MECHANISM TIMING (always runs) ----
                if (elapsedTime >= 1000) { //1450
                    intake.setPower(0.2);
                    follower.followPath(intake3);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 18:
                if (!intake.isBusy())
                {
                    intakeAlign(0);
                }
                if (spindexerAtTarget(intakeSlotPositions[0]))
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
                    follower.followPath(intake4);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 20:
                if (elapsedTime >= 1200) { //1470
                    intake.setPower(0.2);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 21:
                if (!intake.isBusy())
                {
                    intakeAlign(1);
                }
                if (spindexerAtTarget(intakeSlotPositions[1]))
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
                    follower.followPath(intake5);
                }
                if (elapsedTime>=1100) //1850
                {
                    turret.setVelocity(1500);
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;



            case 23: //little baby first move
                turret.setVelocity(1750); //ball 4
                angleTurret0.setPosition(0.005);
                angleTurret1.setPosition(0.995);
                // follower.followPath(startShoot);
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Finished first path");
                break;

            case 24:
                if (!follower.isBusy()) {   // ensures it only starts once
                    follower.followPath(sco1Sho);
                    pathStage++;
                }
                break;

            case 25:

                shootAlign(0);

                if (spindexerAtTarget(shootSlotPositions[0]))
                {

                    telemetry.addData("position", "slot 0 - " + getSpindexerAngleDeg());

                }
                if (!follower.isBusy())
                {
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
                turret.setVelocity(1630); //ball 5
                if (elapsedTime >= 300) {
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
                turret.setVelocity(1720); //ball 6
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("FINISHED HERE? 30", pathStage);

                break;
            case 31:
                shootAlign(2);
                if (spindexerAtTarget(shootSlotPositions[2]))
                {

                    telemetry.addData("position", "slot 0 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 32:
                popUp.setPosition(0.4);
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 33:
                popUp.setPosition(0);
                turret.setVelocity(800);
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("6th ball shot", pathStage);

                break;

            case 34:
                //spindexer.setPower(0.6);
                intakeAlign(2);
                if (spindexerAtTarget(intakeSlotPositions[2]))
                {

                    telemetry.addData("position", " intake slot 2 - " + getSpindexerAngleDeg());

                }
                //intake.setPower(0);
                if (elapsedTime >= 600) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting second path");
                telemetry.addData("FINISHED HERE 31", pathStage);
                break;
            case 35:
                if (!follower.isBusy()) {
                    intake.setPower(0.8);
                    follower.followPath(preSco2);
                    pathStage = 37;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 37:
                //elapsedTime = System.currentTimeMillis() - startTime;

                // ---- MECHANISM TIMING (always runs) ----
                if (elapsedTime >= 1200) { //1450
                    intake.setPower(0.2);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 38:
                intakeAlign(0);
                if (spindexerAtTarget(intakeSlotPositions[0]))
                {

                    telemetry.addData("position", " intake slot 0 - " + getSpindexerAngleDeg());

                }
                //intake.setPower(0);
                if (elapsedTime >= 600) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 39:
                if (!follower.isBusy()) {
                    intake.setPower(0.8);
                    follower.followPath(preSco3);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 40:
                if (elapsedTime >= 1200) { //1470
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 41:
                if (!intake.isBusy())
                {
                    intakeAlign(1);
                }
                if (spindexerAtTarget(intakeSlotPositions[1]))
                {

                    telemetry.addData("position", " intake slot 1 - " + getSpindexerAngleDeg());

                }
                //intake.setPower(0);
                if (!follower.isBusy()) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 42:
                if (!follower.isBusy()) {
                    intake.setPower(0.8);
                    follower.followPath(preSco4);
                }
                if (elapsedTime>=1000) //1850
                {
                    turret.setVelocity(1800);
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 43:

                shootAlign(2);

                if (spindexerAtTarget(shootSlotPositions[2]))
                {

                    telemetry.addData("position", "slot 0 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }




                break;



            case 44: //pop up shoot
                if (!follower.isBusy())
                {
                    angleTurret0.setPosition(0.015);
                    angleTurret1.setPosition(0.985);
                    popUp.setPosition(0.4);
                    if (elapsedTime >= 300) {
                        pathStage++;
                        startTime = System.currentTimeMillis();
                    }
                }
                break;

            case 45: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1700); //ball 2
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 46: //spindex to next spot

                shootAlign(0);

                if (spindexerAtTarget(shootSlotPositions[0]))
                {

                    telemetry.addData("position", "slot 1 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();

                }

                break;

            case 47: //pop up shoot
                popUp.setPosition(0.4);
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 48: //pop up down
                popUp.setPosition(0);
                turret.setVelocity(1700); //ball 3
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("FINISHED HERE? 30", pathStage);

                break;
            case 49:
                shootAlign(1);
                if (spindexerAtTarget(shootSlotPositions[1]))
                {

                    telemetry.addData("position", "slot 0 - " + getSpindexerAngleDeg());
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 50:
                popUp.setPosition(0.4);
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 51:
                popUp.setPosition(0);
                //turret.setVelocity(300); //ball 3
                if (elapsedTime >= 300) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("6th ball shot", pathStage);

                break;
            case 52:
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
                telemetry.addData("FINISHED HERE 31", pathStage);
                break;
            case 53:
                if (!follower.isBusy()) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 54:

                // Start the path
                intake.setPower(0.8);

                pathStage++;
                startTime = System.currentTimeMillis();
                break;
            case 55:


                //elapsedTime = System.currentTimeMillis() - startTime;

                // ---- MECHANISM TIMING (always runs) ----
                if (elapsedTime >= 1200) { //1450
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 56:
                if (!intake.isBusy())
                {
                    intakeAlign(0);
                }
                if (spindexerAtTarget(intakeSlotPositions[0]))
                {

                    telemetry.addData("position", " intake slot 0 - " + getSpindexerAngleDeg());

                }
                //intake.setPower(0);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 57:
                if (!follower.isBusy()) {
                    intake.setPower(0.8);
                    //   follower.followPath(intake2);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 58:
                if (elapsedTime >= 1200) { //1470
                    intake.setPower(0);
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 59:
                if (!intake.isBusy())
                {
                    intakeAlign(1);
                }
                if (spindexerAtTarget(intakeSlotPositions[1]))
                {

                    telemetry.addData("position", " intake slot 1 - " + getSpindexerAngleDeg());

                }
                //intake.setPower(0);
                if (elapsedTime >= 800) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 60:
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



            case 61: // All paths complete
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
        
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        
        double dt = Math.max(shootPidTimer.seconds(), 0.02);
        shootPidTimer.reset();
        double voltageComp = 12.0 / Math.max(batteryVoltageSensor.getVoltage(), 1.0);
        double derivative = (error - shootLastError) / dt;
        double power = (error * kP) + (derivative * kD);
        shootLastError = error;

        // Give it more max power (0.55 instead of 0.3) so it arrives snappier and brakes harder, like teleop
        power = Math.max(-0.55, Math.min(0.55, power));
        double minPower = 0.22; // Boosted minimum power to punch through internal friction so it doesn't get stuck and then snap

        if (Math.abs(error) > PositionToleranceDeg) {
            double finalPower = Math.max(Math.abs(power), minPower) * Math.signum(power);
            spindexer.setPower(finalPower * voltageComp);
            spindexerSettleCounter = 0;
        } else {
            spindexer.setPower(0);
            spindexerSettleCounter++;
        }

    }

    public void intakeAlign(int slot)
    {
        double targetPos = intakeSlotPositions[slot];


        // 1. Calculate shortest distance to prevent spinning forever on overshoot
        double error = targetPos - getSpindexerAngleDeg();
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        double dt = Math.max(intakePidTimer.seconds(), 0.02);
        intakePidTimer.reset();

        // 1. Get current voltage
        double currentVoltage = batteryVoltageSensor.getVoltage();

// 2. Calculate the compensation factor (Target 12V / Current)
// We use Math.max to prevent division by zero just in case
        double voltageComp = 12.0 / Math.max(currentVoltage, 1.0);

        double derivative = (error - intakeLastError) / dt;
        double power = (error * kP) + (derivative * kD);
        intakeLastError = error;

        // Cap the power so it doesn't go crazy, but allow enough to be snappy
        power = Math.max(-0.55, Math.min(0.55, power));
        double minPower = 0.22; // Boosted minimum power to punch through internal friction so it doesn't get stuck and then snap

        if (Math.abs(error) > PositionToleranceDeg) {
            double finalPower = Math.max(Math.abs(power), minPower) * Math.signum(power);
            spindexer.setPower(finalPower * voltageComp);
            spindexerSettleCounter = 0;
        } else {
            spindexer.setPower(0);
            spindexerSettleCounter++;
        }
    }

    boolean spindexerAtTarget(double target) {
        return spindexerSettleCounter >= 3;
    }

}
