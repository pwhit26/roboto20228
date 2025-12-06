package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Death By Auto")
public class deathByAuto extends OpMode {
    private Follower follower;
    private Pose start, shoot, shoot2;
    private PathChain paths, side;
    String pathState="";
    long startTime = 0;
    int pathStage = 0; // 0 = not started, 1 = first path, 2 = second path, 3 = done
    public ElapsedTime runtime = new ElapsedTime();
    //hi
    Servo turnTurret, angleTurret0, angleTurret1, popUp;
    CRServo spinny;
    DcMotorEx turret, intake, transferR, transferL, frontRight, frontLeft, backRight, backLeft;
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
        transferR = hardwareMap.get(DcMotorEx.class, "transferR");
        transferL = hardwareMap.get(DcMotorEx.class, "transferL");
        turret=hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servo init
        popUp = hardwareMap.get(Servo.class, "popup");
        popUp.scaleRange(0.19, 0.24); //0.19 is up, 0.24 is down
        popUp.setPosition(0.8);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.5);
        angleTurret0.scaleRange(0.44,0.58);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.5);
        angleTurret1.scaleRange(0.42, 0.56);
        spinny = hardwareMap.get(CRServo.class, "spinny");
        turnTurret = hardwareMap.get(Servo.class, "turnTurret");
        turnTurret.scaleRange(0.21, 0.7); //hard limits, 0.7 left, 0.21 right
        turnTurret.setPosition(0.1);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize poses - adjust these values to match your field setup
        start = new Pose(0, 0, Math.toRadians(0));
        shoot = new Pose(80, 0, Math.toRadians(-42));
        shoot2 = new Pose(80, 0, Math.toRadians(0));
        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        
        // Build paths
        paths = follower.pathBuilder()
                .addPath(new BezierLine(start, shoot))
                .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
                .build();
        side = follower.pathBuilder()
                .addPath(new BezierLine(shoot, shoot2))
                .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
                .build();

        
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
            case 0: // Start first path
                follower.followPath(paths);
                if (elapsedTime >= 3500) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting first path");
                break;
                
            case 1: // First path in progress

                if (!follower.isBusy()) {
                    turret.setPower(1); //1 for low battery
                }
                if (elapsedTime >= 4000) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting second path");
                break;

            case 2:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    transferR.setPower(0.7);
                    transferL.setPower(0.7);
                    spinny.setPower(1);
                }
                if (elapsedTime >= 4500) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    popUp.setPosition(0.1);
                }
                if (elapsedTime >= 5000) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
                
            case 4: // All paths complete
                // Robot is stopped, do nothing
                return;
        }
        
        // Update the follower to move the robot
        follower.update();
        
        // Add telemetry for debugging
        telemetry.addData("Path Stage", pathStage);
        telemetry.addData("Current Pose", follower.getPose());
        telemetry.update();
    }


    /*public void autonomousPathUpdate()
    {
        switch(pathState)
        {
            case "init":
                if(!follower.isBusy())
                {
                    follower.followPath(shootOne, true);
                }
                setPathState("grabBalls");
                break;

            case "grabBalls":
                if (!follower.isBusy())
                {
                    follower.followPath(grabBalls, true);
                }
                setPathState("final");
                break;
            case "final":
                if(!follower.isBusy())
                {
                    terminateOpModeNow();
                }
                break;

        }


    }*/

    public void setPathState (String pState){
        pathState = pState;
    }
}
