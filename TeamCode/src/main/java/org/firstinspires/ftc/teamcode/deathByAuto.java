package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    
    // Drive motors
    private DcMotorEx frontRight, frontLeft, backRight, backLeft, turret;
    private Servo angleTurret0, angleTurret1, popup;

    @Override
    public void init() {
        // Initialize drive motors
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.04);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.95);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        popup=hardwareMap.get(Servo.class, "popup");
        popup.setPosition(0.14);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize poses - adjust these values to match your field setup
        start = new Pose(0, 0, Math.toRadians(0));
        shoot = new Pose(20, 0, Math.toRadians(0));
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
                    follower.turnDegrees(22.5, false);
                }
                if (elapsedTime >= 4500) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting second path");
                break;

            case 2:
                if (!follower.isBusy()) {
                    frontRight.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    backLeft.setPower(0);
                    turret.setPower(1);
                }
                if (elapsedTime >= 5500) {
                    pathStage++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 3: // Second path in progress
                if (!follower.isBusy()) {
                    // Both paths complete
                    frontRight.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    backLeft.setPower(0);
                    popup.setPosition(0.105);
                    telemetry.addData("Status", "All paths complete");
                }
                if (elapsedTime >= 6000) {
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
