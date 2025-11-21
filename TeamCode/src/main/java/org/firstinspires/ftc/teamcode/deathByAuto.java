package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Death By Auto")
public class deathByAuto extends OpMode {
    private Follower follower;
    private Pose start, shoot, end, hi;
    private PathChain[] paths;
    String pathState="";
    private int pathStage = 0; // 0 = not started, 1 = first path, 2 = second path, 3 = done
    
    // Drive motors
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;

    @Override
    public void init() {
        // Initialize drive motors
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize poses - adjust these values to match your field setup
        start = new Pose(22.1, 125.1, Math.toRadians(325));
        shoot = new Pose(57.9, 86.3, Math.toRadians(180));
        end = new Pose(15, 86.3, Math.toRadians(180));
        
        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        
        // Build paths
        PathChain shootOne = follower.pathBuilder()
                .addPath(new BezierLine(start, shoot))
                .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
                .build();
                
        PathChain grabBalls = follower.pathBuilder()
                .addPath(new BezierLine(shoot, end))
                .setLinearHeadingInterpolation(shoot.getHeading(), end.getHeading())
                .build();
        
        // Store paths in an array for easier access
        paths = new PathChain[]{shootOne, grabBalls};
        
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
        switch (pathStage) {
            case 0: // Start first path
                follower.followPath(paths[0]);
                pathStage = 1;
                telemetry.addData("Status", "Starting first path");
                break;
                
            case 1: // First path in progress
                if (!follower.isBusy()) {
                    // First path complete, start second path
                    follower.followPath(paths[1]);
                    pathStage = 2;
                    telemetry.addData("Status", "Starting second path");
                }
                break;
                
            case 2: // Second path in progress
                if (!follower.isBusy()) {
                    // Both paths complete
                    pathStage = 3;
                    telemetry.addData("Status", "All paths complete");
                    // Stop the robot
                    stopRobot();
                }
                break;
                
            case 3: // All paths complete
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
    
    private void stopRobot() {
        // Stop all drive motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        
        telemetry.addData("Status", "Robot stopped");
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
