package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Point;

@Autonomous
public class babyAuto extends OpMode {
    //DcMotorEx frontRight, frontLeft, backRight, backLeft;
    private Pose start, first;
    private Follower follower;
    private PathChain yayMove;
    String Pathstate;

    @Override
    public void init() {
        /*frontRight=hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft=hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight=hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft=hardwareMap.get(DcMotorEx.class, "backLeft");*/
        start=new Pose(0, 0, Math.toRadians(0));
        first=new Pose(40, 0, Math.toRadians(0));
        follower= Constants.createFollower(hardwareMap);

        follower.setStartingPose(start);

    }

    @Override
    public void loop() {
        yayMove = follower.pathBuilder()
                .addPath(new BezierLine(start, first))
                .setLinearHeadingInterpolation(start.getHeading(), first.getHeading())
                .build();


        follower.update();
        autonomousPathUpdate();



    }
    @Override
    public void start(){
        Pathstate="init";

    }

    public void autonomousPathUpdate()
    {
        switch(Pathstate)
        {
            case "init":
                if(!follower.isBusy())
                {
                    follower.followPath(yayMove, true);
                }
                setPathState("final");
                break;
            case "final":
                terminateOpModeNow();
                break;

        }

    }
    public void setPathState (String pState){
        Pathstate = pState;
    }

}
