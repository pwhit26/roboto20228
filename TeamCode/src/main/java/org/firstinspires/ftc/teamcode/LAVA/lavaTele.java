package org.firstinspires.ftc.teamcode.LAVA;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//RED APRILTAG LIMELIGHT
@TeleOp(group = "AAA", name ="laveTele")
public class lavaTele extends LinearOpMode {
    private Follower follower;
    public ElapsedTime runtime = new ElapsedTime();
    boolean popSequenceActive = false;
    boolean popSequenceComplete = true;
    long sequenceStartTime = 0;
    int popSequenceStep = 0;
    private final Pose startPose = new Pose(0, 0, 0);

    Servo turnTurret, angleTurret0, angleTurret1, popUp;
    CRServo spinny;
    DcMotorEx turret, intake, transferR, transferL, frontRight, frontLeft, backRight, backLeft;
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
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Follower after constants are set
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        //other motor init
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transferR = hardwareMap.get(DcMotorEx.class, "transferR");
        transferL = hardwareMap.get(DcMotorEx.class, "transferL");
        turret=hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servo init
        popUp = hardwareMap.get(Servo.class, "popup");
        popUp.setPosition(0.14);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.135);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.865);
        spinny = hardwareMap.get(CRServo.class, "spinny");

        waitForStart();
        follower.startTeleopDrive();
        runtime.reset();
        while (opModeIsActive()) {
            //BASE TELE W RED LIMELIGHT
        }
    }
}
