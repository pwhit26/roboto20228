package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.4)
            .forwardZeroPowerAcceleration(-32.88194452857188)
            .lateralZeroPowerAcceleration(-61.28260921141079)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.020,0,0.00004,0.6,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.35,0.00015,0.0055,0.0095))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0,0.000007,0.06,0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(5,0,0.08,0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.3,0,0.01,0.015))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2,0.0001,0.006,0.005))
            .centripetalScaling(0.0007);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)// Flipped
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)// Flipped
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(71.26820241372417)
            .yVelocity(60.808970533956696);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.5)
            .strafePodX(-4.3)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}