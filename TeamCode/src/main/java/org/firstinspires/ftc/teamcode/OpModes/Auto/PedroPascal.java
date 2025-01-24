package org.firstinspires.ftc.teamcode.OpModes.Auto;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.lang.annotation.Annotation;

@Config
@Autonomous (name = "PedroPascal", group = "Autonomous")
public class PedroPascal extends OpMode {
    private Follower follower;

    private PathChain one_zeroObs;

    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        one_zeroObs = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.ObsZoneStart), new Point(SP.Specimen5)))
                .setLinearHeadingInterpolation(SP.ObsZoneStart.getHeading(), SP.Specimen5.getHeading())

                .addPath(new BezierLine(new Point(SP.Specimen5), new Point(SP.SampleIntakeObsSub)))
                .setLinearHeadingInterpolation(SP.Specimen5.getHeading(), SP.SampleIntakeObsSub.getHeading())

                .addPath(new BezierLine(new Point(SP.SampleIntakeObsSub), new Point(SP.SampleDropSub)))
                .setLinearHeadingInterpolation(SP.SampleIntakeObsSub.getHeading(), SP.SampleDropSub.getHeading())

                .addPath(new BezierLine(new Point(SP.SampleDropSub), new Point(SP.SampleIntakeObsMid)))
                .setLinearHeadingInterpolation(SP.SampleDropSub.getHeading(), SP.SampleIntakeObsMid.getHeading())

                .addPath(new BezierLine(new Point(SP.SampleIntakeObsMid), new Point(SP.SampleDropMid)))
                .setLinearHeadingInterpolation(SP.SampleIntakeObsMid.getHeading(), SP.SampleDropMid.getHeading())

                .addPath(new BezierLine(new Point(SP.SampleDropMid), new Point(SP.SampleIntakeObsWall)))
                .setLinearHeadingInterpolation(SP.SampleDropMid.getHeading(), SP.SampleIntakeObsWall.getHeading())

                .addPath(new BezierLine(new Point(SP.SampleIntakeObsWall), new Point(SP.SampleDropWall)))
                .setLinearHeadingInterpolation(SP.SampleIntakeObsWall.getHeading(), SP.SampleDropWall.getHeading())

                .build();
        follower.followPath(one_zeroObs);
    }

//    @Override
//    public String name() {
//        return "";
//    }
//
//    @Override
//    public String group() {
//        return "";
//    }
//
//    @Override
//    public String preselectTeleOp() {
//        return "";
//    }
//
//    @Override
//    public Class<? extends Annotation> annotationType() {
//        return null;
//    }


//    public void init() {
//        follower.followPath(one_zeroObs);
//        buildPaths();
//    }

    @Override
    public void loop() {
        follower.update();
//        follower.followPath(one_zeroObs);
        if (follower.atParametricEnd()) {
            follower.followPath(one_zeroObs);
        }

//        follower.telemetryDebug(telemetryA);
    }



}
