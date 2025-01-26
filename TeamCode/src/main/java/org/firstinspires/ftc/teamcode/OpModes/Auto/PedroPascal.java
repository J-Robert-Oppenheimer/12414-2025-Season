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

import org.firstinspires.ftc.teamcode.OpModes.GeneralHardwareMap;

import java.lang.annotation.Annotation;

@Config
@Autonomous (name = "PedroPascal", group = "Autonomous")
public class PedroPascal extends OpMode {
    GeneralHardwareMap gHMap = new GeneralHardwareMap(this);
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private double LowJointPos;
    private double HighJointPos;


    private PathChain start2Spec, spec2Sub, sub2Net, net2Mid, mid2Net, net2Wall, wall2Net, net2Park;

    public void buildPaths() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        start2Spec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.NetZoneStart), new Point(SP.NetSpecimen)))
                .setLinearHeadingInterpolation(SP.NetZoneStart.getHeading(), SP.NetSpecimen.getHeading())
                .build();

        spec2Sub = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.NetSpecimen), new Point(SP.SampleIntakeNetSub)))
                .setLinearHeadingInterpolation(SP.NetSpecimen.getHeading(), SP.SampleIntakeNetSub.getHeading())
                .build();

        sub2Net = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.SampleIntakeNetSub), new Point(SP.NetDiagonal)))
                .setLinearHeadingInterpolation(SP.SampleIntakeNetSub.getHeading(), SP.NetDiagonal.getHeading())
                .build();

        net2Mid = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.NetDiagonal), new Point(SP.SampleIntakeNetMid)))
                .setLinearHeadingInterpolation(SP.NetDiagonal.getHeading(), SP.SampleIntakeNetMid.getHeading())
                .build();

        mid2Net = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.SampleIntakeNetMid), new Point(SP.NetDiagonal)))
                .setLinearHeadingInterpolation(SP.SampleIntakeNetMid.getHeading(), SP.NetDiagonal.getHeading())
                .build();

        net2Wall = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.NetDiagonal), new Point(SP.SampleIntakeNetWall)))
                .setLinearHeadingInterpolation(SP.NetDiagonal.getHeading(), SP.SampleIntakeNetWall.getHeading())
                .build();

        wall2Net = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.SampleIntakeNetWall), new Point(SP.NetDiagonal)))
                .setLinearHeadingInterpolation(SP.SampleIntakeNetWall.getHeading(), SP.NetDiagonal.getHeading())
                .build();

        net2Park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(SP.NetDiagonal), new Point(83, 23), new Point(SP.NetPark)))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                if (!follower.isBusy()) {
                    follower.followPath(start2Spec);
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()){
                    follower.followPath(spec2Sub);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()){
                    follower.followPath(sub2Net);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()){
                    follower.followPath(net2Mid);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()){
                    follower.followPath(mid2Net);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()){
                    follower.followPath(net2Wall);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()){
                    follower.followPath(wall2Net);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()){
                    follower.followPath(net2Park);

                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
//        gHMap.init("Kineses");
//        gHMap.smoothJoints(0,0.28,0.30);
        LowJointPos = 0.28;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(SP.ObsZoneStart);
        buildPaths();
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

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }

//        follower.telemetryDebug(telemetryA);




}
