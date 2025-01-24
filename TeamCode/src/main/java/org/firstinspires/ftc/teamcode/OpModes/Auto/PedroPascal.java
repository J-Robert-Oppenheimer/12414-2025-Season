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
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;


    private PathChain obs2Spec5, spec52SubIntake, subIntake2SubDrop, subDrop2MidIntake, midIntake2MidDrop, midDrop2WallIntake, wallIntake2WallDrop, wallDrop2Park;

    public void buildPaths() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        obs2Spec5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.ObsZoneStart), new Point(SP.Specimen5)))
                .setLinearHeadingInterpolation(SP.ObsZoneStart.getHeading(), SP.Specimen5.getHeading())
                .build();

        spec52SubIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.Specimen5), new Point(SP.SampleIntakeObsSub)))
                .setLinearHeadingInterpolation(SP.Specimen5.getHeading(), SP.SampleIntakeObsSub.getHeading())
                .build();

        subIntake2SubDrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.SampleIntakeObsSub), new Point(SP.SampleDropSub)))
                .setLinearHeadingInterpolation(SP.SampleIntakeObsSub.getHeading(), SP.SampleDropSub.getHeading())
                .build();

        subDrop2MidIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.SampleDropSub), new Point(SP.SampleIntakeObsMid)))
                .setLinearHeadingInterpolation(SP.SampleDropSub.getHeading(), SP.SampleIntakeObsMid.getHeading())
                .build();

        midIntake2MidDrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.SampleIntakeObsMid), new Point(SP.SampleDropMid)))
                .setLinearHeadingInterpolation(SP.SampleIntakeObsMid.getHeading(), SP.SampleDropMid.getHeading())
                .build();

        midDrop2WallIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.SampleDropMid), new Point(SP.SampleIntakeObsWall)))
                .setLinearHeadingInterpolation(SP.SampleDropMid.getHeading(), SP.SampleIntakeObsWall.getHeading())
                .build();

        wallIntake2WallDrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.SampleIntakeObsWall), new Point(SP.SampleDropWall)))
                .setLinearHeadingInterpolation(SP.SampleIntakeObsWall.getHeading(), SP.SampleDropWall.getHeading())
                .build();

        wallDrop2Park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(SP.SampleDropWall), new Point(65, 137), new Point(SP.ObsPark)))
                .setLinearHeadingInterpolation(SP.SampleDropWall.getHeading(), SP.ObsPark.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                if (!follower.isBusy()) {
                    follower.followPath(obs2Spec5);
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()){
                    follower.followPath(spec52SubIntake);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()){
                    follower.followPath(subIntake2SubDrop);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()){
                    follower.followPath(subDrop2MidIntake);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()){
                    follower.followPath(midIntake2MidDrop);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()){
                    follower.followPath(midDrop2WallIntake);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()){
                    follower.followPath(wallIntake2WallDrop);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()){
                    follower.followPath(wallDrop2Park);
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
