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
    public int ArmPos = 1;


    private PathChain start2Park;

    public void buildPaths() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        start2Park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(SP.ObsZoneStart), new Point(78, 139), new Point(SP.ObsPark)))
                .setLinearHeadingInterpolation(SP.ObsZoneStart.getHeading(), SP.ObsPark.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                if (!follower.isBusy()) {
                    follower.followPath(start2Park);
                }
                break;
        }
    }

        public void setPathState (int pState){
            pathState = pState;
            pathTimer.resetTimer();
        }


        @Override
        public void init () {

            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();
            gHMap.init("Autonomous");
            gHMap.armHandler(ArmPos, 0);
            ArmPos = 0;
            while (opmodeTimer.getElapsedTimeSeconds() < 5) {
                if (opmodeTimer.getElapsedTimeSeconds() > 4) {
                    gHMap.closeClaw();
                }
            }


            //        gHMap.init("Kineses");
//        gHMap.smoothJoints(0,0.28,0.30);


            Constants.setConstants(FConstants.class, LConstants.class);
            follower = new Follower(hardwareMap);
            follower.setStartingPose(SP.NetZoneStart);
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
