package org.firstinspires.ftc.teamcode;


// RR-specific imports

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.GeneralHardwareMap;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;


@Config
@Autonomous(name = "DaddyHagrid", group = "Autonomous")


public class TargetingMath extends LinearOpMode {

    public static double A = Math.tan(0.0776671517137);
    public static double h = 10.0;
    public static double n = 10.2;
    public static double b = Math.tan(0.775798852511);
    double x= 0;
    double SmoothY = 0;
    double SmoothX = 0;
    double calculatedX =0;
    double y =0;
    double calculatedY = 0;
    double theta = 0;
    double SmoothTheta = 0;
    ArrayList<Double> Xs = new ArrayList<>();
    ArrayList<Double> Ys = new ArrayList<>();
    ArrayList<Double> Thetas = new ArrayList<>();
    GeneralHardwareMap gHMap = new GeneralHardwareMap(this);

    @Override//this is where set up is done
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // instantiate your Mecanum Drive at a particular pose.

        //GeneralHardwareMap gHMap = new GeneralHardwareMap(this);
        gHMap.initLL();




        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            //Determine Number of Reds
            gHMap.limelight.pipelineSwitch(5);
            LLResult result = gHMap.limelight.getLatestResult();
            ArrayList<ArrayList<LLResultTypes.ColorResult>> twoDList = new ArrayList<>();

//            ArrayList<Double> Ys = new ArrayList<>();

//            int num = 0;
//            double error = 30;
//            boolean a;


                result = gHMap.limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        List<LLResultTypes.ColorResult> color = result.getColorResults();
                        if (!color.isEmpty()) {
                            x = color.get(0).getTargetXPixels()-300;
                            y = -1* (color.get(0).getTargetYPixels()-480);
                            theta = color.get(0).getTargetXDegrees();
                            calculatedY = (n + h/((y/480*(A+b))-b))*-1;
                            calculatedX = -1*Math.tan(Math.toRadians(theta))*(calculatedY+n);
                            //Outer Loop

                        }
                    }
                }
                Xs.add(calculatedX);
                Ys.add(calculatedY);
//                Thetas.add(theta);
                if(Ys.size() > 10) {
                    SmoothX = gHMap.averageLastContents(Xs, 8);
                    SmoothY = gHMap.averageLastContents(Ys, 8);
//                    calculatedY = (n + h / ((SmoothY / 480 * (A + b)) - b)) * -1;
//                    calculatedX = Math.tan(Math.toRadians(SmoothTheta)) * (SmoothY + n);
                }



//

                telemetry.addData("X value", x);
                telemetry.addData("Y value", y);
                telemetry.addData("X SMooth value", SmoothX);
                telemetry.addData("Y Smototh value", SmoothY);
//                telemetry.addData("X value Smoothed", gHMap.averageLastContents(Xs, 5));
                telemetry.addData("Distance", calculatedY);
                telemetry.addData("Horizontal Translation ", calculatedX);
//                telemetry.addData("Distance", n + h/((y/480*(A+b))-b));
                telemetry.addData("THETA: ", theta);


                telemetry.update();
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
//Descision Algorithm
//(Xs.get(0)-300)*(-28/600)
//                retu = drive.actionBuilder(drive.pose)
//                        .waitSeconds(0.5)
//                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y), 0)
//                        .build();
//                Actions.runBlocking(new SequentialAction(retu));


                packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            Drawing.drawRobot(new Pose(-24+ calculatedY,calculatedX), "#4CAF50");
            Drawing.drawRobot(new Pose(-24+ SmoothY,SmoothX), "#4CAF50");
            Drawing.sendPacket();

            }//While OpMode Active
        }//While Running
    //}//IF not null
}




