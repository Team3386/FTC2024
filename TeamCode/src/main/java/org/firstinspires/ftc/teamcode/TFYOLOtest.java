package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.GlobalSubsystem;
import org.firstinspires.ftc.teamcode.utils.ncnnprocessor.DetectedObject;
import org.firstinspires.ftc.teamcode.utils.ncnnprocessor.NcnnProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "TensorFlow YOLO Object Detection")
public class TFYOLOtest extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private NcnnProcessor ncnnProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        GlobalSubsystem.getInstance().init(telemetry, hardwareMap);

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {
        ncnnProcessor = new NcnnProcessor(
                "models/nanodet_m-int8.param", "models/nanodet_m-int8.bin",
                new int[]{320, 320}, new float[]{103.53f, 116.28f, 123.675f},
                new float[]{1.f / 57.375f, 1.f / 57.12f, 1.f / 58.395f},
                0.5f,
                Runtime.getRuntime().availableProcessors()
        );

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(ncnnProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(ncnnProcessor, true);

    }   // end method initTfod()

    private void telemetryTfod() {
        for (DetectedObject object : ncnnProcessor.getDetections()) {
            telemetry.addData("detect", object);
        }
    }
}
