package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class Camera extends SubsystemBase {
    // This is a placeholder class for the camera subsystem.
    // It currently does not have any functionality, but it can be expanded in the future.
    UsbCamera camera;

    public Camera() {
        camera = new UsbCamera("Camera", 0); // Initialize the camera with a name and device ID
        camera.setResolution(640, 480); // Set the resolution of the camera
        camera.setFPS(30); // Set the frames per second of the camera
        camera.setBrightness(50); // Set the brightness of the camera
        camera.setExposureAuto(); // Set the exposure to auto
        camera.setWhiteBalanceAuto(); // Set the white balance to auto
        // camera.setVideoMode(UsbCamera.VideoMode.kMJPEG, 640, 480, 30); // Set the video mode of the camera

        CameraServer.startAutomaticCapture(camera); // Start the camera server to stream the camera feed
        
        // Constructor for the Camera subsystem
    }

    public void initialize() {
        // Initialize the camera subsystem
    }

    public void update() {
        // Update the camera subsystem
    }

    public void captureImage() {
        // Capture an image using the camera
    }
}
