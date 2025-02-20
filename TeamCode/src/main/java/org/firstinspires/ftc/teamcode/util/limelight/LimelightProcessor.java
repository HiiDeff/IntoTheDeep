package org.firstinspires.ftc.teamcode.util.limelight;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class LimelightProcessor {

    private final Limelight3A limelight;
    private final LimelightConfig LLConfig;

    public LimelightProcessor(Limelight3A limelight, LimelightConfig LLConfig) {
        this.limelight = limelight;
        this.LLConfig = LLConfig;
    }

    public Coordinates getObjectPosition(double objectCenterHeightInch) {
        LLResult result = limelight.getLatestResult();
        if(result!=null && result.isValid()){
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            LLResultTypes.ColorResult firstResult = colorResults.get(0);

            Coordinates pixelCoordinates = new Coordinates(firstResult.getTargetXPixels(), firstResult.getTargetYPixels());
            Coordinates normalizedCoordinates = pixelCoordinatesToNormalizedCoordinates(pixelCoordinates);
            return calcObjectPositionOnGround(normalizedCoordinates, objectCenterHeightInch);
        }
        else return new Coordinates(0, 0);
    }


    // Use normalized because of different resolutions
    private Coordinates pixelCoordinatesToNormalizedCoordinates(Coordinates pixelCoordinates) {
        double nx = 1.0/(LLConfig.resolutionX/2.0) * (pixelCoordinates.x - (LLConfig.resolutionX/2.0-0.5));
        double ny = 1.0/(LLConfig.resolutionY/2.0) * -(pixelCoordinates.y - (LLConfig.resolutionY/2.0-0.5));
        return new Coordinates(nx, ny);
    }

    // Basically never need to use, normalized is better
    private Coordinates pixelCoordinatesToCenteredCoordinates(Coordinates pixelCoordinates) {
        double nx = (pixelCoordinates.x - (LLConfig.resolutionX/2.0-0.5));
        double ny = -(pixelCoordinates.y - (LLConfig.resolutionY/2.0-0.5));
        return new Coordinates(nx, ny);
    }

    //receives normalized pixel coordinates as input
    private Coordinates calcObjectPositionOnGround(Coordinates pixel, double objectCenterHeightInch) {

        //Relative to the Limelight

        // y dist
        double objectAngleWithHorizontal = pixel.y * (LLConfig.verticalFOV/2.0) + LLConfig.angleWithHorizontal; //pixel.y should be negative if object is lower than center of camera fov
        double yDistInch = (LLConfig.zOffset-objectCenterHeightInch) * Math.tan(toRadians(90+objectAngleWithHorizontal));

        // x dist
        double xyPlaneProjectedDist = Math.sqrt(yDistInch*yDistInch + (LLConfig.zOffset-objectCenterHeightInch)*(LLConfig.zOffset-objectCenterHeightInch)); // projected distance on the yz plane
        double xAngleWithObject = pixel.x * (LLConfig.horizontalFOV/2.0);
        double xDistInch = xyPlaneProjectedDist * Math.tan(toRadians(xAngleWithObject));


        // account for Limelight position relative to the center of the intake
        yDistInch += LLConfig.yOffset;
        xDistInch += LLConfig.xOffset;

        Log.i("pixel x", ""+pixel.x);
        Log.i("pixel y", ""+pixel.y);
        Log.i("objectAngleWithHorizontal",""+objectAngleWithHorizontal);
        Log.i("xAngleWithObject", ""+xAngleWithObject);


        return new Coordinates(xDistInch, yDistInch);


    }

    private double toRadians(double angDeg) {
        return Math.toRadians(angDeg);
    }
}
