package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.config.ConfigTable;

// import java.util.ArrayList;


public class Vision extends SubsystemBase {

    // [ private's ]
    // get the limelight table from the NetworkTableInstance object
    private static NetworkTable mLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    
    // a object used to write to the "priorityid" on the limelight
    private IntegerPublisher mFilterId = mLimelightTable.getIntegerTopic("priorityid").publish();
    
    private static final String FILTER_ID_INPUT_KEY = "filter april tag Id (Input): ";
    private static final String OVER_WRITE_APRILTAG_FILTER_INPUT_KEY = "Over write april tag Id (Input): ";
    
    // the fliter id of the april tag
    private int filterId = -1;

    // target data vars
    private double mTx = 0.0;
    private double mTy = 0.0;
    private double mTa = 0.0;
    private double mTl = 0.0;
    
    private int mTargetTagId = -1;
    
    private final boolean DEBUG_MODE;
    
    private double[] mData3D = new double[6]; 
    
    // [ public's ]
    // for now instead of a toml FIXME:
    public static final double STOP_MOVING_THERSHOLD = 0.55;
    public static final double TURNING_THERSHOLD = 0.08;


    // how far we want the robot to be from the target apirl tag
    public static final double DISTANCE_WANTED_FOR_ROBOT = 0.55;

    
    public Vision() {
        // reset the filters
        SmartDashboard.putNumber(FILTER_ID_INPUT_KEY, -1);
        SmartDashboard.putBoolean(OVER_WRITE_APRILTAG_FILTER_INPUT_KEY, false);
        setFilter(-1);
        // FIXME: load value from config table.
        DEBUG_MODE = true;
    }

    public void setFilter(int newFilterId) {
        // set "priorityid" Ids from limelight
        filterId = newFilterId;
        mFilterId.set(newFilterId);
    }

    public int getFilter() {
        // get "priorityid" Ids from limelight
        return filterId;
    }

    // get Tx
    public double getTx() {
        return mTx;
    }

    // get Ty
    public double getTy() {
        return mTy;
    }

    // get Tl
    public double getTl() {
        return mTl;
    }

    // get Ta
    public double getTa() {
        return mTa;
    }

    // get tag X
    public double getTagX() {
        return mData3D[0];
    }

    // get tag Y
    public double getTagY() {
        return mData3D[1];
    }

    // get tag Z
     public double getTagZ() {
        return mData3D[2];
    }

    // get tag pitch
    public double getTagPitch() {
        return mData3D[3];
    }


    // get tag yaw
    public double getTagYaw() {
        return mData3D[4];
    }

    // get tag roll
    public double getTagRoll() {
        return mData3D[5];
    } 

    // get mTargetTagId
    public int getTagId() {
        return mTargetTagId;
    }

    // returns if an april tag (with the fliter) is in veiw
    public boolean isAprilTagDetected() {
        return mTargetTagId != -1;
    }

    // NOTE: this function DOES return a null value if no april tag is found
    public VisionTagData robotDriveToAprilTag(int Id) {
        if (Id != filterId){
            setFilter(Id);
        }

        // final - initial

        if (isAprilTagDetected()) {
            return new VisionTagData(0.0 - mTx, DISTANCE_WANTED_FOR_ROBOT - getTagZ());
        }

        return null;
    }
        
    public void updateLimelight() {
        // get limelight data and put it a vars
        mTx =          mLimelightTable.getEntry("tx").getDouble(0.0);
        mTy =          mLimelightTable.getEntry("ty").getDouble(0.0);
        mTa =          mLimelightTable.getEntry("ta").getDouble(0.0);
        mTl =          mLimelightTable.getEntry("tl").getDouble(0.0);
        mTargetTagId = (int) mLimelightTable.getEntry("tid").getInteger(0);

        mData3D =      mLimelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

    } 
    
    public void updateSmartDashboard() {
        if (!DEBUG_MODE) {
            return;
        }
        // put values to the SmartDashboard
        SmartDashboard.putNumber("tx", mTx);
        SmartDashboard.putNumber("ty", mTy); 
        SmartDashboard.putNumber("ta", mTa); 
        SmartDashboard.putNumber("tl", mTl); 
        SmartDashboard.putNumber("tag id", mTargetTagId);
        SmartDashboard.putNumber("current filter id: ", getFilter());

        // detect if the toogle button was press if so then reset it and overwrite the April tag filter
        if (SmartDashboard.getBoolean(OVER_WRITE_APRILTAG_FILTER_INPUT_KEY, false)) {
            int filter = (int) SmartDashboard.getNumber(FILTER_ID_INPUT_KEY, 0);
            setFilter(filter);
            SmartDashboard.putBoolean(OVER_WRITE_APRILTAG_FILTER_INPUT_KEY, false);
            System.out.println("April tag Id overrided to: " + Integer.toString(filter));
        }

        // put the 3D position of the current April tag
        for (int i=0; i < 6; i++) {
            SmartDashboard.putNumber(String.format("%d 3D Data", i), mData3D[i]);
        }

    }


    // data for other subsystems to use this class has:
    // turningDistance (double)
    // distanceToAprilTag (double)
    public static final class VisionTagData {
        // how far we are from the target value
        public final double mTurningDistance;
        public final double mDistanceToAprilTag;
        
        public VisionTagData(double TurningDistance, double DistanceToAprilTag) {
            mTurningDistance = TurningDistance;
            mDistanceToAprilTag = DistanceToAprilTag;
        }
    }
}