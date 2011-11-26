#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>
#include "oscpack/ip/UdpSocket.h"
#include "oscpack/osc/OscOutboundPacketStream.h"
#include <pcl/filters/passthrough.h>


char *ADDRESS = "127.0.0.1";
int PORT = 7110;

#define OUTPUT_BUFFER_SIZE 1024
char osc_buffer[OUTPUT_BUFFER_SIZE];
UdpTransmitSocket *transmitSocket;

char tmp[50];
float jointCoords[4]; //Fix for MacOSX's crazy gcc (was overwriting userGenerator with the last coord apparently...)

//Multipliers for coordinate system. This is useful if you use software like animata,
//that needs OSC messages to use an arbitrary coordinate system.
double mult_x = 1;
double mult_y = 1;
double mult_z = 1;

//Offsets for coordinate system. This is useful if you use software like animata,
//that needs OSC messages to use an arbitrary coordinate system.
double off_x = 0.0;
double off_y = 0.0;
double off_z = 0.0;

bool kitchenMode = false;
bool mirrorMode = false;
int nDimensions = 3;

xn::UserGenerator userGenerator;
XnChar g_strPose[20] = "";

void checkRetVal(XnStatus nRetVal) {
	if (nRetVal != XN_STATUS_OK) {
		printf("There was a problem initializing kinect... Make sure you have\
connected both usb and power cables and that the driver and OpenNI framework\
are correctly installed.\n\n");
		exit(1);
	}
}


// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	printf("New User %d\n", nId);
	userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);

	osc::OutboundPacketStream p( osc_buffer, OUTPUT_BUFFER_SIZE );
	p << osc::BeginBundleImmediate;
	p << osc::BeginMessage( "/new_user" );
	p << (int)nId;
	p << osc::EndMessage;
	p << osc::EndBundle;
	transmitSocket->Send(p.Data(), p.Size());
}



// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	printf("Lost user %d\n", nId);

	if (kitchenMode) return;

	osc::OutboundPacketStream p( osc_buffer, OUTPUT_BUFFER_SIZE );
	p << osc::BeginBundleImmediate;
	p << osc::BeginMessage( "/lost_user" );
	p << (int)nId;
	p << osc::EndMessage;
	p << osc::EndBundle;
	transmitSocket->Send(p.Data(), p.Size());
}

// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie) {
	printf("Pose %s detected for user %d\n", strPose, nId);
	userGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}



// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	printf("Calibration started for user %d\n", nId);
}



// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		printf("Calibration complete, start tracking user %d\n", nId);
		userGenerator.GetSkeletonCap().StartTracking(nId);

		if (kitchenMode) return;

		osc::OutboundPacketStream p( osc_buffer, OUTPUT_BUFFER_SIZE );
		p << osc::BeginBundleImmediate;
		p << osc::BeginMessage( "/new_skel" );
		p << (int)nId;
		p << osc::EndMessage;
		p << osc::EndBundle;
		transmitSocket->Send(p.Data(), p.Size());
	}
	else {
		printf("Calibration failed for user %d\n", nId);
		userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}



int jointPos(XnUserID player, XnSkeletonJoint eJoint) {
	XnSkeletonJointPosition joint;
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

	if (joint.fConfidence < 0.5)
		return -1;

	jointCoords[0] = player;
	jointCoords[1] = off_x + (mult_x * (1280 - joint.position.X) / 2560); //Normalize coords to 0..1 interval
	jointCoords[2] = off_y + (mult_y * (1280 - joint.position.Y) / 2560); //Normalize coords to 0..1 interval
	jointCoords[3] = off_z + (mult_z * joint.position.Z * 7.8125 / 10000); //Normalize coords to 0..7.8125 interval
//uncomment if you want native coordinates.	
//	jointCoords[1] = joint.position.X;
//	jointCoords[2] = joint.position.Y;
//	jointCoords[3] = joint.position.Z;

	return 0;
}



void genOscMsg(osc::OutboundPacketStream *p, char *name) {
	*p << osc::BeginMessage( "/joint" );
	*p << name;
	if (!kitchenMode)
		*p << (int)jointCoords[0];
	for (int i = 1; i < nDimensions+1; i++)
		*p << jointCoords[i];
	*p << osc::EndMessage;
}



void sendOSC(const xn::DepthMetaData& dmd)
{
	XnUserID aUsers[2];
	XnUInt16 nUsers = 2;
	userGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		if (userGenerator.GetSkeletonCap().IsTracking(aUsers[i])) {
			osc::OutboundPacketStream p( osc_buffer, OUTPUT_BUFFER_SIZE );
			p << osc::BeginBundleImmediate;

			if (jointPos(aUsers[i], XN_SKEL_HEAD) == 0) {
				genOscMsg(&p, "head");
			}
			if (jointPos(aUsers[i], XN_SKEL_NECK) == 0) {
				genOscMsg(&p, "neck");
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_COLLAR) == 0) {
				genOscMsg(&p, "l_collar");
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_SHOULDER) == 0) {
				genOscMsg(&p, "l_shoulder");
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_ELBOW) == 0) {
				genOscMsg(&p, "l_elbow");
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_WRIST) == 0) {
				genOscMsg(&p, "l_wrist");
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_HAND) == 0) {
				genOscMsg(&p, "l_hand");
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_FINGERTIP) == 0) {
				genOscMsg(&p, "l_fingertip");
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_COLLAR) == 0) {
				genOscMsg(&p, "r_collar");
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_SHOULDER) == 0) {
				genOscMsg(&p, "r_shoulder");
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_ELBOW) == 0) {
				genOscMsg(&p, "r_elbow");
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_WRIST) == 0) {
				genOscMsg(&p, "r_wrist");
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_HAND) == 0) {
				genOscMsg(&p, "r_hand");
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_FINGERTIP) == 0) {
				genOscMsg(&p, "r_fingertip");
			}
			if (jointPos(aUsers[i], XN_SKEL_TORSO) == 0) {
				genOscMsg(&p, "torso");
			}
			if (jointPos(aUsers[i], XN_SKEL_WAIST) == 0) {
				genOscMsg(&p, "waist");
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_HIP) == 0) {
				genOscMsg(&p, "l_hip");
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_KNEE) == 0) {
				genOscMsg(&p, "l_knee");
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_ANKLE) == 0) {
				genOscMsg(&p, "l_ankle");
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_FOOT) == 0) {
				genOscMsg(&p, "l_foot");
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_HIP) == 0) {
				genOscMsg(&p, "r_hip");
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_KNEE) == 0) {
				genOscMsg(&p, "r_knee");
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_ANKLE) == 0) {
				genOscMsg(&p, "r_ankle");
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_FOOT) == 0) {
				genOscMsg(&p, "r_foot");
			}

			p << osc::EndBundle;
		    transmitSocket->Send(p.Data(), p.Size());
		}
	}
}


void viewerAnnotation (pcl::visualization::PCLVisualizer& viewer){
//	viewer.addCoordinateSystem (0.1);    

//text annotation of joints? 
//    static unsigned count = 0;
//    std::stringstream ss;
//    ss << "Hand?: " << count++;
//    viewer.removeShape ("text", 0);  
//		if (joint.fConfidence > 0.5){
//		    viewer.addText (ss.str(), joint.position.X, joint.position.Y, "text", 0);
//			viewer.addText3D (ss.str(), pclPoint, 2.0, 0.0, 255.0, 0.0, "text", 0);	
//		}
	
	
//	pcl::PointXYZ p1, p2;
//	p1.x=0.0; p1.y=0.0; p1.z=0.0;
//	p2.x=1.0; p2.y=1.0; p2.z=1.0;
//	viewer.addCoordinateSystem(0.025);
//	viewer.addLine(p1, p2, 0.0, 255.0, 0.0,"line1", 0); 

	viewer.removeShape ("headneck", 0);  
	viewer.removeShape ("necktorso", 0);  
	viewer.removeShape ("torsowaist", 0);  	
	viewer.removeShape ("neckrshoulder", 0);  		
	viewer.removeShape ("rshoulderrelbow", 0);  		
	viewer.removeShape ("relbowrhand", 0);  		
	viewer.removeShape ("rhandrfinger", 0);  			
	viewer.removeShape ("necklshoulder", 0);  		
	viewer.removeShape ("lshoulderlelbow", 0);  		
	viewer.removeShape ("lelbowlhand", 0);  			
	
	//add the skeleton outline	
	if (userGenerator.GetSkeletonCap().IsTracking(1)) {    
		XnSkeletonJointPosition joint;

		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_HEAD, joint);		
		pcl::PointXYZ pclHeadPoint;
		pclHeadPoint.x=joint.position.X/1000;
		pclHeadPoint.y=-1*joint.position.Y/1000;
		pclHeadPoint.z=joint.position.Z/1000;
		
		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_NECK, joint);
		pcl::PointXYZ pclNeckPoint;
		pclNeckPoint.x=joint.position.X/1000;
		pclNeckPoint.y=-1*joint.position.Y/1000;
		pclNeckPoint.z=joint.position.Z/1000;				

		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_WAIST, joint);
		pcl::PointXYZ pclWaistPoint;
		pclWaistPoint.x=joint.position.X/1000;
		pclWaistPoint.y=-1*joint.position.Y/1000;
		pclWaistPoint.z=joint.position.Z/1000;	
		
		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_TORSO, joint);
		pcl::PointXYZ pclTorsoPoint;
		pclTorsoPoint.x=joint.position.X/1000;
		pclTorsoPoint.y=-1*joint.position.Y/1000;
		pclTorsoPoint.z=joint.position.Z/1000;				

		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_SHOULDER, joint);		
		pcl::PointXYZ pclLShoulderPoint;
		pclLShoulderPoint.x=joint.position.X/1000;
		pclLShoulderPoint.y=-1*joint.position.Y/1000;
		pclLShoulderPoint.z=joint.position.Z/1000;				
		
		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_SHOULDER, joint);		
		pcl::PointXYZ pclRShoulderPoint;
		pclRShoulderPoint.x=joint.position.X/1000;
		pclRShoulderPoint.y=-1*joint.position.Y/1000;
		pclRShoulderPoint.z=joint.position.Z/1000;				

		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_ELBOW, joint);
		pcl::PointXYZ pclRElbowPoint;
		pclRElbowPoint.x=joint.position.X/1000;
		pclRElbowPoint.y=-1*joint.position.Y/1000;
		pclRElbowPoint.z=joint.position.Z/1000;
		
		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_HAND, joint);
		pcl::PointXYZ pclRHandPoint;
		pclRHandPoint.x=joint.position.X/1000;
		pclRHandPoint.y=-1*joint.position.Y/1000;
		pclRHandPoint.z=joint.position.Z/1000 ;

		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_FINGERTIP, joint);
		pcl::PointXYZ pclRFingerPoint;
		pclRFingerPoint.x=joint.position.X/1000;
		pclRFingerPoint.y=-1*joint.position.Y/1000;
		pclRFingerPoint.z=joint.position.Z/1000 ;

		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_ELBOW, joint);
		pcl::PointXYZ pclLElbowPoint;
		pclLElbowPoint.x=joint.position.X/1000;
		pclLElbowPoint.y=-1*joint.position.Y/1000;
		pclLElbowPoint.z=joint.position.Z/1000;
		
		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_HAND, joint);
		pcl::PointXYZ pclLHandPoint;
		pclLHandPoint.x=joint.position.X/1000;
		pclLHandPoint.y=-1*joint.position.Y/1000;
		pclLHandPoint.z=joint.position.Z/1000 ;
//		cout << "ZPos: "  << pclHandPoint.z << endl;q



		viewer.addLine(pclHeadPoint, pclNeckPoint, 0.0, 255.0, 0.0,"headneck", 0); 
		viewer.addLine(pclNeckPoint, pclTorsoPoint, 0.0, 255.0, 0.0,"necktorso", 0); 
		viewer.addLine(pclTorsoPoint, pclWaistPoint, 0.0, 255.0, 0.0,"torsowaist", 0); 
		viewer.addLine(pclNeckPoint, pclRShoulderPoint, 0.0, 255.0, 0.0,"neckrshoulder", 0); 
		viewer.addLine(pclRShoulderPoint, pclRElbowPoint, 0.0, 255.0, 0.0,"rshoulderrelbow", 0); 
		viewer.addLine(pclRElbowPoint, pclRHandPoint, 0.0, 255.0, 0.0,"relbowrhand", 0); 
		viewer.addLine(pclRHandPoint,pclRFingerPoint, 0.0, 255.0, 0.0,"rhandrfinger", 0); 
		viewer.addLine(pclNeckPoint, pclLShoulderPoint, 0.0, 255.0, 0.0,"necklshoulder", 0); 
		viewer.addLine(pclLShoulderPoint, pclLElbowPoint, 0.0, 255.0, 0.0,"lshoulderlelbow", 0); 
		viewer.addLine(pclLElbowPoint, pclLHandPoint, 0.0, 255.0, 0.0,"lelbowlhand", 0); 
		

	}
	
}		

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}


     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);     
        pcl::PassThrough<pcl::PointXYZ> pass;	
    	if (!viewer.wasStopped()){
			pass.setInputCloud (cloud);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0.0, 3.0);
			//pass.setFilterLimitsNegative (true);
			pass.filter (*cloud_filtered);
		}

        viewer.showCloud (cloud_filtered);
		viewer.runOnVisualizationThread (viewerAnnotation);
     }

     void run ()
     {
		pcl::Grabber* interface = new pcl::OpenNIGrabber();
		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
		interface->registerCallback (f);
		interface->start ();
		//openni  code.       
		xn::Context context;
		xn::DepthGenerator depth;
		context.Init();
		depth.Create(context);
		XnMapOutputMode mapMode;
//uncomment on slower boxen for lower res		
//		mapMode.nXRes = XN_VGA_X_RES;
//		mapMode.nYRes = XN_VGA_Y_RES;
		mapMode.nXRes = XN_SVGA_X_RES;
		mapMode.nYRes = XN_SVGA_Y_RES;		
		mapMode.nFPS = 30;
		depth.SetMapOutputMode(mapMode);
		XnStatus nRetVal = XN_STATUS_OK;
		nRetVal = context.FindExistingNode(XN_NODE_TYPE_USER, userGenerator);
		if (nRetVal != XN_STATUS_OK)
			nRetVal = userGenerator.Create(context);		

		XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
		checkRetVal(userGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks));
		checkRetVal(userGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks));
		checkRetVal(userGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks));
		checkRetVal(userGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose));
		checkRetVal(userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL));	

		context.StartGeneratingAll();
		transmitSocket = new UdpTransmitSocket(IpEndpointName(ADDRESS, PORT));
		cout << "openni/osceleton portion done..pcl portion starting\n" ;
       while (!viewer.wasStopped())
       {
		// Read next available data
		context.WaitAnyUpdateAll();
		// Process the data
		xn::DepthMetaData depthMD;
		depth.GetMetaData(depthMD);
		sendOSC(depthMD);		 
       }

       interface->stop ();
       context.Shutdown();
     }

	pcl::visualization::CloudViewer viewer;
	//pcl::visualization::PCLVisualizer viewer;
	//viewer.addCoordinateSystem(1.0,0);	 
 };

 int main ()
 {

 	SimpleOpenNIViewer v;
	v.run ();
	return 0;
 }

