#include <Windows.h>
#include <fstream>
#include <iostream>
//#include <winsock.h>
#include <string.h>
#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/synch/CThreadSafeVariable.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <SFML/Network.hpp>
//#pragma comment(lib,"wsock32.lib")
#pragma comment(lib,"sfml-network-d.lib")
#define PROM 8.183001

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace std;

// Thread for grabbing: Do this is another thread so we divide rendering and grabbing
//   and exploit multicore CPUs.
struct TThreadParam
{
	TThreadParam() : quit(false), pushed_key(0), Hz(0) { }

	volatile bool   quit;
	volatile int    pushed_key;
	volatile double Hz;

	mrpt::synch::CThreadSafeVariable<CObservation3DRangeScanPtr> new_obs;     // RGB+D (+3D points)
	mrpt::synch::CThreadSafeVariable<CObservationIMUPtr>         new_obs_imu; // Accelerometers
};


CObservation2DRangeScanPtr obs_2d;
//SOCKET sock;
//SOCKADDR_IN server_addr;
sf::UdpSocket socket1;
std::size_t received;
sf::IpAddress sender;
unsigned short senderPort;

void thread_grabbing(TThreadParam &p)
{
	try
	{
		CKinect  kinect;

		// Set params:
		// kinect.enableGrab3DPoints(true);
		// kinect.enablePreviewRGB(true);
		//...
		const std::string cfgFile = "kinect_calib.cfg";
		if (mrpt::system::fileExists(cfgFile))
		{
			cout << "Loading calibration from: " << cfgFile << endl;
			kinect.loadConfig(mrpt::utils::CConfigFile(cfgFile), "KINECT");
		}
		else cerr << "Warning: Calibration file [" << cfgFile << "] not found -> Using default params.\n";

		// Open:
		cout << "Calling CKinect::initialize()...";
		kinect.initialize();
		cout << "OK\n";

		CTicTac tictac;
		int nImgs = 0;
		bool there_is_obs = true, hard_error = false;

		while (!hard_error && !p.quit)
		{
			// Grab new observation from the camera:
			CObservation3DRangeScanPtr  obs = CObservation3DRangeScan::Create(); // Smart pointers to observations
			CObservationIMUPtr          obs_imu = CObservationIMU::Create();

			kinect.getNextObservation(*obs, *obs_imu, there_is_obs, hard_error);

			if (!hard_error && there_is_obs)
			{
				p.new_obs.set(obs);
				p.new_obs_imu.set(obs_imu);
			}

			if (p.pushed_key != 0)
			{
				switch (p.pushed_key)
				{
				case 27:
					p.quit = true;
					break;
				}

				// Clear pushed key flag:
				p.pushed_key = 0;
			}

			nImgs++;
			if (nImgs>10)
			{
				p.Hz = nImgs / tictac.Tac();
				nImgs = 0;
				tictac.Tic();
			}
		}
	}
	catch (std::exception &e)
	{
		cout << "Exception in Kinect thread: " << e.what() << endl;
		p.quit = true;
	}
}
//unsigned char s_data[1024];
void SEND_FROM_KINECT()
{
	
	//ZeroMemory(s_data, 1024);
	int data = sizeof(SOCKADDR);
	int k = 1;
	string s;

	for (int i = 0; i < obs_2d->scan.size(); i++)
	{

		if ((*obs_2d).scan.at(i) < 0.4)
			(*obs_2d).scan[i] = PROM;
		if (i % 12 == 0)
		{
			s += to_string((*obs_2d).scan.at(i));
			s += "\n";
		}
	}
		/*
		if (i % 12 == 0)
		{
			short v = (short)(*obs_2d).scan.at(i);
			s_data[k++] = v << 2;
			s_data[k++] = v;
		}
	}
	s_data[0] = (unsigned char)k;*/
		
	if (socket1.send(s.c_str(), s.length(), sender, senderPort) != sf::Socket::Done)
		return;
	std::cout << "Message sent to the client: \"" << s << "\"" << std::endl;
}
// ------------------------------------------------------
//				Test_Kinect
// ------------------------------------------------------
void Test_Kinect()
{
	// Launch grabbing thread:
	// --------------------------------------------------------
	TThreadParam thrPar;
	mrpt::system::TThreadHandle thHandle = mrpt::system::createThreadRef(thread_grabbing, thrPar);

	// Wait until data stream starts so we can say for sure the sensor has been initialized OK:
	cout << "Waiting for sensor initialization...\n";
	do {
		CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
		if (possiblyNewObs && possiblyNewObs->timestamp != INVALID_TIMESTAMP)
			break;
		else 	mrpt::system::sleep(10);
	} while (!thrPar.quit);

	// Check error condition:
	if (thrPar.quit) return;


	// Create window and prepare OpenGL object in the scene:
	// --------------------------------------------------------
	mrpt::gui::CDisplayWindow3D  win3D("Kinect 3D -> 2D laser scan", 800, 600);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(30);
	win3D.setCameraZoom(10.0);
	win3D.setFOV(90);
	win3D.setCameraPointingToPoint(2.5, 0, 0);

	// The 3D point cloud OpenGL object:
	mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(2.5);

	// The 2D "laser scan" OpenGL object:
	mrpt::opengl::CPlanarLaserScanPtr gl_2d_scan = mrpt::opengl::CPlanarLaserScan::Create();
	gl_2d_scan->enablePoints(true);
	gl_2d_scan->enableLine(true);
	gl_2d_scan->enableSurface(true);
	gl_2d_scan->setSurfaceColor(0, 0, 1, 0.3);  // RGBA

	mrpt::opengl::CFrustumPtr gl_frustum = mrpt::opengl::CFrustum::Create(0.2f, 5.0f, 90.0f, 5.0f, 2.0f, true, true);

	const double aspect_ratio = 480.0 / 640.0; // kinect.getRowCount() / double( kinect.getColCount() );

	opengl::COpenGLViewportPtr viewRange, viewInt; // Extra viewports for the RGB & D images.
	{
		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

		//Create the Opengl object for the point cloud:
		scene->insert(gl_points);
		scene->insert(gl_2d_scan);
		scene->insert(gl_frustum);

		{
			mrpt::opengl::CGridPlaneXYPtr gl_grid = mrpt::opengl::CGridPlaneXY::Create();
			gl_grid->setColor(0.6, 0.6, 0.6);
			scene->insert(gl_grid);
		}
		{
			mrpt::opengl::CSetOfObjectsPtr gl_corner = mrpt::opengl::stock_objects::CornerXYZ();
			gl_corner->setScale(0.2);
			scene->insert(gl_corner);
		}

		const int VW_WIDTH = 250;	// Size of the viewport into the window, in pixel units.
		const int VW_HEIGHT = aspect_ratio*VW_WIDTH;
		const int VW_GAP = 30;

		// Create the Opengl objects for the planar images, as textured planes, each in a separate viewport:
		win3D.addTextMessage(30, -25 - 1 * (VW_GAP + VW_HEIGHT), "Range data", TColorf(1, 1, 1), 1, MRPT_GLUT_BITMAP_HELVETICA_12);
		viewRange = scene->createViewport("view2d_range");
		viewRange->setViewportPosition(5, -10 - 1 * (VW_GAP + VW_HEIGHT), VW_WIDTH, VW_HEIGHT);

		win3D.addTextMessage(30, -25 - 2 * (VW_GAP + VW_HEIGHT), "Intensity data", TColorf(1, 1, 1), 2, MRPT_GLUT_BITMAP_HELVETICA_12);
		viewInt = scene->createViewport("view2d_int");
		viewInt->setViewportPosition(5, -10 - 2 * (VW_GAP + VW_HEIGHT), VW_WIDTH, VW_HEIGHT);

		win3D.unlockAccess3DScene();
		win3D.repaint();
	}


	CObservation3DRangeScanPtr  last_obs;
	CObservationIMUPtr          last_obs_imu;

	while (win3D.isOpen() && !thrPar.quit)
	{
		CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
		if (possiblyNewObs && possiblyNewObs->timestamp != INVALID_TIMESTAMP &&
			(!last_obs || possiblyNewObs->timestamp != last_obs->timestamp))
		{
			// It IS a new observation:
			last_obs = possiblyNewObs;
			last_obs_imu = thrPar.new_obs_imu.get();

			// Update visualization ---------------------------------------
			bool do_refresh = false;

			// Show 2D ranges as a grayscale image:
			if (last_obs->hasRangeImage)
			{
				mrpt::utils::CImage  img;

				// Normalize the image
				static CMatrixFloat  range2D;   // Static to save time allocating the matrix in every iteration
				range2D = last_obs->rangeImage * (1.0 / 5.0); //kinect.getMaxRange());

				img.setFromMatrix(range2D);

				win3D.get3DSceneAndLock();
				viewRange->setImageView_fast(img);
				win3D.unlockAccess3DScene();
				do_refresh = true;
			}


			// Convert ranges to an equivalent 2D "fake laser" scan:
			if (last_obs->hasRangeImage)
			{
				// Convert to scan:
				obs_2d = CObservation2DRangeScan::Create();
				const float vert_FOV = DEG2RAD(gl_frustum->getVertFOV());
				//cout << "Max ugao: " << (*obs_2d).aperture << endl;
				//cout << "[" << 0 << "]:" << (*obs_2d).scan.at(0) << endl;


				ofstream outStream("data.txt");
				last_obs->convertTo2DScan(*obs_2d, "KINECT_2D_SCAN", .5f*vert_FOV, .5f*vert_FOV);
				for (int i = 0; i < obs_2d->scan.size(); i++)
				{
				//cout << "[" << i << "]:" << (*obs_2d).scan.at(i) << endl;
				//outStream << "[" << i << "]:" << (*obs_2d).scan.at(i) << endl;

				if ((*obs_2d).scan.at(i) < 0.4)
					(*obs_2d).scan[i] = PROM;

				}

				//cout << "Max ugao: " << (*obs_2d).aperture << endl;
				// And load scan in the OpenGL object:
				gl_2d_scan->enablePoints();
				gl_2d_scan->setScan(*obs_2d);
				outStream.close();
			}


			// Show intensity image:
			if (last_obs->hasIntensityImage)
			{
				win3D.get3DSceneAndLock();
				viewInt->setImageView(last_obs->intensityImage); // This is not "_fast" since the intensity image is used below in the coloured point cloud.
				win3D.unlockAccess3DScene();
				do_refresh = true;
			}

			// Show 3D points:
			if (last_obs->hasPoints3D)
			{
				win3D.get3DSceneAndLock();
				last_obs->project3DPointsFromDepthImageInto(*gl_points, false); // ignore pose_on_robot of the sensor );
				win3D.unlockAccess3DScene();
				do_refresh = true;
			}

			// Some text messages:
			win3D.get3DSceneAndLock();
			// Estimated grabbing rate:
			win3D.addTextMessage(-100, -20, format("%.02f Hz", thrPar.Hz), TColorf(1, 1, 1), "sans", 15, mrpt::opengl::FILL, 100);

			win3D.addTextMessage(10, 10, "'o'/'i'-zoom out/in, '2'/'3'/'f':show/hide 2D/3D/frustum data, mouse: orbit 3D, ESC: quit", TColorf(1, 1, 1), "sans", 10, mrpt::opengl::FILL, 110);

			win3D.addTextMessage(10, 25,
				format("Show: 3D=%s 2D=%s Frustum=%s (vert. FOV=%.1fdeg)", gl_points->isVisible() ? "YES" : "NO", gl_2d_scan->isVisible() ? "YES" : "NO", gl_frustum->isVisible() ? "YES" : "NO", gl_frustum->getVertFOV()),
				TColorf(1, 1, 1), "sans", 10, mrpt::opengl::FILL, 111);
			win3D.unlockAccess3DScene();

			// Do we have accelerometer data?
			if (last_obs_imu && last_obs_imu->dataIsPresent[IMU_X_ACC])
			{
				win3D.get3DSceneAndLock();
				win3D.addTextMessage(10, 65,
					format("Acc: x=%.02f y=%.02f z=%.02f", last_obs_imu->rawMeasurements[IMU_X_ACC], last_obs_imu->rawMeasurements[IMU_Y_ACC], last_obs_imu->rawMeasurements[IMU_Z_ACC]),
					TColorf(.7, .7, .7), "sans", 10, mrpt::opengl::FILL, 102);
				win3D.unlockAccess3DScene();
				do_refresh = true;
			}

			// Force opengl repaint:
			if (do_refresh) win3D.repaint();
		} // end update visualization:


		// Process possible keyboard commands:
		// --------------------------------------
		if (win3D.keyHit() && thrPar.pushed_key == 0)
		{
			const int key = tolower(win3D.getPushedKey());

			switch (key)
			{
				// Some of the keys are processed in this thread:
			case 'o':
				win3D.setCameraZoom(win3D.getCameraZoom() * 1.2);
				win3D.repaint();
				break;
			case 'i':
				win3D.setCameraZoom(win3D.getCameraZoom() / 1.2);
				win3D.repaint();
				break;
			case '2':
				gl_2d_scan->setVisibility(!gl_2d_scan->isVisible());
				break;
			case '3':
				gl_points->setVisibility(!gl_points->isVisible());
				break;
			case 'F':
			case 'f':
				gl_frustum->setVisibility(!gl_frustum->isVisible());
				break;
			case '+':
				gl_frustum->setVertFOV(gl_frustum->getVertFOV() + 1);
				break;
			case '-':
				gl_frustum->setVertFOV(gl_frustum->getVertFOV() - 1);
				break;
				// ...and the rest in the kinect thread:
			default:
				thrPar.pushed_key = key;
				break;
			};
		}


		//mrpt::system::sleep(1);
		char in[128];
		ZeroMemory(in, 128);
		if (socket1.receive(in, sizeof(in), received, sender, senderPort) == sf::Socket::NotReady)
		{
			cout << "Nista" << endl;
		}
		else
		{
			cout << "Receive!" << endl;
			cout << "Data:" << in << endl;
			if (strcmp(in, "kinect") == 0)
			{
				SEND_FROM_KINECT();
			}
		}
	}

	mrpt::system::sleep(1);
	cout << "Waiting for grabbing thread to exit...\n";
	thrPar.quit = true;
	mrpt::system::joinThread(thHandle);
	cout << "Bye!\n";
}






int main(int argc, char **argv)
{
	int port = 15040;
	socket1.setBlocking(false);
	if (socket1.bind(port) != sf::Socket::Done)
		return 1;
	std::cout << "Server is listening to port " << port << ", waiting for a message... " << std::endl;

	while (true)
	{
		int m;
		cout << "Unesi 1" << endl;
		cin >> m;
		if (m == 1)
		{
			try
			{
				if (m = 1)
					Test_Kinect();




				mrpt::system::sleep(50);
				//return 0;

			}
			catch (std::exception &e)
			{
				std::cout << "EXCEPCION: " << e.what() << std::endl;
				//return -1;
			}
			catch (...)
			{
				printf("Another exception!!");
				//return -1;
			}
			m = 0;
		}

		
		
	}


	return 0;

}

