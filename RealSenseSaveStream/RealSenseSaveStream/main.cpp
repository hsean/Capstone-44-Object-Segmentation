#include "pxcsensemanager.h"
#include "pxccapture.h"
#include <stdio.h>


void RecordOrPlayback(pxcCHAR *file, bool record);

int main()
{ 
	// have to use this code to play back, so we do not really care about file format
	pxcCHAR *filename = L"save/SavedStream01";

	RecordOrPlayback(filename, true);

	RecordOrPlayback(filename, false);

	return 0;
}

// Function for record or playback from the SDK CHM help file.
// Under Basic Concepts > Raw Stream Capturing and Processing > Recording and Playing Back
// Note that pxcCHAR is a typedef for wchar_t, or wide characters for langauges that support
// more than 256 characters.
void RecordOrPlayback(pxcCHAR *file, bool record)
{
	// Create an instance of the Realsense Manager class
	PXCSenseManager *ourRealsense = PXCSenseManager::CreateInstance();
	
	// Set file as to record or playback (write vs read)
	// QueryCaptureManager returns an instance of the PXCCaptureManager
	// which is a interface for complex pipelines.
	// takes only pxcCHAR pointers.
	ourRealsense->QueryCaptureManager()->SetFileName(file, record);

	// Select the color and depth streams and turn on strong stream sync
	// strong stream sync is useful when doing 3D background segmentation (the inverse of object segmentation lol)
	// so it may be useful to gather our test streams using that format.
	ourRealsense->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480, 0, PXCCapture::Device::STREAM_OPTION_STRONG_STREAM_SYNC);
	ourRealsense->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 320, 240, 30, PXCCapture::Device::STREAM_OPTION_STRONG_STREAM_SYNC);

	// Initialize and Record 300 frames
	ourRealsense->Init();
	for (int i = 0; i<300; i++) {
		// This function blocks until a color sample is ready
		if (ourRealsense->AcquireFrame(true)<PXC_STATUS_NO_ERROR) break;

		// Retrieve the sample
		PXCCapture::Sample *sample = ourRealsense->QuerySample();

		// Work on the image sample->color
		// Is this were the object segmentation happens when opening?
		// If this is where, then this should not be a function.
		// It should be the main driver. 
		// So this is where we do our point cloud library stuff?
		// Or the openCV stuff?
		// ...


		// Prepare for the next sample
		ourRealsense->ReleaseFrame();
	}
	ourRealsense->Release();
}