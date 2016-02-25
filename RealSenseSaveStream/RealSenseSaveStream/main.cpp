#include "pxcsensemanager.h"
#include "pxccapture.h"
#include <stdio.h>
#include "util_render.h"
#include <conio.h>

void RecordOrPlayback(pxcCHAR *file, bool record);
void PlaybackRecording(pxcCHAR *file);

int main()
{ 
	// have to use this code to play back, so we do not really care about file format
	pxcCHAR *filename = L"SavedStream04";

	//RecordOrPlayback(filename, true);

	PlaybackRecording(filename);

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
	PXCCaptureManager *capManager = ourRealsense->QueryCaptureManager();
	capManager->SetFileName(file, record);

	// Select the color and depth streams and turn on strong stream sync
	// strong stream sync is useful when doing 3D background segmentation (the inverse of object segmentation lol)
	// so it may be useful to gather our test streams using that format.
	//ourRealsense->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480, 0, PXCCapture::Device::STREAM_OPTION_STRONG_STREAM_SYNC);
	ourRealsense->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 320, 240, 30, PXCCapture::Device::STREAM_OPTION_STRONG_STREAM_SYNC);
	// Initialize
	ourRealsense->Init();

	// Playback settings
	if (!record)
	{
		// set pause mode, that is, return the same sample of the current
		// frame repeatedly.
		capManager->SetRealtime(false);
		capManager->SetPause(true);
	}

	// Create stream renders
	//UtilRender renderc(L"Color"), renderd(L"Depth"), renderi(L"IR");
	UtilRender renderDepth(L"Depth");

	// process 300 frames
	for (int i = 0; i<300; i++) {

		// playback mode stuff
		if (!record)
		{
			capManager->SetFrameByIndex(i);		//set to current frame
			ourRealsense->FlushFrame();			//used in playback to empty cache
		}
		
		// This function blocks until a color is ready
		pxcStatus frameStatus = ourRealsense->AcquireFrame(true);
		if (frameStatus<PXC_STATUS_NO_ERROR) break;

		// Retrieve the sample
		PXCCapture::Sample *sample = ourRealsense->QuerySample();

		// Work on the image sample->color
		// render the frame or exit if not able to render
		// works for both recording and playback
		if (sample->depth && !renderDepth.RenderFrame(sample->depth)) break;

		// Prepare for the next sample
		ourRealsense->ReleaseFrame();
		
		// playback recording frame by frame 
		if (!record)
		{
			// wait for keyboard hit to go to next frame
			while(!_kbhit()) {
			}
			int c = _getch();
			if (c == 27 || c == 'q' || c == 'Q') break; // ESC|q|Q for Exit
		}
		
	}
	ourRealsense->Release();
}

void PlaybackRecording(pxcCHAR *file)
{
	// Create an instance of the Realsense Manager class
	PXCSenseManager *ourRealsense = PXCSenseManager::CreateInstance();

	// Set file as to record or playback (write vs read)
	// QueryCaptureManager returns an instance of the PXCCaptureManager
	// which is a interface for complex pipelines.
	// takes only pxcCHAR pointers.
	PXCCaptureManager *capManager = ourRealsense->QueryCaptureManager();
	capManager->SetFileName(file, false);

	// Playback settings
	ourRealsense->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 320, 240, 30, PXCCapture::Device::STREAM_OPTION_STRONG_STREAM_SYNC);

	// set pause mode, that is, return the same sample of the current
	// frame repeatedly.
	capManager->SetRealtime(false);
	capManager->SetPause(true);
	
	// Create stream renders
	//UtilRender renderc(L"Color"), renderd(L"Depth"), renderi(L"IR");
	UtilRender renderDepth(L"Depth");

	// process 300 frames
	for (int i = 0; i<300; i++) {

		// playback mode stuff
	
		capManager->SetFrameByIndex(i);		//set to current frame
		ourRealsense->FlushFrame();			//used in playback to empty cache
		
		// This function blocks until a color is ready
		pxcStatus frameStatus = ourRealsense->AcquireFrame(true);
		if (frameStatus<PXC_STATUS_NO_ERROR) break;

		// Retrieve the sample
		PXCCapture::Sample *sample = ourRealsense->QuerySample();

		// Work on the image sample->color
		// render the frame or exit if not able to render
		// works for both recording and playback
		if (sample->depth && !renderDepth.RenderFrame(sample->depth)) break;

		// Prepare for the next sample
		ourRealsense->ReleaseFrame();

		// playback recording frame by frame 
		// wait for keyboard hit to go to next frame
		while (!_kbhit()) {
		}
		int c = _getch();
		if (c == 27 || c == 'q' || c == 'Q') break; // ESC|q|Q for Exit


	}
	ourRealsense->Release();

}