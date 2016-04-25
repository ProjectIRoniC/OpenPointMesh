#include <set>

#ifndef _OPM_NIVIEWER
#define _OPM_NIVIEWER

/**
 * @author - nicole cranon
 * used for blurry frame omission
 */
extern std::set<int> *viewerOmittedFrames;
extern int currentFrame;

// --------------------------------
// Defines
// --------------------------------
#define SAMPLE_XML_PATH "../../../../Data/SamplesConfig.xml"

// --------------------------------
// Types
// --------------------------------
enum {
	ERR_OK,
	ERR_USAGE,
	ERR_DEVICE,
	ERR_UNKNOWN
};

// --------------------------------
// Global Variables
// --------------------------------
/* When true, frames will not be read from the device. */
extern bool g_bPause;
/* When true, only a single frame will be read, and then reading will be paused. */
extern bool g_bStep;
extern bool g_bShutdown;

// extern IntPair mouseLocation;
// extern IntPair windowSize;

// --------------------------------
// Utilities
// --------------------------------
void motionCallback(int x, int y);

void mouseCallback(int button, int state, int x, int y);

void keyboardCallback(unsigned char key, int /*x*/, int /*y*/);

void keyboardSpecialCallback(int key, int /*x*/, int /*y*/);

void reshapeCallback(int width, int height);

void idleCallback();

void step(int);

void seek(int nDiff);

void seekToUserInputEnd(bool ok, const char* userInput);

void seekToUserInputBegin(int);

void init_opengl();

void closeSample(int errCode);

void togglePause(int);

void startCapture(int delay);

void createKeyboardMap();

void createMenu();

void onExit();

int changeDirectory(char* arg0);

void recordFrameNumber (std::set<int>& frames);

void initOmittedFrameSet ();

std::set<int> getOmittedFrameSet ();

void destroyOmittedFrameSet ();

void initLaunchViewer (char* );

#endif
