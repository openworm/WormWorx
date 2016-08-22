// WormWorx.

#ifndef WORMWORX_H
#define WORMWORX_H

#include "IwGeom.h"
#include "s3ePointer.h"
#include "s3eKeyboard.h"

typedef enum SoftKeyCodes
{
   QUIT_KEY       = S3E_DEVICE_SOFTKEY_BOTTOM_RIGHT,
   RUN_KEY        = S3E_DEVICE_SOFTKEY_BOTTOM_LEFT,
   SKIN_KEY       = S3E_DEVICE_SOFTKEY_TOP_LEFT,
   CONNECTOME_KEY = S3E_DEVICE_SOFTKEY_TOP_RIGHT
} SoftKeyCodes;

typedef enum CursorKeyCodes
{
   EXCURSOR_NONE = 0,
   EXCURSOR_CLICK,
   EXCURSOR_UP,
   EXCURSOR_DOWN,
   EXCURSOR_LEFT,
   EXCURSOR_RIGHT
} CursorKeyCodes;

// App functions.
void AppInit();
void AppShutDown();
bool AppUpdate();
void AppRender();
void AppSetRunState();
int AppGetRunState();
void AppSetSkinState();
bool AppGetSkinState();
void AppSetConnectomeState();
bool AppGetConnectomeState();

// Images.
extern CIw2DImage *QuitImage;
extern CIw2DImage *StartImage;
extern CIw2DImage *PauseImage;
extern CIw2DImage *ScalpelImage;
extern CIw2DImage *LightImage;

S3E_BEGIN_C_DECL

int TestKey(int pointerx, int pointery);
void RenderKeys();
void RenderSoftkeys();

S3E_END_C_DECL

#endif
