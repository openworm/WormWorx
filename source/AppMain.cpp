// App main file
//-----------------------------------------------------------------------------

#include "s3e.h"
#include "IwDebug.h"
#include "IwGx.h"
#include "IwGL.h"
#include "Iw2D.h"
#include "IwGxPrint.h"
#include "IwTexture.h"
#include "IwMaterial.h"
#include "WormWorx.h"

// Attempt to lock to 25 frames per second
#define MS_PER_FRAME    (1000 / 25)

void getKeyDimensions(int key, int& width, int& height, int& x, int& y)
{
   int w = (int)IwGxGetScreenWidth();
   int h = (int)IwGxGetScreenHeight();

   width  = w;
   height = h;
   if (width < height)
   {
      width *= 0.1;
      height = width;
   }
   else
   {
      height *= 0.1;
      width   = height;
   }

   x = 0;
   y = 0;

   switch (key)
   {
   case QUIT_KEY:
      y = h - height;
      x = w - width;
      break;

   case RUN_KEY:
      y = h - height;
      x = 0;
      break;

   case SKIN_KEY:
      x = 0;
      y = 0;
      break;

   case CONNECTOME_KEY:
      y = 0;
      x = w - width;
      break;
   }
}


int TestKey(int pointerx, int pointery)
{
   int width, height, x, y;

   for (int key = 0; key < 4; key++)
   {
      getKeyDimensions(key, width, height, x, y);

      if ((pointerx >= x) && (pointerx <= x + width) && (pointery >= y) && (pointery <= y + height))
      {
         return(key);
      }
   }
   return(-1);
}


void RenderKey(int key, CIwMaterial *fadeMat, CIwColour *cols_light, CIwColour *cols_dark)
{
   int width, height, x, y;

   getKeyDimensions(key, width, height, x, y);
   CIwSVec2 XY(x, y - 2), dXY(width, height);
   CIwFVec2 p;
   p.x = (float)XY.x;
   p.y = (float)XY.y;
   CIwFVec2 s;
   s.x = (float)width;
   s.y = (float)height;

   CIwColour *cols = cols_light;
   if (s3ePointerGetState(S3E_POINTER_BUTTON_SELECT) & S3E_POINTER_STATE_DOWN)
   {
      int pointerx = s3ePointerGetX();
      int pointery = s3ePointerGetY();
      if ((pointerx >= x) && (pointerx <= x + width) && (pointery >= y) && (pointery <= y + height))
      {
         cols = cols_dark;
      }
   }

   // Draw key.
   switch (key)
   {
   case QUIT_KEY:
      Iw2DSetColour(0xffffffff);
      Iw2DFillRect(CIwFVec2(x, y), CIwFVec2(width, height));
      IwGxDrawRectScreenSpace(&XY, &dXY, cols);
      Iw2DDrawImage(QuitImage, p, s);
      break;

   case RUN_KEY:
      Iw2DSetColour(0xffffffff);
      Iw2DFillRect(CIwFVec2(x, y), CIwFVec2(width, height));
      IwGxDrawRectScreenSpace(&XY, &dXY, cols);
      switch (AppGetRunState())
      {
      case START:
         Iw2DDrawImage(StartImage, p, s);
         break;

      case RUN:
         Iw2DDrawImage(PauseImage, p, s);
         break;

      case RESTART:
         Iw2DDrawImage(RestartImage, p, s);
         break;
      }
      break;

   case SKIN_KEY:
      if (!AppGetSkinState())
      {
         cols = cols_dark;
      }
      IwGxDrawRectScreenSpace(&XY, &dXY, cols);
      Iw2DDrawImage(ScalpelImage, p, s);
      break;

   case CONNECTOME_KEY:
      if (AppGetConnectomeState())
      {
         cols = cols_dark;
      }
      IwGxDrawRectScreenSpace(&XY, &dXY, cols);
      Iw2DDrawImage(LightImage, p, s);
      break;
   }
   Iw2DSetColour(0xff000000);
   Iw2DDrawRect(CIwFVec2(x, y - 2), CIwFVec2(width, height));
}


void RenderKeys()
{
   CIwMaterial *fadeMat = IW_GX_ALLOC_MATERIAL();

   fadeMat->SetAlphaMode(CIwMaterial::SUB);
   IwGxSetMaterial(fadeMat);
   CIwColour *cols_light = IW_GX_ALLOC(CIwColour, 4);
   memset(cols_light, 50, sizeof(CIwColour) * 4);
   CIwColour *cols_dark = IW_GX_ALLOC(CIwColour, 4);
   memset(cols_dark, 15, sizeof(CIwColour) * 4);

   RenderKey(QUIT_KEY, fadeMat, cols_light, cols_dark);
   RenderKey(RUN_KEY, fadeMat, cols_light, cols_dark);
   RenderKey(SKIN_KEY, fadeMat, cols_light, cols_dark);
   RenderKey(CONNECTOME_KEY, fadeMat, cols_light, cols_dark);
}


void RenderSoftkeys()
{
}


int main()
{
   IwGxInit();
   AppInit();

   IwGxSetColClear(0xff, 0xff, 0xff, 0xff);
   IwGxPrintSetColour(128, 128, 128);

   while (1)
   {
      s3eDeviceYield(0);
      s3eKeyboardUpdate();
      s3ePointerUpdate();

      int64 start = s3eTimerGetMs();

      bool result = AppUpdate();
      if (
         ((result == false) ||
          (s3eKeyboardGetState(s3eKeyEsc) & S3E_KEY_STATE_DOWN) ||
          (s3eKeyboardGetState(s3eKeyAbsBSK) & S3E_KEY_STATE_DOWN) ||
          (s3eDeviceCheckQuitRequest()))
         )
      {
         break;
      }

      IwGxClear(IW_GX_COLOUR_BUFFER_F | IW_GX_DEPTH_BUFFER_F);
      RenderSoftkeys();
      AppRender();

      while ((s3eTimerGetMs() - start) < MS_PER_FRAME)
      {
         int32 yield = (int32)(MS_PER_FRAME - (s3eTimerGetMs() - start));
         if (yield < 0)
         {
            break;
         }
         s3eDeviceYield(yield);
      }
   }

   AppShutDown();
   IwGxTerminate();
   return(0);
}
