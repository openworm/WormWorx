// App main file
//-----------------------------------------------------------------------------

#include "s3e.h"
#include "IwDebug.h"
#include "IwGx.h"
#include "IwGL.h"
#include "Iw2D.h"
#include "IwGxFont.h"
#include "IwGxPrint.h"
#include "IwTexture.h"
#include "IwMaterial.h"

#include "AppMain.h"

// Globlal for buttons link list
ExButtons          *g_ButtonsHead    = NULL;
ExButtons          *g_ButtonsTail    = NULL;
CursorKeyCodes     g_Cursorkey       = EXCURSOR_NONE;
bool               g_CursorEnabled   = true;
static CIwMaterial *g_CursorMaterial = NULL;
static bool        g_EnableQuit      = true;
static bool        g_DrawQuit        = true;
int                g_CursorHeight    = 20;
int                g_CursorWidth     = 45;
int                g_CursorXmargin   = 10;

// Externs for app functions.
void AppInit();
void AppCheckQuit();
void AppShutDown();
void AppRender();
bool AppUpdate();
void AppRun();
int AppRunState();

// Font.
extern CIwGxFont *font;

// Attempt to lock to 25 frames per second
#define MS_PER_FRAME    (1000 / 25)

// Helper function to display message for Debug-Only Apps
void DisplayMessage(const char *strmessage)
{
   uint16 *screen = (uint16 *)s3eSurfacePtr();
   int32  width   = s3eSurfaceGetInt(S3E_SURFACE_WIDTH);
   int32  height  = s3eSurfaceGetInt(S3E_SURFACE_HEIGHT);
   int32  pitch   = s3eSurfaceGetInt(S3E_SURFACE_PITCH);

   for (int y = 0; y < height; y++)
   {
      for (int x = 0; x < width; x++)
      {
         screen[y * pitch / 2 + x] = 0;
      }
   }
   s3eDebugPrint(0, 10, strmessage, 1);
   s3eSurfaceShow();
   while (!s3eDeviceCheckQuitRequest() && !s3eKeyboardAnyKey())
   {
      s3eDeviceYield(0);
      s3eKeyboardUpdate();
   }
}


CIwSVec2 *AllocClientScreenRectangle()
{
   CIwSVec2 *pCoords = IW_GX_ALLOC(CIwSVec2, 4);

   pCoords[0].x = 0;
   pCoords[0].y = 0;
   pCoords[1].x = 0;
   pCoords[1].y = (int16)IwGxGetScreenHeight();
   pCoords[2].x = (int16)IwGxGetScreenWidth();
   pCoords[2].y = 0;
   pCoords[3].x = (int16)IwGxGetScreenWidth();
   pCoords[3].y = (int16)IwGxGetScreenHeight();

   return(pCoords);
}


void RenderSoftkey(const char *text, s3eDeviceSoftKeyPosition pos, void (*handler)())
{
   int width  = (int)IwGxGetScreenWidth();
   int height = (int)IwGxGetScreenHeight();

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

   int x = 0;
   int y = 0;
   switch (pos)
   {
   case S3E_DEVICE_SOFTKEY_BOTTOM_LEFT:
      y = IwGxGetScreenHeight() - height;
      x = 0;
      break;

   case S3E_DEVICE_SOFTKEY_BOTTOM_RIGHT:
      y = IwGxGetScreenHeight() - height;
      x = IwGxGetScreenWidth() - width;
      break;

   case S3E_DEVICE_SOFTKEY_TOP_RIGHT:
      y = 0;
      x = IwGxGetScreenWidth() - width;
      break;

   case S3E_DEVICE_SOFTKEY_TOP_LEFT:
      x = 0;
      y = 0;
      break;
   }

   CIwMaterial *fadeMat = IW_GX_ALLOC_MATERIAL();
   fadeMat->SetAlphaMode(CIwMaterial::SUB);
   IwGxSetMaterial(fadeMat);
   CIwColour *cols = IW_GX_ALLOC(CIwColour, 4);
   memset(cols, 50, sizeof(CIwColour) * 4);

   if (s3ePointerGetState(S3E_POINTER_BUTTON_SELECT) & S3E_POINTER_STATE_DOWN)
   {
      int pointerx = s3ePointerGetX();
      int pointery = s3ePointerGetY();
      if ((pointerx >= x) && (pointerx <= x + width) && (pointery >= y) && (pointery <= y + height))
      {
         memset(cols, 15, sizeof(CIwColour) * 4);
         handler();
      }
   }

   // Draw button area
   CIwSVec2 XY(x, y - 2), dXY(width, height);
   IwGxDrawRectScreenSpace(&XY, &dXY, cols);

   // Label button.
   //IwGxFontSetRect(CIwRect(x + 10, y + 10, dXY.x, dXY.y));
   //IwGxFontDrawText(text);
   //IwGxPrintSetScale(1);
   //IwGxPrintString(x + 10, y + 10, text, false);
   if (strcmp(text, "Quit") == 0)
   {
      Iw2DDrawLine(CIwFVec2(XY.x + 5, XY.y + 5), CIwFVec2(XY.x + width - 5, XY.y + height - 5));
      Iw2DDrawLine(CIwFVec2(XY.x + 5, XY.y + height - 5), CIwFVec2(XY.x + width - 5, XY.y + 5));
   }
   else if (strcmp(text, "Pause") == 0)
   {
      Iw2DDrawLine(CIwFVec2(XY.x + (width / 3), XY.y + 5), CIwFVec2(XY.x + (width / 3), XY.y + height - 5));
      Iw2DDrawLine(CIwFVec2(XY.x + (2 * width / 3), XY.y + 5), CIwFVec2(XY.x + (2 * width / 3), XY.y + height - 5));
   }
   else if ((strcmp(text, "Start") == 0) || (strcmp(text, "Reset") == 0))
   {
      Iw2DDrawLine(CIwFVec2(XY.x + (width / 4), XY.y + 5), CIwFVec2(XY.x + (width / 4), XY.y + height - 5));
      Iw2DDrawLine(CIwFVec2(XY.x + (width / 4), XY.y + 5), CIwFVec2(XY.x + (3 * width / 4), XY.y + (height / 2)));
      Iw2DDrawLine(CIwFVec2(XY.x + (width / 4), XY.y + height - 5), CIwFVec2(XY.x + (3 * width / 4), XY.y + (height / 2)));
   }
}


void RenderSoftkeys()
{
   if (g_EnableQuit && g_DrawQuit)
   {
      int back = s3eDeviceGetInt(S3E_DEVICE_BACK_SOFTKEY_POSITION);
      RenderSoftkey("Quit", (s3eDeviceSoftKeyPosition)back, AppCheckQuit);
   }
   int run = s3eDeviceGetInt(S3E_DEVICE_ADVANCE_SOFTKEY_POSITION);
   switch (AppRunState())
   {
   case 0:
      RenderSoftkey("Start", (s3eDeviceSoftKeyPosition)run, AppRun);
      break;

   case 1:
      RenderSoftkey("Pause", (s3eDeviceSoftKeyPosition)run, AppRun);
      break;

   case 2:
      RenderSoftkey("Reset", (s3eDeviceSoftKeyPosition)run, AppRun);
      break;
   }
}


int AddButton(const char *text, int x, int y, int w, int h, s3eKey key, exbutton_handler handler)
{
   ExButtons *newbutton = new ExButtons;

   strncpy(newbutton->name, text, 63);
   newbutton->x         = x;
   newbutton->y         = y;
   newbutton->w         = w;
   newbutton->h         = h;
   newbutton->key       = key;
   newbutton->key_state = 0;
   newbutton->handler   = handler;
   newbutton->next      = NULL;

   ExButtons *pbutton = g_ButtonsHead;

   if (g_ButtonsHead)
   {
      while (pbutton->next != NULL)
      {
         pbutton = pbutton->next;
      }
      pbutton->next = newbutton;
   }
   else
   {
      g_ButtonsHead = newbutton;
   }

   return(1);
}


int32 CheckButton(const char *text)
{
   ExButtons *pbutton = g_ButtonsHead;

   if (g_ButtonsHead)
   {
      pbutton = g_ButtonsHead;
      while (pbutton != NULL)
      {
         if (strcmp(text, pbutton->name) == 0)
         {
            return(pbutton->key_state);
         }
         pbutton = pbutton->next;
      }
   }

   return(0);
}


void RenderButtons()
{
   ExButtons *pbutton = g_ButtonsHead;

   if (g_ButtonsHead)
   {
      pbutton = g_ButtonsHead;
      while (pbutton != NULL)
      {
         // Check the key and pointer states.
         pbutton->key_state = s3eKeyboardGetState(pbutton->key);
         if (s3eKeyboardGetState(pbutton->key) & S3E_KEY_STATE_DOWN)
         {
            if (pbutton->handler)
            {
               pbutton->handler();
            }
         }

         if (!(s3ePointerGetState(S3E_POINTER_BUTTON_SELECT) & S3E_POINTER_STATE_UP))
         {
            int pointerx = s3ePointerGetX();
            int pointery = s3ePointerGetY();
            if ((pointerx >= pbutton->x) && (pointerx <= pbutton->x + pbutton->w) && (pointery >= pbutton->y) && (pointery <= pbutton->y + pbutton->h))
            {
               if (s3ePointerGetState(S3E_POINTER_BUTTON_SELECT) & S3E_POINTER_STATE_DOWN)
               {
                  pbutton->key_state = S3E_KEY_STATE_DOWN;
               }
               if (s3ePointerGetState(S3E_POINTER_BUTTON_SELECT) & S3E_POINTER_STATE_PRESSED)
               {
                  pbutton->key_state = S3E_KEY_STATE_PRESSED;
               }

               if (pbutton->handler)
               {
                  pbutton->handler();
               }
            }
         }

         // Draw the text
         IwGxSetScreenSpaceSlot(0);

         if (s3ePointerGetInt(S3E_POINTER_AVAILABLE))
         {
            CIwMaterial *fadeMat = IW_GX_ALLOC_MATERIAL();
            fadeMat->SetAlphaMode(CIwMaterial::SUB);
            IwGxSetMaterial(fadeMat);

            CIwColour *cols = IW_GX_ALLOC(CIwColour, 4);
            if (pbutton->key_state == S3E_KEY_STATE_DOWN)
            {
               memset(cols, 15, sizeof(CIwColour) * 4);
            }
            else
            {
               memset(cols, 50, sizeof(CIwColour) * 4);
            }

            // Draw button area
            CIwSVec2 XY(pbutton->x, pbutton->y - 2), dXY(pbutton->w, pbutton->h);
            IwGxDrawRectScreenSpace(&XY, &dXY, cols);
         }

         IwGxPrintString(pbutton->x + 2, pbutton->y + ((pbutton->h - 10) / 2), pbutton->name, false);
         pbutton = pbutton->next;
      }
   }
}


void DeleteButtons()
{
   ExButtons *pbutton     = g_ButtonsHead;
   ExButtons *pbuttonnext = NULL;

   while (pbutton != NULL)
   {
      pbuttonnext = pbutton->next;
      delete pbutton;
      pbutton     = pbuttonnext;
      pbuttonnext = NULL;
   }
   g_ButtonsHead = NULL;
}


void RemoveButton(const char *text)
{
   ExButtons *button     = g_ButtonsHead;
   ExButtons *prevbutton = NULL;

   while (button != NULL)
   {
      if (strcmp(text, button->name) == 0)
      {
         // break list
         if (prevbutton != NULL)
         {
            prevbutton->next = button->next;
         }
         else
         {
            g_ButtonsHead = button->next;
         }
         delete button;
         return;
      }
      prevbutton = button;
      button     = button->next;
   }
}


//-----------------------------------------------------------------------------
// Main global function
//-----------------------------------------------------------------------------
int main()
{
#ifdef APP_DEBUG_ONLY
   // Test for Debug only apps
#ifndef IW_DEBUG
   DisplayMessage("This app is designed to run from a Debug build. Please build the app in Debug mode and run it again.");
   return(0);
#endif
#endif
   // We can not use g_EnableExit because it is disable all ways to exit application in Win32 Simulator:
   // - Exit button
   // - Alt + F4
   // - Cross button.
   // But we need to have way to close application in simulator but not display exit button.
   // Because draw exit button lead to incorrect pre-compiled shaders generation for Windows Store 8.0/8.1.
   g_DrawQuit = !(s3eDeviceGetInt(S3E_DEVICE_OS) == S3E_OS_ID_WINDOWS &&
                  (IwGetCompileShadersPlatformType() == IW_CS_OS_ID_WS8 ||
                   IwGetCompileShadersPlatformType() == IW_CS_OS_ID_WS81
                  )
                  );
   g_EnableQuit =
      !(s3eDeviceGetInt(S3E_DEVICE_OS) == S3E_OS_ID_WS8 ||
        s3eDeviceGetInt(S3E_DEVICE_OS) == S3E_OS_ID_WS81 ||
        s3eDeviceGetInt(S3E_DEVICE_OS) == S3E_OS_ID_WIN10);

   //IwGx can be initialised in a number of different configurations to help the linker eliminate unused code.
   //Normally, using IwGxInit() is sufficient.
   //To only include some configurations, see the documentation for IwGxInit_Base(), IwGxInit_GLRender() etc.
   IwGxInit();

   // App main loop
   AppInit();

   // Set screen clear colour
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
         g_EnableQuit &&
         ((result == false) ||
          (s3eKeyboardGetState(s3eKeyEsc) & S3E_KEY_STATE_DOWN) ||
          (s3eKeyboardGetState(s3eKeyAbsBSK) & S3E_KEY_STATE_DOWN) ||
          (s3eDeviceCheckQuitRequest()))
         )
      {
         break;
      }

      // Clear the screen
      IwGxClear(IW_GX_COLOUR_BUFFER_F | IW_GX_DEPTH_BUFFER_F);
      RenderButtons();
      RenderSoftkeys();
      AppRender();

      // Attempt frame rate
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

   delete g_CursorMaterial;
   AppShutDown();
   DeleteButtons();
   IwGxTerminate();
   return(0);
}
