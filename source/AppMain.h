// Application core.

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "IwGeom.h"
#include "s3ePointer.h"
#include "s3eKeyboard.h"

typedef enum CursorKeyCodes
{
   EXCURSOR_NONE = 0,
   EXCURSOR_CLICK,
   EXCURSOR_UP,
   EXCURSOR_DOWN,
   EXCURSOR_LEFT,
   EXCURSOR_RIGHT
} CursorKeyCodes;

typedef void (*exbutton_handler)();

typedef struct ExButtons
{
   char             name[64];
   int              x;
   int              y;
   int              w;
   int              h;
   s3eKey           key;
   int32            key_state;
   exbutton_handler handler;
   ExButtons        *next;
   ExButtons()
   {
      name[0]   = '\0';
      x         = 0;
      y         = 0;
      w         = 0;
      h         = 0;
      key       = s3eKeyFirst;
      key_state = 0;
      handler   = NULL;
      next      = NULL;
   }
} ExButtons;

S3E_BEGIN_C_DECL

int AddButton(const char *text, int x, int y, int w, int h, s3eKey key, exbutton_handler handler = NULL);
void DeleteButtons();
void RenderButtons();
void RemoveButton(const char *text);
int32 CheckButton(const char *text);
int TestSoftkey(int pointerx, int pointery);
void RenderSoftkeys();

// Allocate (and configure) a vertex stream for rendering a 'fullscreen' backdrop that
// does not obscure the Ideaworks logo & softkeys
CIwSVec2 *AllocClientScreenRectangle();
void DisplayMessage(const char *strmessage);

int RenderActionkey(const char *text, int x, int y, void (*handler)() = NULL);

S3E_END_C_DECL

#endif /* !APP_MAIN_H */
