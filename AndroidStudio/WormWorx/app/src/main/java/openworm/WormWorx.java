/*
 * WormWorx: a simulation of the C. elegans nematode worm.
 * User tunes the worm's nervous system to allow it to find food.
 *
 * Copyright (c) 2016-2018 Tom Portegys (portegys@openworm.org). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name(s) of the author(s) nor the names of other contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR(S) OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

package openworm;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.graphics.Color;
import android.graphics.Rect;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.view.Display;
import android.view.View;
import android.view.MotionEvent;
import android.view.View.OnClickListener;
import android.view.WindowManager;
import android.widget.ImageButton;
import android.widget.FrameLayout;
import android.widget.AbsoluteLayout;

public class WormWorx extends Activity implements OnClickListener
{
   // View.
   WormWorxView view;

   // Screen dimensions.
   int screenWidth, screenHeight;

   // Keys.
   static final int QUIT_KEY       = 0;
   static final int RUN_KEY        = 1;
   static final int SKIN_KEY       = 2;
   static final int CONNECTOME_KEY = 3;
   ImageButton      quitKey;
   int              quitKeyId;
   ImageButton      runKey;
   int              runKeyId;
   ImageButton      skinKey;
   int              skinKeyId;
   ImageButton      connectomeKey;
   int              connectomeKeyId;

   // Drawables.
   Drawable backDrawable, backBorderedDrawable, pauseDrawable, pauseBorderedDrawable,
            quitDrawable, quitBorderedDrawable, restartDrawable, restartBorderedDrawable,
            scalpelDrawable, scalpelBorderedDrawable, startDrawable, startBorderedDrawable,
            touchDrawable, touchBorderedDrawable;

   @Override
   protected void onCreate(Bundle savedInstanceState)
   {
      super.onCreate(savedInstanceState);

      // Get screen dimensions.
      Display screen = getWindowManager().getDefaultDisplay();
      screenWidth  = screen.getWidth();
      screenHeight = screen.getHeight();
      if (screenWidth < screenHeight)
      {
         int d = screenWidth;
         screenWidth = screenHeight;
         screenHeight = d;
      }

      // Create surface view.
      view = new WormWorxView(this, screenWidth, screenHeight);

      // Create drawables.
      backDrawable            = getResources().getDrawable(R.drawable.back);
      backBorderedDrawable    = getResources().getDrawable(R.drawable.back_bordered);
      pauseDrawable           = getResources().getDrawable(R.drawable.pause);
      pauseBorderedDrawable   = getResources().getDrawable(R.drawable.pause_bordered);
      quitDrawable            = getResources().getDrawable(R.drawable.quit);
      quitBorderedDrawable    = getResources().getDrawable(R.drawable.quit_bordered);
      restartDrawable         = getResources().getDrawable(R.drawable.restart);
      restartBorderedDrawable = getResources().getDrawable(R.drawable.restart_bordered);
      scalpelDrawable         = getResources().getDrawable(R.drawable.scalpel);
      scalpelBorderedDrawable = getResources().getDrawable(R.drawable.scalpel_bordered);
      startDrawable           = getResources().getDrawable(R.drawable.start);
      startBorderedDrawable   = getResources().getDrawable(R.drawable.start_bordered);
      touchDrawable           = getResources().getDrawable(R.drawable.touch);
      touchBorderedDrawable   = getResources().getDrawable(R.drawable.touch_bordered);

      // Create buttons.
      FrameLayout    frame   = new FrameLayout(this);
      AbsoluteLayout buttons = new AbsoluteLayout(this);
      quitKey   = new ImageButton(this);
      quitKeyId = View.generateViewId();
      quitKey.setId(quitKeyId);
      quitKey.setForeground(quitDrawable);
      quitKey.setBackgroundColor(Color.WHITE);
      quitKey.setOnTouchListener(new View.OnTouchListener()
                                 {
                                    @Override
                                    public boolean onTouch(View view, MotionEvent event)
                                    {
                                       if (event.getAction() == MotionEvent.ACTION_UP)
                                       {
                                          quitKey.setForeground(quitDrawable);
                                       }
                                       else if (event.getAction() == MotionEvent.ACTION_DOWN)
                                       {
                                          quitKey.setForeground(quitBorderedDrawable);
                                       }
                                       return(false);
                                    }
                                 }
                                 );
      quitKey.setOnClickListener(this);
      runKey   = new ImageButton(this);
      runKeyId = View.generateViewId();
      runKey.setId(runKeyId);
      runKey.setForeground(startDrawable);
      runKey.setBackgroundColor(Color.WHITE);
      runKey.setOnTouchListener(new View.OnTouchListener()
                                {
                                   @Override
                                   public boolean onTouch(View v, MotionEvent event)
                                   {
                                      switch (view.runState)
                                      {
                                      case WormWorxView.START:
                                         if (event.getAction() == MotionEvent.ACTION_UP)
                                         {
                                            runKey.setForeground(startDrawable);
                                         }
                                         else if (event.getAction() == MotionEvent.ACTION_DOWN)
                                         {
                                            runKey.setForeground(startBorderedDrawable);
                                         }
                                         break;

                                      case WormWorxView.RUN:
                                         if (event.getAction() == MotionEvent.ACTION_UP)
                                         {
                                            runKey.setForeground(pauseDrawable);
                                         }
                                         else if (event.getAction() == MotionEvent.ACTION_DOWN)
                                         {
                                            runKey.setForeground(pauseBorderedDrawable);
                                         }
                                         break;

                                      case WormWorxView.RESTART:
                                         if (event.getAction() == MotionEvent.ACTION_UP)
                                         {
                                            runKey.setForeground(restartDrawable);
                                         }
                                         else if (event.getAction() == MotionEvent.ACTION_DOWN)
                                         {
                                            runKey.setForeground(restartBorderedDrawable);
                                         }
                                         break;
                                      }
                                      return(false);
                                   }
                                }
                                );
      runKey.setOnClickListener(this);
      skinKey   = new ImageButton(this);
      skinKeyId = View.generateViewId();
      skinKey.setId(skinKeyId);
      skinKey.setForeground(scalpelDrawable);
      skinKey.setBackgroundColor(Color.WHITE);
      skinKey.setOnTouchListener(new View.OnTouchListener()
                                 {
                                    @Override
                                    public boolean onTouch(View v, MotionEvent event)
                                    {
                                       if (event.getAction() == MotionEvent.ACTION_UP)
                                       {
                                          skinKey.setForeground(scalpelDrawable);
                                          if (view.skinState)
                                          {
                                             skinKey.setBackgroundColor(Color.WHITE);
                                          }
                                          else
                                          {
                                             skinKey.setBackgroundColor(Color.GRAY);
                                          }
                                       }
                                       else if (event.getAction() == MotionEvent.ACTION_DOWN)
                                       {
                                          skinKey.setForeground(scalpelBorderedDrawable);
                                       }
                                       return(false);
                                    }
                                 }
                                 );
      skinKey.setOnClickListener(this);
      connectomeKey   = new ImageButton(this);
      connectomeKeyId = View.generateViewId();
      connectomeKey.setId(connectomeKeyId);
      connectomeKey.setForeground(touchDrawable);
      connectomeKey.setBackgroundColor(Color.WHITE);
      connectomeKey.setOnTouchListener(new View.OnTouchListener()
                                       {
                                          @Override
                                          public boolean onTouch(View v, MotionEvent event)
                                          {
                                             if (view.connectomeState)
                                             {
                                                if (event.getAction() == MotionEvent.ACTION_UP)
                                                {
                                                   connectomeKey.setForeground(backDrawable);
                                                }
                                                else if (event.getAction() == MotionEvent.ACTION_DOWN)
                                                {
                                                   connectomeKey.setForeground(backBorderedDrawable);
                                                }
                                             }
                                             else
                                             {
                                                if (event.getAction() == MotionEvent.ACTION_UP)
                                                {
                                                   connectomeKey.setForeground(touchDrawable);
                                                }
                                                else if (event.getAction() == MotionEvent.ACTION_DOWN)
                                                {
                                                   connectomeKey.setForeground(touchBorderedDrawable);
                                                }
                                             }
                                             return(false);
                                          }
                                       }
                                       );
      connectomeKey.setOnClickListener(this);
      setKeyPositions();
      buttons.addView(quitKey);
      buttons.addView(runKey);
      buttons.addView(skinKey);
      buttons.addView(connectomeKey);
      frame.addView(view);
      frame.addView(buttons);
      setContentView(frame);
      setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
      getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
   }


   // Key click.
   public void onClick(android.view.View v)
   {
      if (v.getId() == quitKeyId)
      {
         view.onClick(QUIT_KEY);
         finishAndRemoveTask();
         System.exit(0);
      }
      if (v.getId() == runKeyId)
      {
         view.onClick(RUN_KEY);
         switch (view.runState)
         {
         case WormWorxView.START:
            runKey.setForeground(startDrawable);
            break;

         case WormWorxView.RUN:
            runKey.setForeground(pauseDrawable);
            break;

         case WormWorxView.RESTART:
            runKey.setForeground(restartDrawable);
            break;
         }
         return;
      }
      if (v.getId() == skinKeyId)
      {
         view.onClick(SKIN_KEY);
         if (view.skinState)
         {
            skinKey.setBackgroundColor(Color.WHITE);
         }
         else
         {
            skinKey.setBackgroundColor(Color.GRAY);
         }
         return;
      }
      if (v.getId() == connectomeKeyId)
      {
         view.onClick(CONNECTOME_KEY);
         if (view.connectomeState)
         {
            connectomeKey.setForeground(backDrawable);
         }
         else
         {
            connectomeKey.setForeground(touchDrawable);
         }
         return;
      }
   }

   // Position keys.
   void setKeyPositions()
   {
      Rect r = getKeyRect(QUIT_KEY);

      AbsoluteLayout.LayoutParams params =
         new AbsoluteLayout.LayoutParams(r.width(), r.height(), r.left, r.top);
      quitKey.setLayoutParams(params);
      r      = getKeyRect(RUN_KEY);
      params = new AbsoluteLayout.LayoutParams(r.width(), r.height(), r.left, r.top);
      runKey.setLayoutParams(params);
      r      = getKeyRect(SKIN_KEY);
      params = new AbsoluteLayout.LayoutParams(r.width(), r.height(), r.left, r.top);
      skinKey.setLayoutParams(params);
      r      = getKeyRect(CONNECTOME_KEY);
      params = new AbsoluteLayout.LayoutParams(r.width(), r.height(), r.left, r.top);
      connectomeKey.setLayoutParams(params);
   }


   // Get key rectangle.
   Rect getKeyRect(int key)
   {
      int width  = screenWidth;
      int height = screenHeight;
      int w      = width;
      int h      = height;

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
      return(new Rect(x, y, x + width, y + height));
   }


   // Resume.
   @Override
   protected void onResume()
   {
      super.onResume();
      view.resume();
   }


   // Pause.
   @Override
   protected void onPause()
   {
      super.onPause();
      view.pause();
   }
}
