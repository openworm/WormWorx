// WormWorx view.

package openworm;

import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.Rect;
import android.graphics.RectF;
import android.net.Uri;
import android.util.Log;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;

import static openworm.WormWorx.CONNECTOME_KEY;
import static openworm.WormWorx.QUIT_KEY;
import static openworm.WormWorx.RUN_KEY;
import static openworm.WormWorx.SKIN_KEY;

// View.
public class WormWorxView extends SurfaceView
{
   // Context.
   Context  context;
   WormWorx wormworx;

   // Run states.
   static final int START   = 0;
   static final int RUN     = 1;
   static final int RESTART = 2;
   int              runState;

   // Surface holder.
   SurfaceHolder surfaceHolder;

   // Screen dimensions.
   int screenWidth, screenHeight;

   // Images.
   Bitmap bannerBitmap, minusBitmap, motorsBitmap, plusBitmap,
          quitBitmap, saltyBitmap, steeringBitmap;
   Rect bannerSrcRect, minusSrcRect, motorsSrcRect, plusSrcRect,
        quitSrcRect, saltySrcRect, steeringSrcRect;

   // Worm structure.
   static final int   NSEG  = 48;
   static final int   NBAR  = NSEG + 1;
   static final double L_seg = (double)(1e-3) / (float)NSEG;
   static final double D     = (double)(80e-6);
   static double[]     R;
   double[] wormBodyUpdate;
   double[] wormBody;
   PointF[] wormVerts;
   int currentSegment;

   // Salty food.
   static final int NUM_SALTY = 4;
   double[] saltyX;
   double[] saltyY;
   float              saltyX_origin;
   float              saltyY_origin;
   float              saltyX_off;
   float              saltyY_off;
   int                currentSalty;
   static final float SALT_CONSUMPTION_RANGE = 0.65f;
   double              salt_stimulus;

   // Skin/muscles.
   boolean            skinState;
   static final float MUSCLE_WIDTH_SCALE = 0.25f;
   float              muscleWidthScale;

   // Connectome.
   boolean            connectomeState;
   boolean            synapseWeightState;
   boolean            synapseWeightChange;
   SteeringSynapse    currentSynapse;
   float              synapseRadius;
   static final double I     = 0.5;
   static final double theta = -0.5;
   class SteeringNeuron
   {
      double I;
      double theta;
      double activation;
      SteeringNeuron(double I, double theta, double activation)
      {
         this.I          = I;
         this.theta      = theta;
         this.activation = activation;
      }
   };
   SteeringNeuron asel, aser;
   SteeringNeuron aiyl, aiyr;
   SteeringNeuron aizl, aizr;
   SteeringNeuron smbd, smbv;
   class SteeringSynapse
   {
      double  weight;
      PointF position;
      SteeringSynapse()
      {
         weight   = 0.5;
         position = new PointF();
      }


      SteeringSynapse(double weight)
      {
         this.weight = weight;
         position    = new PointF();
      }
   };
   SteeringSynapse asel0;
   SteeringSynapse asel1;
   SteeringSynapse aser0;
   SteeringSynapse aser1;
   SteeringSynapse aiyl0;
   SteeringSynapse aiyr0;
   SteeringSynapse aizl0;
   SteeringSynapse aizl1;
   SteeringSynapse aizr0;
   SteeringSynapse aizr1;
   double[] dorsalMotorActivations;
   double[] ventralMotorActivations;
   static final float XMIN = .38f;
   static final float XMAX = .69f;
   static final float YMIN = .16f;
   static final float YMAX = .62f;
   float              motorYPartition;

   // Transforms.
   static final float   SCALE     = 0.5f;
   static final float   MIN_SCALE = .12f;
   static final float   MAX_SCALE = 2.0f;
   ScaleGestureDetector scaleDetector;
   float                scale, scale2;
   float                x_off, y_off;
   float                x_off2, y_off2;
   int m_x, m_y;
   int m_x2, m_y2;
   int m_x3, m_y3;

   // Scale listener.
   class ScaleListener extends ScaleGestureDetector.SimpleOnScaleGestureListener
   {
      @Override
      public boolean onScale(ScaleGestureDetector detector)
      {
         if (!connectomeState)
         {
            //scale *= detector.getScaleFactor();
            //scale  = Math.max(MIN_SCALE, Math.min(scale, MAX_SCALE));
         }
         else if (!synapseWeightState)
         {
            scale2 *= detector.getScaleFactor();
            scale2  = Math.max(MIN_SCALE, Math.min(scale2, MAX_SCALE));
         }
         invalidate();
         return(true);
      }
   }

   // Banner.
   boolean             showBanner;
   Rect                bannerRect;
   static final String WORMWORX_URL = "http://wormworx.portegys.com";

   // Update worker.
   WormWorxUpdate updateWorker;

   // Display worker.
   WormWorxDisplay displayWorker;

   // Worker synchronization.
   final Object updateLock = new Object();
   boolean      readyToUpdate;
   boolean      updateReady;

   // Synapse file.
   static final String SYNAPSE_FILE = "synapse_weights.txt";

   // Constructor.
   public WormWorxView(Context context, int screenWidth, int screenHeight)
   {
      super(context);
      this.context = context;
      wormworx     = (WormWorx)context;

      // Store screen dimensions.
      this.screenWidth  = screenWidth;
      this.screenHeight = screenHeight;

      // Get surface holder.
      surfaceHolder = getHolder();

      // Load image bitmaps.
      bannerBitmap    = BitmapFactory.decodeResource(this.getResources(), openworm.R.drawable.banner);
      bannerSrcRect   = new Rect(0, 0, bannerBitmap.getWidth(), bannerBitmap.getHeight());
      minusBitmap     = BitmapFactory.decodeResource(this.getResources(), openworm.R.drawable.minus);
      minusSrcRect    = new Rect(0, 0, minusBitmap.getWidth(), minusBitmap.getHeight());
      motorsBitmap    = BitmapFactory.decodeResource(this.getResources(), openworm.R.drawable.motors);
      motorsSrcRect   = new Rect(0, 0, motorsBitmap.getWidth(), motorsBitmap.getHeight());
      plusBitmap      = BitmapFactory.decodeResource(this.getResources(), openworm.R.drawable.plus);
      plusSrcRect     = new Rect(0, 0, plusBitmap.getWidth(), plusBitmap.getHeight());
      quitBitmap      = BitmapFactory.decodeResource(this.getResources(), openworm.R.drawable.quit);
      quitSrcRect     = new Rect(0, 0, quitBitmap.getWidth(), quitBitmap.getHeight());
      saltyBitmap     = BitmapFactory.decodeResource(this.getResources(), openworm.R.drawable.salty);
      saltySrcRect    = new Rect(0, 0, saltyBitmap.getWidth(), saltyBitmap.getHeight());
      steeringBitmap  = BitmapFactory.decodeResource(this.getResources(), openworm.R.drawable.steering);
      steeringSrcRect = new Rect(0, 0, steeringBitmap.getWidth(), steeringBitmap.getHeight());

      // Initialize.
      R = new double[NBAR];
      for (int i = 0; i < NBAR; ++i)
      {
         R[i] = D / 2.0 * Math.abs(Math.sin(Math.acos((i - NSEG / 2.0) / (NSEG / 2.0 + 0.2))));
      }
      init();

      // Create workers.
      updateWorker  = new WormWorxUpdate();
      displayWorker = new WormWorxDisplay();

      // Initialize state.
      runState            = START;
      skinState           = true;
      connectomeState     = false;
      synapseWeightState  = false;
      synapseWeightChange = false;
   }


   // Initialize.
   void init()
   {
      wormBodyUpdate = new double[(NBAR) * 3];
      wormBody       = new double[(NBAR) * 3];
      wormVerts      = new PointF[(NBAR) * 2];
      for (int i = 0; i < wormVerts.length; i++)
      {
         wormVerts[i] = new PointF();
      }
      scale            = MIN_SCALE;
      scale2           = 1.0f;
      scaleDetector    = new ScaleGestureDetector(context, new ScaleListener());
      muscleWidthScale = MUSCLE_WIDTH_SCALE;
      float w = (float)screenWidth;
      float h = (float)screenHeight;
      x_off          = w * 0.775f;
      y_off          = h * 0.5f;
      x_off2         = 0.0f;
      y_off2         = 0.0f;
      m_x            = m_y = -1;
      m_x2           = m_y2 = -1;
      m_x3           = m_y3 = -1;
      currentSegment = -1;
      saltyX         = new double[NUM_SALTY];
      saltyY         = new double[NUM_SALTY];
      saltyX[0]      = -w * 5.0f;
      saltyY[0]      = h * 1.5f;
      saltyX[1]      = -w * 6.0f;
      saltyY[1]      = 0.0f;
      saltyX[2]      = -w * 5.0f;
      saltyY[2]      = -h * 1.5f;
      saltyX[3]      = w * 1.5f;
      saltyY[3]      = 0.0f;
      saltyX_origin  = x_off;
      saltyY_origin  = y_off;
      saltyX_off     = saltyY_off = 0.0f;
      currentSalty   = 0;
      salt_stimulus = 0.0f;
      asel           = new SteeringNeuron(I, theta, 0.0);
      aser           = new SteeringNeuron(I, theta, 0.0);
      aiyl           = new SteeringNeuron(I, theta, 0.0);
      aiyr           = new SteeringNeuron(I, theta, 0.0);
      aizl           = new SteeringNeuron(I, theta, 0.0);
      aizr           = new SteeringNeuron(I, theta, 0.0);
      smbd           = new SteeringNeuron(I, theta, 0.0);
      smbv           = new SteeringNeuron(I, theta, 0.0);
      synapseRadius  = 0.0f;
      asel0          = new SteeringSynapse(1.0);
      asel1          = new SteeringSynapse(0.0);
      aser0          = new SteeringSynapse(0.0);
      aser1          = new SteeringSynapse(1.0);
      aiyl0          = new SteeringSynapse(1.0);
      aiyr0          = new SteeringSynapse(1.0);
      aizl0          = new SteeringSynapse(.7);
      aizl1          = new SteeringSynapse(.7);
      aizr0          = new SteeringSynapse(0.0);
      aizr1          = new SteeringSynapse(3.0 / 5.0);
      try (FileInputStream fin = wormworx.openFileInput(SYNAPSE_FILE))
      {
            DataInputStream in = new DataInputStream(fin);
            BufferedReader  br = new BufferedReader(new InputStreamReader(in));
            asel0.weight = Double.parseDouble(br.readLine());
            asel1.weight = Double.parseDouble(br.readLine());
            aser0.weight = Double.parseDouble(br.readLine());
            aser1.weight = Double.parseDouble(br.readLine());
            aiyl0.weight = Double.parseDouble(br.readLine());
            aiyr0.weight = Double.parseDouble(br.readLine());
            aizl0.weight = Double.parseDouble(br.readLine());
            aizl1.weight = Double.parseDouble(br.readLine());
            aizr0.weight = Double.parseDouble(br.readLine());
            aizr1.weight = Double.parseDouble(br.readLine());
      }
      catch (FileNotFoundException e) {
      }
      catch (IOException e)
      {
         Log.e("WormWorx", "Cannot load synapse weights from file " + SYNAPSE_FILE + ": " + e.getMessage());
      }

      dorsalMotorActivations  = new double[12];
      ventralMotorActivations = new double[12];
      showBanner = true;
      bannerRect = null;
      WormWorxLib.init();
      double[] weights = new double[10];
      weights[0]      = asel0.weight;
      weights[1]      = asel1.weight;
      weights[2]      = aser0.weight;
      weights[3]      = aser1.weight;
      weights[4]      = aiyl0.weight;
      weights[5]      = aiyr0.weight;
      weights[6]      = aizl0.weight;
      weights[7]      = aizl1.weight;
      weights[8]      = aizr0.weight;
      weights[9]      = aizr1.weight;
      WormWorxLib.setSteeringSynapseWeights(weights);
      WormWorxLib.getBody(wormBodyUpdate);
      WormWorxLib.getBody(wormBody);
      logAngles();
      readyToUpdate = false;
      updateReady   = true;
   }

   // Reset.
   void reset()
   {
      scale            = MIN_SCALE;
      scale2           = 1.0f;
      float w = (float)screenWidth;
      float h = (float)screenHeight;
      x_off          = w * 0.775f;
      y_off          = h * 0.5f;
      x_off2         = 0.0f;
      y_off2         = 0.0f;
      m_x            = m_y = -1;
      m_x2           = m_y2 = -1;
      m_x3           = m_y3 = -1;
      currentSegment = -1;
      saltyX_origin  = x_off;
      saltyY_origin  = y_off;
      saltyX_off     = saltyY_off = 0.0f;
      salt_stimulus = 0.0f;
      WormWorxLib.terminate();
      WormWorxLib.init();
      WormWorxLib.getBody(wormBodyUpdate);
      WormWorxLib.getBody(wormBody);
      logAngles();
      runState = START;
      wormworx.runOnUiThread(new Runnable() {
         public void run() {
            wormworx.runKey.setForeground(wormworx.startDrawable);
         }
      });
      connectomeState = false;
      synapseWeightState  = false;
      synapseWeightChange = false;
   }

   // Terminate.
   void terminate()
   {
      try
      {
         FileOutputStream fos    = wormworx.openFileOutput(SYNAPSE_FILE, Context.MODE_PRIVATE);
         PrintWriter      writer = new PrintWriter(fos);
         writer.println(asel0.weight + "");
         writer.println(asel1.weight + "");
         writer.println(aser0.weight + "");
         writer.println(aser1.weight + "");
         writer.println(aiyl0.weight + "");
         writer.println(aiyr0.weight + "");
         writer.println(aizl0.weight + "");
         writer.println(aizl1.weight + "");
         writer.println(aizr0.weight + "");
         writer.println(aizr1.weight + "");
         writer.close();
      }
      catch (IOException e)
      {
         Log.e("WormWorx", "Cannot save synapse weights to file " + SYNAPSE_FILE + ": " + e.getMessage());
      }
      WormWorxLib.terminate();
   }

   // Log segment angles.
   void logAngles() {
      double[] angles = new double[12];
      WormWorxLib.getSegmentAngles(angles);
      String s = "";
      for (int i = 0; i < 11; i++) s += angles[i] + ",";
      s += angles[11];
      //Log.i("WormWorx", s);
   }

   // Update thread.
   class WormWorxUpdate implements Runnable
   {
      Thread  thread;
      boolean running;

      @Override
      public void run()
      {
         while (running)
         {
            // Wait for display.
            synchronized (updateLock)
            {
               while (!readyToUpdate)
               {
                  try
                  {
                     updateLock.wait();
                  }
                  catch (InterruptedException e)
                  {
                     Log.w("WormWorx", "Wait interrupted");
                  }
               }
               readyToUpdate = false;
            }

            // Wait for running state.
            while (running && (runState != RUN))
            {
               try
               {
                  Thread.sleep(100);
               }
               catch (InterruptedException e)
               {
                  Log.w("WormWorx", "Sleep interrupted");
               }
            }

            // Step.
            if (running)
            {
               if (currentSalty != -1)
               {
                  WormWorxLib.step(salt_stimulus);
                  salt_stimulus = 0.0;
                  float cx     = ((float)saltyX[currentSalty] * scale) + saltyX_origin + saltyX_off;
                  float cy     = ((float)saltyY[currentSalty] * scale) + saltyY_origin + saltyY_off;
                  float s = (float)screenWidth * scale / 0.001f;
                  WormWorxLib.getBody(wormBodyUpdate);
                  logAngles();
                  double bx            = (float)(((wormBodyUpdate[0]) * s) + x_off);
                  double by            = (float)(((wormBodyUpdate[1]) * s) + y_off);
                  salt_stimulus = distance(cx, cy, bx, by);
               }
               updateReady = true;
            }
         }
      }

      // Euclidean distance.
      double distance (double x1, double y1, double x2, double y2) {
         double deltaX = x1 - x2;
         double deltaY = y1 - y2;
         return Math.sqrt(deltaX*deltaX + deltaY*deltaY);
      }

      // Resume.
      public void resume()
      {
         running = true;
         thread  = new Thread(this);
         thread.start();
      }


      // Pause.
      public void pause()
      {
         running = false;
         synchronized (updateLock)
         {
            readyToUpdate = true;
            updateLock.notifyAll();
         }
         try
         {
            thread.join();
         }
         catch (InterruptedException e)
         {
            Log.e("WormWorx", "Cannot join thread");
         }
      }
   }

   // Display thread.
   class WormWorxDisplay implements Runnable
   {
      // Target frame rate.
      static final int TARGET_FRAME_RATE = 25;

      // Paint.
      Paint paint;

      // Thread.
      Thread  thread;
      boolean running;

      // Reset salty goal?
      boolean saltyReset;

      // Constructor.
      WormWorxDisplay()
      {
         paint = new Paint();
         saltyReset = false;
      }


      @Override
      public void run()
      {
         while (running)
         {
            long t = System.currentTimeMillis();

            // Draw the surface.
            boolean updateDisplayed = false;
            if (surfaceHolder.getSurface().isValid())
            {
               updateDisplayed = draw();
            }

            // Run another update.
            synchronized (updateLock)
            {
               if (updateDisplayed)
               {
                  updateReady   = false;
                  readyToUpdate = true;
                  updateLock.notifyAll();
               }
            }

            // Wait for next draw.
            long s = System.currentTimeMillis() - t;
            s = (1000 / TARGET_FRAME_RATE) - s;
            if (s > 0)
            {
               try
               {
                  Thread.sleep(s);
               }
               catch (InterruptedException e)
               {
                  Log.w("WormWorx", "Sleep interrupted");
               }
            }
         }
      }


      // Draw.
      // Return true if update has been displayed.
      boolean draw()
      {
         boolean updateDisplayed = false;
         Canvas  canvas          = null;

         try
         {
            canvas = surfaceHolder.lockCanvas();
            float width  = (float)screenWidth;
            float height = (float)screenHeight;
            canvas.drawColor(Color.argb(255, 255, 255, 255));
            paint.setStyle(Paint.Style.FILL);
            float s = (float)screenWidth * scale / 0.001f;

            // Worm body update available?
            if (updateReady)
            {
               for (int i = 0; i < wormBody.length; i++)
               {
                  wormBody[i] = wormBodyUpdate[i];
               }
               if (saltyReset)
               {
                  saltyReset = false;
                  reset();
               }
               updateDisplayed = true;
            }

            // Set worm vertices.
            int n2 = (NBAR) * 2;
            for (int i = 0; i < NBAR; ++i)
            {
               double dX = R[i] * Math.cos(wormBody[i * 3 + 2]);
               double dY = R[i] * Math.sin(wormBody[i * 3 + 2]);
               wormVerts[i].x            = (float)(((wormBody[i * 3] + dX) * s) + x_off);
               wormVerts[i].y            = (float)(((wormBody[i * 3 + 1] + dY) * s) + y_off);
               wormVerts[(n2 - 1) - i].x = (float)(((wormBody[i * 3] - dX) * s) + x_off);
               wormVerts[(n2 - 1) - i].y = (float)(((wormBody[i * 3 + 1] - dY) * s) + y_off);
            }

            // Found salty food?
            float range;
            if (width < height)
            {
               range = width * SALT_CONSUMPTION_RANGE;
            }
            else
            {
               range = height * SALT_CONSUMPTION_RANGE;
            }
            if (currentSalty != -1)
            {
               float cx = ((float) saltyX[currentSalty] * scale) + saltyX_origin + saltyX_off;
               float cy = ((float) saltyY[currentSalty] * scale) + saltyY_origin + saltyY_off;
               float r = (float)Math.sqrt(Math.pow((cx - wormVerts[0].x), 2) +
                       Math.pow((cy - wormVerts[0].y), 2));
               r /= scale;
               if (r <= range)
               {
                  currentSalty++;
                  if (currentSalty == NUM_SALTY)
                  {
                     currentSalty = -1;
                     runState     = RESTART;
                     wormworx.runOnUiThread(new Runnable() {
                        public void run() {
                           wormworx.runKey.setForeground(wormworx.restartDrawable);
                        }
                     });
                  } else {
                     saltyReset = true;
                  }
               }
            }

            if (!connectomeState)
            {
               // Draw salty food.
               for (int i = 0; i < NUM_SALTY; i++) {
                  float radius = range * scale;
                  float cx = ((float) saltyX[i] * scale) + saltyX_origin + saltyX_off;
                  float cy = ((float) saltyY[i] * scale) + saltyY_origin + saltyY_off;
                  int c = 0xff000000;
                  if (currentSalty == -1 || i <= currentSalty) {
                     switch (i) {
                        case 0:
                           c = 0xffff0000;
                           break;
                        case 1:
                           c = 0xff00ff00;
                           break;
                        case 2:
                           c = 0xff0000ff;
                           break;
                        case 3:
                           c = 0xffff00ff;
                           break;
                     }
                  } else {
                     switch (i) {
                        case 0:
                           c = 0x33ff0000;
                           break;
                        case 1:
                           c = 0x3300ff00;
                           break;
                        case 2:
                           c = 0x330000ff;
                           break;
                        case 3:
                           c = 0x33ff00ff;
                           break;
                     }
                  }
                  paint.setColor(c);
                  canvas.drawCircle(cx, cy, radius, paint);
                  paint.setColor(Color.WHITE);
                  float r = radius * 0.5f;
                  RectF rect = new RectF(cx - r, cy - r,
                          cx + r, cy + r);
                  canvas.drawBitmap(saltyBitmap, saltySrcRect, rect, paint);
                  if ((currentSalty == -1) || (i < currentSalty)) {
                     canvas.drawBitmap(quitBitmap, quitSrcRect, rect, paint);
                  }
                  if (i == currentSalty) {
                     paint.setColor(0xff000000);
                     paint.setStyle(Paint.Style.STROKE);
                     for (int j = 2; j <= 25; j++)
                     {
                        canvas.drawCircle(cx, cy, (radius * (float)j), paint);
                     }
                     paint.setStyle(Paint.Style.FILL);
                  }
               }

               // Draw worm.
               paint.setColor(0xff777777);
               if (skinState)
               {
                  // Draw skin.
                  Path polyPath = new Path();
                  polyPath.moveTo(wormVerts[0].x, wormVerts[0].y);
                  for (int i = 1, j = wormVerts.length; i < j; i++)
                  {
                     polyPath.lineTo(wormVerts[i].x, wormVerts[i].y);
                  }
                  polyPath.lineTo(wormVerts[0].x, wormVerts[0].y);
                  canvas.drawPath(polyPath, paint);
               }
               else
               {
                  // Draw muscles.
                  for (int i = 0; i < NSEG; i += 4)
                  {
                     for (int n = 0, j = i; n < 2; n++, j += NBAR)
                     {
                        int   k = j + 4;
                        float x = wormVerts[j].x - wormVerts[k].x;
                        float y = wormVerts[j].y - wormVerts[k].y;
                        float a = 0.0f;
                        if (x == 0.0f)
                        {
                           if (y > 0.0f)
                           {
                              a = (float)Math.PI * .5f;
                           }
                           else
                           {
                              a = (float)Math.PI * 1.5f;
                           }
                        }
                        else
                        {
                           a = (float)Math.atan(y / x);
                           if (x > 0.0f)
                           {
                              if (y < 0.0f)
                              {
                                 a += (float)Math.PI * 2.0f;
                              }
                           }
                           else
                           {
                              a += (float)Math.PI;
                           }
                        }
                        a = a * (float)(180.0 / Math.PI);
                        float mx = (wormVerts[j].x + wormVerts[k].x) / 2.0f;
                        float my = (wormVerts[j].y + wormVerts[k].y) / 2.0f;
                        float h  = (float)Math.sqrt(Math.pow(wormVerts[j].x - wormVerts[k].x, 2.0f) +
                                                    Math.pow(wormVerts[j].y - wormVerts[k].y, 2.0f)) / 2.0f;
                        float  l      = (float)L_seg * 4.0f * s / 2.0f;
                        float v = l * muscleWidthScale * (l / h);
                        PointF center = new PointF(mx, my);
                        canvas.rotate(a, mx, my);
                        RectF oval = new RectF(mx - h, my - v, mx + h, my + v);
                        canvas.drawOval(oval, paint);
                        canvas.rotate(-a, mx, my);
                     }
                  }
               }

               // Highlight current segment.
               if (currentSegment != -1)
               {
                  int   j  = currentSegment * 4;
                  int   k  = j + 4;
                  float mx = (float)((wormBody[j * 3] + wormBody[k * 3]) / 2.0);
                  mx = mx * s + x_off;
                  float my = (float)((wormBody[j * 3 + 1] + wormBody[k * 3 + 1]) / 2.0);
                  my = my * s + y_off;
                  float r = (float)(((R[j] + R[k]) / 2.0) * s * 2.0);
                  paint.setColor(0x77ff7777);
                  canvas.drawCircle(mx, my, r, paint);
               }

               // Show banner?
               if (showBanner)
               {
                  paint.setColor(Color.WHITE);
                  float w = width;
                  float h = height;
                  if (width < height)
                  {
                     w = w * .75f;
                  }
                  else
                  {
                     w = w * .25f;
                  }
                  h = height * .05f;
                  float x = (width / 2.0f) - (w / 2.0f);
                  float y = height * .1f;
                  bannerRect = new Rect((int)x, (int)y, (int)(x + w), (int)(y + h));
                  canvas.drawBitmap(bannerBitmap, bannerSrcRect, bannerRect, paint);
               }
            }
            else
            {
               // Get neuron activations.
               double[] a = new double[8];
               WormWorxLib.getSteeringActivations(a);
               asel.activation = a[0];
               aser.activation = a[1];
               aiyl.activation = a[2];
               aiyr.activation = a[3];
               aizl.activation = a[4];
               aizr.activation = a[5];
               smbd.activation = a[6];
               smbv.activation = a[7];
               WormWorxLib.getDorsalMotorActivations(dorsalMotorActivations);
               WormWorxLib.getVentralMotorActivations(ventralMotorActivations);
               if (!synapseWeightState)
               {
                  // Draw connectome.
                  paint.setColor(Color.WHITE);
                  canvas.drawRect(0.0f, 0.0f, width, height, paint);
                  float w = width;
                  float h = height;
                  float r;
                  if (width < height)
                  {
                     w = w * .75f;
                     h = w;
                     r = w * .03f;
                  }
                  else
                  {
                     h = h * .75f;
                     w = h;
                     r = h * .03f;
                  }
                  w            *= scale2;
                  h            *= scale2;
                  r            *= scale2;
                  synapseRadius = r;
                  float x    = (width / 2.0f) - (w / 2.0f) + x_off2;
                  float y    = ((height * .75f) / 2.0f) - (h / 2.0f) + y_off2;
                  RectF rect = new RectF(x, y, x + w, y + h);
                  canvas.drawBitmap(steeringBitmap, steeringSrcRect, rect, paint);
                  if (asel.activation > 0.0f)
                  {
                     paint.setColor(0xff00ff00);
                  }
                  else
                  {
                     paint.setColor(0xff000000);
                  }
                  float left = x + (w * (.337f - .025f));
                  float top  = y + (h * .17f);
                  canvas.drawRect(left, top, left + (w * .06f), top + (h * .02f), paint);
                  left = x + (w * (.36f - .025f));
                  top  = y + (h * .15f);
                  canvas.drawRect(left, top, left + (h * .02f), top + (w * .06f), paint);
                  if (aser.activation > 0.0f)
                  {
                     paint.setColor(0xff00ff00);
                  }
                  else
                  {
                     paint.setColor(0xff000000);
                  }
                  left = x + (w * (.599f + .04f));
                  top  = y + (h * .17f);
                  canvas.drawRect(left, top, left + (w * .06f), top + (h * .02f), paint);
                  left = x + (w * (.62f + .04f));
                  top  = y + (h * .15f);
                  canvas.drawRect(left, top, left + (h * .02f), top + (w * .06f), paint);
                  paint.setColor(Color.BLACK);
                  if (aiyl.activation > 0.0f)
                  {
                     paint.setColor(0xff00ff00);
                  }
                  else
                  {
                     paint.setColor(0xff000000);
                  }
                  left = x + (w * (.337f - .025f));
                  top  = y + (h * .5f);
                  canvas.drawRect(left, top, left + (w * .06f), top + (h * .02f), paint);
                  left = x + (w * (.36f - .025f));
                  top  = y + (h * .48f);
                  canvas.drawRect(left, top, left + (h * .02f), top + (w * .06f), paint);
                  if (aiyr.activation > 0.0f)
                  {
                     paint.setColor(0xff00ff00);
                  }
                  else
                  {
                     paint.setColor(0xff000000);
                  }
                  left = x + (w * (.599f + .04f));
                  top  = y + (h * .5f);
                  canvas.drawRect(left, top, left + (w * .06f), top + (h * .02f), paint);
                  left = x + (w * (.62f + .04f));
                  top  = y + (h * .48f);
                  canvas.drawRect(left, top, left + (h * .02f), top + (w * .06f), paint);
                  if (aizl.activation > 0.0f)
                  {
                     paint.setColor(0xff00ff00);
                  }
                  else
                  {
                     paint.setColor(0xff000000);
                  }
                  left = x + (w * (.337f - .025f));
                  top  = y + (h * .83f);
                  canvas.drawRect(left, top, left + (w * .06f), top + (h * .02f), paint);
                  left = x + (w * (.36f - .025f));
                  top  = y + (h * .81f);
                  canvas.drawRect(left, top, left + (h * .02f), top + (w * .06f), paint);
                  if (aizr.activation > 0.0f)
                  {
                     paint.setColor(0xff00ff00);
                  }
                  else
                  {
                     paint.setColor(0xff000000);
                  }
                  left = x + (w * (.599f + .04f));
                  top  = y + (h * .83f);
                  canvas.drawRect(left, top, left + (w * .06f), top + (h * .02f), paint);
                  left = x + (w * (.62f + .04f));
                  top  = y + (h * .81f);
                  canvas.drawRect(left, top, left + (h * .02f), top + (w * .06f), paint);
                  asel0.position.set(x + (w * (.36f - .0225f)), y + (h * .25f));
                  paint.setColor(0xffffffff);
                  canvas.drawCircle(asel0.position.x, asel0.position.y, r, paint);
                  paint.setColor(0xff000000);
                  left = asel0.position.x - r;
                  top  = asel0.position.y - r;
                  float d = r * 2.0f;
                  canvas.drawArc(left, top, left + d, top + d,
                                 270.0f, 360.0f * (float)asel0.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(asel0.position.x, asel0.position.y, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  asel1.position.set(x + (w * .41f), y + (h * .2f));
                  paint.setColor(0xffffffff);
                  canvas.drawCircle(asel1.position.x, asel1.position.y, r, paint);
                  paint.setColor(0xff000000);
                  left = asel1.position.x - r;
                  top  = asel1.position.y - r;
                  canvas.drawArc(left, top, left + d, top + d,
                                 270.0f, 360.0f * (float)asel1.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(asel1.position.x, asel1.position.y, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  aser0.position.set(x + (w * .6f), y + (h * .2f));
                  paint.setColor(0xffffffff);
                  canvas.drawCircle(aser0.position.x, aser0.position.y, r, paint);
                  paint.setColor(0xff000000);
                  left = aser0.position.x - r;
                  top  = aser0.position.y - r;
                  canvas.drawArc(left, top, left + d, top + d,
                                 270.0f, 360.0f * (float)aser0.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(aser0.position.x, aser0.position.y, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  aser1.position.set(x + (w * (.62f + .045f)), y + (h * .25f));
                  paint.setColor(0xffffffff);
                  canvas.drawCircle(aser1.position.x, aser1.position.y, r, paint);
                  paint.setColor(0xff000000);
                  left = aser1.position.x - r;
                  top  = aser1.position.y - r;
                  canvas.drawArc(left, top, left + d, top + d,
                          270.0f, 360.0f * (float)aser1.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(aser1.position.x, aser1.position.y, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  aiyl0.position.set(x + (w * (.36f - .025f)), y + (h * .58f));
                  paint.setColor(0xffffffff);
                  canvas.drawCircle(aiyl0.position.x, aiyl0.position.y, r, paint);
                  paint.setColor(0xff000000);
                  left = aiyl0.position.x - r;
                  top  = aiyl0.position.y - r;
                  canvas.drawArc(left, top, left + d, top + d,
                                 270.0f, 360.0f * (float)aiyl0.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(aiyl0.position.x, aiyl0.position.y, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  aiyr0.position.set(x + (w * (.62f + .045f)), y + (h * .58f));
                  paint.setColor(0xffffffff);
                  canvas.drawCircle(aiyr0.position.x, aiyr0.position.y, r, paint);
                  paint.setColor(0xff000000);
                  left = aiyr0.position.x - r;
                  top  = aiyr0.position.y - r;
                  canvas.drawArc(left, top, left + d, top + d,
                                 270.0f, 360.0f * (float)aiyr0.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(aiyr0.position.x, aiyr0.position.y, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  aizl0.position.set(x + (w * (.36f - .0225f)), y + (h * .91f));
                  paint.setColor(0xffffffff);
                  canvas.drawCircle(aizl0.position.x, aizl0.position.y, r, paint);
                  paint.setColor(0xff000000);
                  left = aizl0.position.x - r;
                  top  = aizl0.position.y - r;
                  canvas.drawArc(left, top, left + d, top + d,
                                 270.0f, 360.0f * (float)aizl0.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(aizl0.position.x, aizl0.position.y, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  aizl1.position.set(x + (w * .41f), y + (h * .86f));
                  paint.setColor(0xffffffff);
                  canvas.drawCircle(aizl1.position.x, aizl1.position.y, r, paint);
                  paint.setColor(0xff000000);
                  left = aizl1.position.x - r;
                  top  = aizl1.position.y - r;
                  canvas.drawArc(left, top, left + d, top + d,
                                 270.0f, 360.0f * (float)aizl1.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(aizl1.position.x, aizl1.position.y, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  aizr0.position.set(x + (w * .6f), y + (h * .86f));
                  paint.setColor(0xffffffff);
                  canvas.drawCircle(aizr0.position.x, aizr0.position.y, r, paint);
                  paint.setColor(0xff000000);
                  left = aizr0.position.x - r;
                  top  = aizr0.position.y - r;
                  canvas.drawArc(left, top, left + d, top + d,
                                 270.0f, 360.0f * (float)aizr0.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(aizr0.position.x, aizr0.position.y, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  aizr1.position.set(x + (w * (.62f + .045f)), y + (h * .91f));
                  paint.setColor(0xffffffff);
                  canvas.drawCircle(aizr1.position.x, aizr1.position.y, r, paint);
                  paint.setColor(0xff000000);
                  left = aizr1.position.x - r;
                  top  = aizr1.position.y - r;
                  canvas.drawArc(left, top, left + d, top + d,
                          270.0f, 360.0f * (float)aizr1.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(aizr1.position.x, aizr1.position.y, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  motorYPartition = y + h;
                  RectF motorsRect = getMotorConnectomeGeometry();
                  float w2         = motorsRect.right - motorsRect.left;
                  float h2         = motorsRect.bottom - motorsRect.top;
                  for (int i = 0; i < 12; i++)
                  {
                     paint.setColor(0xffffffff);
                     rect = new RectF(motorsRect.left, motorsRect.top + (h2 * (float)i),
                                      motorsRect.right, motorsRect.bottom + (h2 * (float)i));
                     canvas.drawBitmap(motorsBitmap, motorsSrcRect, rect, paint);
                     if (ventralMotorActivations[i] > 0.0f)
                     {
                        paint.setColor(0xff0000ff);
                     }
                     else
                     {
                        paint.setColor(0xff000000);
                     }
                     left = rect.left + (w2 * XMIN) - (w2 * .075f);
                     top  = rect.top + (h2 * YMAX) - (h2 * .0125f);
                     canvas.drawRect(left, top, left + (w2 * .08f), top + (h2 * .04f), paint);
                     if (dorsalMotorActivations[i] > 0.0f)
                     {
                        paint.setColor(0xff0000ff);
                     }
                     else
                     {
                        paint.setColor(0xff000000);
                     }
                     left = rect.left + (w2 * XMAX) - (w2 * .075f);
                     top  = rect.top + (h2 * YMAX) - (h2 * .0125f);
                     canvas.drawRect(left, top, left + (w2 * .08f), top + (h2 * .04f), paint);
                     if (dorsalMotorActivations[i] > 0.0f)
                     {
                        paint.setColor(0xff00ff00);
                     }
                     else
                     {
                        paint.setColor(0xff000000);
                     }
                     left = rect.left + (w2 * XMIN) - (w2 * .08f);
                     top  = rect.top + (h2 * YMIN) - (h2 * .0125f);
                     canvas.drawRect(left, top, left + (w2 * .08f), top + (h2 * .04f), paint);
                     left = rect.left + (w2 * XMIN) - (h2 * .1f);
                     top  = rect.top + (h2 * YMIN) - (w2 * .038f);
                     canvas.drawRect(left, top, left + (h2 * .04f), top + (w2 * .08f), paint);
                     if (ventralMotorActivations[i] > 0.0f)
                     {
                        paint.setColor(0xff00ff00);
                     }
                     else
                     {
                        paint.setColor(0xff000000);
                     }
                     left = rect.left + (w2 * XMAX) - (w2 * .065f);
                     top  = rect.top + (h2 * YMIN) - (h2 * .0125f);
                     canvas.drawRect(left, top, left + (w2 * .08f), top + (h2 * .04f), paint);
                     left = rect.left + (w2 * XMAX) - (h2 * .065f);
                     top  = rect.top + (h2 * YMIN) - (w2 * .038f);
                     canvas.drawRect(left, top, left + (h2 * .04f), top + (w2 * .08f), paint);
                     if (currentSegment == i)
                     {
                        paint.setColor(0xffff7777);
                        paint.setStyle(Paint.Style.STROKE);
                        left = rect.left;
                        top  = rect.top;
                        canvas.drawRect(left, top, left + w2, top + (h2 - 1), paint);
                        left = rect.left + 1;
                        top  = rect.top + (h2 * i) + 1;
                        canvas.drawRect(left, top, left + (w2 - 2), top + (h2 - 3), paint);
                        paint.setStyle(Paint.Style.FILL);
                     }
                  }
               }
               else
               {
                  // Draw synapse weighting.
                  float w = (float)screenWidth;
                  float h = (float)screenHeight;
                  paint.setColor(0xffffffff);
                  canvas.drawRect(0.0f, 0.0f, w, h, paint);
                  float r;
                  if (w < h)
                  {
                     r = w * .15f;
                  }
                  else
                  {
                     r = h * .15f;
                  }
                  float cx = w / 2.0f;
                  float cy = h / 3.0f;
                  paint.setColor(0xff000000);
                  float left = cx - r;
                  float top  = cy - r;
                  float d    = r * 2.0f;
                  paint.setStyle(Paint.Style.FILL);
                  canvas.drawArc(left, top, left + d, top + d,
                                 270.0f, 360.0f * (float)currentSynapse.weight, true, paint);
                  paint.setStyle(Paint.Style.STROKE);
                  canvas.drawCircle(cx, cy, r, paint);
                  paint.setStyle(Paint.Style.FILL);
                  left = w * .1f;
                  top  = h * .65f;
                  canvas.drawRect(left, top, left + (w * .8f), top + (h * .01f), paint);
                  left = w * (.1f - .025f);
                  top  = h * .62f;
                  canvas.drawRect(left, top, left + (w * .025f), top + (h * .06f), paint);
                  left = w * .9f;
                  top  = h * .62f;
                  canvas.drawRect(left, top, left + (w * .025f), top + (h * .06f), paint);
                  paint.setColor(0xff777777);
                  left = (w * (.8f - .05f) * (float)currentSynapse.weight) + (w * .1f);
                  top  = h * .62f;
                  canvas.drawRect(left, top, left + (w * .05f), top + (h * .06f), paint);
               }
            }
         }
         catch (Exception e)
         {
            Log.e("WormWorx", "Cannot lock surface: " + e.getMessage());
         }
         finally
         {
            // Update screen.
            if (canvas != null)
            {
               surfaceHolder.unlockCanvasAndPost(canvas);
            }
         }

         return(updateDisplayed);
      }


      // Resume.
      public void resume()
      {
         running = true;
         thread  = new Thread(this);
         thread.start();
      }


      // Pause.
      public void pause()
      {
         running = false;
         try
         {
            thread.join();
         }
         catch (InterruptedException e)
         {
            Log.e("WormWorx", "Cannot join thread");
         }
      }
   }

   // Key click.
   void onClick(int key)
   {
      showBanner = false;
      switch (key)
      {
      case QUIT_KEY:
         terminate();
         wormworx.finishAndRemoveTask();
         break;

      case RUN_KEY:
         switch (runState)
         {
         case START:
            runState = RUN;
            break;

         case RUN:
            runState = START;
            break;

         case RESTART:
            terminate();
            init();
            break;
         }
         break;

      case SKIN_KEY:
         skinState = !skinState;
         break;

      case CONNECTOME_KEY:
         if (synapseWeightState)
         {
            synapseWeightState = false;
            if (synapseWeightChange)
            {
               synapseWeightChange = false;
               pause();
               terminate();
               init();
               resume();
            }
         }
         else
         {
            connectomeState = !connectomeState;
         }
         break;
      }
   }


   // Touch events.
   @Override
   public boolean onTouchEvent(MotionEvent event)
   {
      // Detect scale.
      scaleDetector.onTouchEvent(event);

      // Ignore multi-touch.
      if (event.getPointerCount() > 1)
      {
         return(true);
      }

      int mx = (int)event.getX();
      int my = (int)event.getY();
      switch (event.getAction())
      {
      case MotionEvent.ACTION_DOWN:

         // Touched banner?
         boolean sb = showBanner;
         showBanner = false;
         if (sb && (bannerRect != null) && bannerRect.contains(mx, my))
         {
            // Bring up WormWorx web page.
            wormworx.startActivity(new Intent(Intent.ACTION_VIEW, Uri.parse(WORMWORX_URL)));
         }
         else
         {
            if (!connectomeState)
            {
               m_x  = mx;
               m_y  = my;
               m_x2 = m_y2 = -1;
               m_x3 = m_y3 = -1;
            }
            else if (!synapseWeightState)
            {
               m_x  = m_y = -1;
               m_x3 = m_y3 = -1;
               m_x2 = mx;
               m_y2 = my;
               setCurrentSegment(mx, my);
               setSynapseWeighting(mx, my);
            }
            else
            {
               m_x  = m_y = -1;
               m_x2 = m_y2 = -1;
               m_x3 = mx;
               m_y3 = my;
            }
         }
         break;

      case MotionEvent.ACTION_MOVE:
         if (!connectomeState)
         {
            if (m_x != -1)
            {
               x_off      += mx - m_x;
               y_off      += my - m_y;
               saltyX_off += mx - m_x;
               saltyY_off += my - m_y;
               m_x         = mx;
               m_y         = my;
            }
         }
         else if (!synapseWeightState)
         {
            if (m_x2 != -1)
            {
               x_off2 += mx - m_x2;
               y_off2 += my - m_y2;
               m_x2    = mx;
               m_y2    = my;
            }
         }
         else
         {
            if (m_x3 != -1)
            {
               float w   = (float)screenWidth;
               int   med = mx;
               int   min = (int)(w * (.1 + .025));
               int   max = (int)(w * (.9 - .025));
               if (med < min)
               {
                  med = min;
               }
               if (med > max)
               {
                  med = max;
               }
               med -= min;
               int   len    = (int)(w * (.8 - .05));
               float weight = (float)med / (float)len;
               if (currentSynapse.weight != weight)
               {
                  currentSynapse.weight = weight;
                  synapseWeightChange   = true;
               }
               m_x3 = mx;
               m_y3 = my;
            }
         }
         break;

      case MotionEvent.ACTION_UP:
         m_x  = m_y = -1;
         m_x2 = m_y2 = -1;
         m_x3 = m_y3 = -1;
         break;
      }
      return(true);
   }


   // Get motor connectome geometry.
   RectF getMotorConnectomeGeometry()
   {
      float width  = (float)screenWidth;
      float height = (float)screenHeight;

      float w = width;
      float h = height;

      if (w < h)
      {
         w = (w * 2.0f) / 3.0f;
         h = w;
      }
      else
      {
         h = (h * 2.0f) / 3.0f;
         w = h;
      }
      w = w * scale2;
      h = h * 0.5f * scale2;
      float x = (width / 2.0f) - (w / 2.0f) + x_off2;
      float y = motorYPartition;
      return(new RectF(x, y, x + w, y + h));
   }


   // Set current connectome segment.
   void setCurrentSegment(int mx, int my)
   {
      float x, y, w, h;

      RectF rect = getMotorConnectomeGeometry();

      x = rect.left;
      y = rect.top;
      w = rect.right - rect.left;
      h = rect.bottom - rect.top;
      for (int i = 0; i < 12; i++)
      {
         if ((mx >= x) && (mx < (x + w)) && (my >= y) && (my < (y + h)))
         {
            if ((currentSegment == -1) || (currentSegment != i))
            {
               currentSegment = i;
            }
            else
            {
               currentSegment = -1;
            }
            return;
         }
         y += h;
      }
      currentSegment = -1;
   }


   // Set synapse weighting mode.
   void setSynapseWeighting(int mx, int my)
   {
      synapseWeightState = false;
      float d = (float)Math.sqrt(Math.pow((float)mx - asel0.position.x, 2.0f) +
                                 Math.pow((float)my - asel0.position.y, 2.0f));
      if (d <= synapseRadius)
      {
         synapseWeightState = true;
         currentSynapse     = asel0;
      }
      d = (float)Math.sqrt(Math.pow((float)mx - asel1.position.x, 2.0f) +
                           Math.pow((float)my - asel1.position.y, 2.0f));
      if (d <= synapseRadius)
      {
         synapseWeightState = true;
         currentSynapse     = asel1;
      }
      d = (float)Math.sqrt(Math.pow((float)mx - aser0.position.x, 2.0f) +
                           Math.pow((float)my - aser0.position.y, 2.0f));
      if (d <= synapseRadius)
      {
         synapseWeightState = true;
         currentSynapse     = aser0;
      }
      d = (float)Math.sqrt(Math.pow((float)mx - aser1.position.x, 2.0f) +
                           Math.pow((float)my - aser1.position.y, 2.0f));
      if (d <= synapseRadius)
      {
         synapseWeightState = true;
         currentSynapse     = aser1;
      }
      d = (float)Math.sqrt(Math.pow((float)mx - aiyl0.position.x, 2.0f) +
                           Math.pow((float)my - aiyl0.position.y, 2.0f));
      if (d <= synapseRadius)
      {
         synapseWeightState = true;
         currentSynapse     = aiyl0;
      }
      d = (float)Math.sqrt(Math.pow((float)mx - aiyr0.position.x, 2.0f) +
                           Math.pow((float)my - aiyr0.position.y, 2.0f));
      if (d <= synapseRadius)
      {
         synapseWeightState = true;
         currentSynapse     = aiyr0;
      }
      d = (float)Math.sqrt(Math.pow((float)mx - aizl0.position.x, 2.0f) +
                           Math.pow((float)my - aizl0.position.y, 2.0f));
      if (d <= synapseRadius)
      {
         synapseWeightState = true;
         currentSynapse     = aizl0;
      }
      d = (float)Math.sqrt(Math.pow((float)mx - aizl1.position.x, 2.0f) +
                           Math.pow((float)my - aizl1.position.y, 2.0f));
      if (d <= synapseRadius)
      {
         synapseWeightState = true;
         currentSynapse     = aizl1;
      }
      d = (float)Math.sqrt(Math.pow((float)mx - aizr0.position.x, 2.0f) +
                           Math.pow((float)my - aizr0.position.y, 2.0f));
      if (d <= synapseRadius)
      {
         synapseWeightState = true;
         currentSynapse     = aizr0;
      }
      d = (float)Math.sqrt(Math.pow((float)mx - aizr1.position.x, 2.0f) +
                           Math.pow((float)my - aizr1.position.y, 2.0f));
      if (d <= synapseRadius)
      {
         synapseWeightState = true;
         currentSynapse     = aizr1;
      }
   }


   // Resume.
   public void resume()
   {
      updateWorker.resume();
      displayWorker.resume();
   }


   // Pause.
   public void pause()
   {
      updateWorker.pause();
      displayWorker.pause();
   }
}
