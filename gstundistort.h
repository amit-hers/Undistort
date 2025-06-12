/* GStreamer
 * Copyright (C) 2025 FIXME <fixme@example.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

 /**
  * EXAMPLE: ARM
  *            appsrc name=tracking_cam_to_rtp_appsrc !
              video/x-raw,framerate=40/1,width=640,height=480,format=GRAY8 !
              undistort fx=279.98435515  cx=290.6865358454301 fy=277.5687125242331 cy=240.1597738541476 d0=-0.006319990900085937 d1=0.00155893534296691 algo=0 fov=1.0 balance=0.5 !
              videoconvert !
              video/x-raw,format=NV12 !
              omxh264enc name=encoder interval-intraframes=60 target-bitrate=1500000 !
              video/x-h264,profile=main !
              h264parse !
              queue max-size-buffers=1 leaky=downstream !
              rtph264pay config-interval=1 name=pay0 pt=96 mtu=1200
  EXAMPLE: AMD
              gst-launch-1.0 --gst-plugin-path=/home/amither/Downloads/calibartion/build_amd/ rtspsrc location=rtsp://192.0.systemID.droneID:port/mount  !
              rtph264depay !
              h264parse !
              avdec_h264 !
              videoconvert !
              undistort fx=277.7090664510777 cx=290.6865358454301 fy=277.5687125242331 cy=240.1597738541476 d0=-0.006319990900085937 d1=0.00155893534296691 algo=0 fov=1.0 balance=0.0 !
              videoconvert !
              autovideosink sync=false

  */

#ifndef _GST_UNDISTORT_H_
#define _GST_UNDISTORT_H_

#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>
#include <iostream>
#include <vector>

#define USE_CV 1

#if defined(USE_CV)
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#endif

G_BEGIN_DECLS

#define GST_TYPE_UNDISTORT   (gst_undistort_get_type())
#define GST_UNDISTORT(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_UNDISTORT,GstUndistort))
#define GST_UNDISTORT_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_UNDISTORT,GstUndistortClass))
#define GST_IS_UNDISTORT(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_UNDISTORT))
#define GST_IS_UNDISTORT_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_UNDISTORT))

typedef struct _GstUndistort GstUndistort;
typedef struct _GstUndistortClass GstUndistortClass;


typedef enum Interpolation_t
{
  INTERPOLATION_K_NEAREST = 0,
  INTERPOLATION_BILINEAR  = 1,
  INTERPOLATION_BICUBIC   = 2,
  INTERPOLATION_OPENCV    = 3,
}Interpolation;

struct _GstUndistort
{
  GstVideoFilter base_undistort;

  gboolean crop = FALSE;
  gboolean initialized = FALSE;

  guint crop_sr = 0;
  guint crop_er = 0;
  guint crop_sc = 0;
  guint crop_ec = 0;

  float fx;
  float cx;
  float fy;
  float cy;
  float d0;
  float d1;
  float p1;
  float p2;

  float fov = 1.0f;
  float balance = 1.0f;

  std::vector<float> K;
  std::vector<float> D;
  std::vector<float> R;

  std::vector<float> Knew;

  std::vector<uint16_t> map_x;
  std::vector<uint16_t> map_y;

  int width;
  int height;
  int stride;
  int channels;
  Interpolation interpolation;
  
  #if defined(USE_CV)
  cv::Mat map1CV, map2CV;
  #endif
};

struct _GstUndistortClass
{
  GstVideoFilterClass base_undistort_class;
};

GType gst_undistort_get_type (void);

G_END_DECLS

#endif
