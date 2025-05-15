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

#ifndef _GST_UNDISTORT_H_
#define _GST_UNDISTORT_H_

#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>
#include <iostream>
#include <vector>

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
  INTERPOLATION_BILINEAR  = 1
}Interpolation;

struct _GstUndistort
{
  GstVideoFilter base_undistort;

  double balance = 0.0; // 0 = crop more, 1 = crop less
  double fov_scale = 1.0; // default
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

};

struct _GstUndistortClass
{
  GstVideoFilterClass base_undistort_class;
};

GType gst_undistort_get_type (void);

G_END_DECLS

#endif
