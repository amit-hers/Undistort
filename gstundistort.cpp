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
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
 */
/**
 * SECTION:element-gstundistort
 *
 * The undistort element does FIXME stuff.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v fakesrc ! undistort ! FIXME ! fakesink
 * ]|
 * FIXME Describe what the pipeline does.
 * </refsect2>
 */

       // undistort->fx = 277.7090664510777;
      // undistort->cx = 290.6865358454301;
      // undistort->fy = 277.5687125242331;
      // undistort->cy = 240.1597738541476;
      // undistort->d0 = -0.006319990900085937;
      // undistort->d1 = 0.00155893534296691;

/**
 *  gst-launch-1.0 --gst-plugin-path=/home/amither/Downloads/calibartion filesrc location=/home/amither/Downloads/2025-05-07_17-28-45.mp4 ! 
 * decodebin ! videoconvert ! video/x-raw,format=GRAY8 !
 *  undistort fx=277.7090664510777 cx=290.6865358454301 fy=277.5687125242331 cy=240.1597738541476 d0=-0.006319990900085937 d1=0.00155893534296691 algo=1
 *  ! videoconvert ! autovideosink
 */



#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>
#include "gstundistort.h"
#include <algorithm> // for std::min/max
#include <cmath>

GST_DEBUG_CATEGORY_STATIC (gst_undistort_debug_category);
#define GST_CAT_DEFAULT gst_undistort_debug_category

/* prototypes */


static void gst_undistort_set_property (GObject * object,
    guint property_id, const GValue * value, GParamSpec * pspec);
static void gst_undistort_get_property (GObject * object,
    guint property_id, GValue * value, GParamSpec * pspec);
static void gst_undistort_dispose (GObject * object);
static void gst_undistort_finalize (GObject * object);

static gboolean gst_undistort_start (GstBaseTransform * trans);
static gboolean gst_undistort_stop (GstBaseTransform * trans);
static gboolean gst_undistort_set_info (GstVideoFilter * filter, GstCaps * incaps,
    GstVideoInfo * in_info, GstCaps * outcaps, GstVideoInfo * out_info);
static GstFlowReturn gst_undistort_transform_frame (GstVideoFilter * filter,
    GstVideoFrame * inframe, GstVideoFrame * outframe);
static GstFlowReturn gst_undistort_transform_frame_ip (GstVideoFilter * filter,
    GstVideoFrame * frame);

enum
{
  PROP_0,
  PROP_START_ROW_CROP,
  PROP_END_ROW_CROP,
  PROP_START_COLUMN_CROP,
  PROP_END_COLUMN_CROP,
  PROP_FX,
  PROP_CX,
  PROP_FY,
  PROP_CY,
  PROP_D0,
  PROP_D1,
  PROP_ALGO,

};


/* pad templates */

/* FIXME: add/remove formats you can handle */
#define VIDEO_SRC_CAPS \
    GST_VIDEO_CAPS_MAKE("{ GRAY8 }")

/* FIXME: add/remove formats you can handle */
#define VIDEO_SINK_CAPS \
    GST_VIDEO_CAPS_MAKE("{ GRAY8 }")


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (GstUndistort, gst_undistort, GST_TYPE_VIDEO_FILTER,
  GST_DEBUG_CATEGORY_INIT (gst_undistort_debug_category, "undistort", 0,
  "debug category for undistort element"));

static void
gst_undistort_class_init (GstUndistortClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstBaseTransformClass *base_transform_class = GST_BASE_TRANSFORM_CLASS (klass);
  GstVideoFilterClass *video_filter_class = GST_VIDEO_FILTER_CLASS (klass);

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_pad_template (GST_ELEMENT_CLASS(klass),
      gst_pad_template_new ("src", GST_PAD_SRC, GST_PAD_ALWAYS,
        gst_caps_from_string (VIDEO_SRC_CAPS)));
  gst_element_class_add_pad_template (GST_ELEMENT_CLASS(klass),
      gst_pad_template_new ("sink", GST_PAD_SINK, GST_PAD_ALWAYS,
        gst_caps_from_string (VIDEO_SINK_CAPS)));

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS(klass),
      "FIXME Long name", "Generic", "FIXME Description",
      "FIXME <fixme@example.com>");

  gobject_class->set_property = gst_undistort_set_property;
  gobject_class->get_property = gst_undistort_get_property;
  gobject_class->dispose = gst_undistort_dispose;
  gobject_class->finalize = gst_undistort_finalize;
  base_transform_class->start = GST_DEBUG_FUNCPTR (gst_undistort_start);
  base_transform_class->stop = GST_DEBUG_FUNCPTR (gst_undistort_stop);
  video_filter_class->set_info = GST_DEBUG_FUNCPTR (gst_undistort_set_info);
  video_filter_class->transform_frame = GST_DEBUG_FUNCPTR (gst_undistort_transform_frame);
  video_filter_class->transform_frame_ip = GST_DEBUG_FUNCPTR (gst_undistort_transform_frame_ip);

  g_object_class_install_property (gobject_class, PROP_START_ROW_CROP,
  g_param_spec_uint ("crop-sr", "Crop Start Row",
  "Show all pixels (1), only valid ones (0) or something in between",
  0, 100000, 0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_END_ROW_CROP,
  g_param_spec_uint ("crop-er", "Pixels",
  "Show all pixels (1), only valid ones (0) or something in between",
  0, 100000, 0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_START_COLUMN_CROP,
  g_param_spec_uint ("crop-sc", "Pixels",
  "Show all pixels (1), only valid ones (0) or something in between",
  0, 100000, 0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_END_COLUMN_CROP,
  g_param_spec_uint ("crop-ec", "Pixels",
  "Show all pixels (1), only valid ones (0) or something in between",
  0, 100000, 0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_FX,
  g_param_spec_float ("fx", "Focal axis x",
  "Focal length in x (pixels)",
  0.0, 100000.0, 0.0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_FY,
  g_param_spec_float ("fy", "Focal axis y",
  "Focal length in y (pixels)",
  0.0, 100000.0, 0.0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_CX,
  g_param_spec_float ("cx", "Center x of the image",
  "Principal point x (pixels) x-coordinate of the optical center (usually near image center). |",
  0.0, 100000.0, 0.0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_CY,
  g_param_spec_float ("cy", "Center y of the image",
  "Principal point y (pixels)| y-coordinate of the optical center. |",
  0.0, 100000.0, 0.0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_D0,
  g_param_spec_float ("d0", "k1",
  "First radial distortion coefficient",
  -1.0, 1.0, 0.0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_D1,
  g_param_spec_float ("d1", "k2",
  "Second radial distortion coefficient",
  -1.0, 1.0, 0.0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_ALGO,
  g_param_spec_uint("algo", "Interpolation",
  "The Interpolation:\n"
  "0 - K nearest\n"
  "1 - Bilinear",
  0, 1, 0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

}

static void
gst_undistort_init (GstUndistort *undistort)
{
  undistort->R = {1,0,0, 0,1,0, 0,0,1}; // Identity rotation
}

void
gst_undistort_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  GstUndistort *undistort = GST_UNDISTORT (object);
  int valueZ;
  GST_DEBUG_OBJECT (undistort, "set_property");

  switch (property_id) {

    case PROP_START_ROW_CROP:
      undistort->crop_sr = g_value_get_uint(value);
      undistort->crop = TRUE;
      break;

    case PROP_END_ROW_CROP:
      undistort->crop_er = g_value_get_uint(value);
      undistort->crop = TRUE;
      break;

    case PROP_START_COLUMN_CROP:
      undistort->crop_sc = g_value_get_uint(value);
      undistort->crop = TRUE;
      break;

    case PROP_END_COLUMN_CROP:
      undistort->crop_ec = g_value_get_uint(value);
      undistort->crop = TRUE;
      break;

    case PROP_FX:
      undistort->fx = g_value_get_float(value);
      break;

    case PROP_FY:
      undistort->fy = g_value_get_float(value);
      break;

    case PROP_CX:
      undistort->cx = g_value_get_float(value);
      break;

    case PROP_CY:
      undistort->cy = g_value_get_float(value);
      break;
    
    case PROP_D0:
      undistort->d0 = g_value_get_float(value);
      break;

    case PROP_D1:
      undistort->d1 = g_value_get_float(value);
      break;
    
    case PROP_ALGO:
      valueZ = g_value_get_uint(value);
      undistort->interpolation = (Interpolation)valueZ;
      if (undistort->interpolation == INTERPOLATION_K_NEAREST)
      {
        g_print("Using INTERPOLATION_K_NEAREST interpolation %d\n", valueZ);
      }
      if (undistort->interpolation == INTERPOLATION_BILINEAR)
      {
        g_print("Using INTERPOLATION_BILINEAR interpolation\n");
      }
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_undistort_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  GstUndistort *undistort = GST_UNDISTORT (object);

  GST_DEBUG_OBJECT (undistort, "get_property");

  switch (property_id) {
    case PROP_START_ROW_CROP:
      g_value_set_uint(value, undistort->crop_sr);
      break;

    case PROP_END_ROW_CROP:
      g_value_set_uint(value, undistort->crop_er);
      break;

    case PROP_START_COLUMN_CROP:
      g_value_set_uint(value, undistort->crop_sc);
      break;

    case PROP_END_COLUMN_CROP:
      g_value_set_uint(value, undistort->crop_ec);
      break;

    case PROP_FX:
      g_value_set_float(value, undistort->fx);
      break;

    case PROP_FY:
    g_value_set_float(value, undistort->fy);
      break;

    case PROP_CX:
    g_value_set_float(value, undistort->cx);
      break;

    case PROP_CY:
    g_value_set_float(value, undistort->cy);
      break;
    
    case PROP_D0:
    g_value_set_float(value, undistort->d0);
      break;

    case PROP_D1:
    g_value_set_float(value, undistort->d1);
      break;
    
    case PROP_ALGO:
      g_value_set_uint(value, undistort->interpolation);
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_undistort_dispose (GObject * object)
{
  GstUndistort *undistort = GST_UNDISTORT (object);

  GST_DEBUG_OBJECT (undistort, "dispose");

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (gst_undistort_parent_class)->dispose (object);
}

void
gst_undistort_finalize (GObject * object)
{
  GstUndistort *undistort = GST_UNDISTORT (object);

  GST_DEBUG_OBJECT (undistort, "finalize");

  /* clean up object here */

  G_OBJECT_CLASS (gst_undistort_parent_class)->finalize (object);
}

static gboolean
gst_undistort_start (GstBaseTransform * trans)
{
  GstUndistort *undistort = GST_UNDISTORT (trans);

  GST_DEBUG_OBJECT (undistort, "start");

  return TRUE;
}

static gboolean
gst_undistort_stop (GstBaseTransform * trans)
{
  GstUndistort *undistort = GST_UNDISTORT (trans);

  GST_DEBUG_OBJECT (undistort, "stop");

  return TRUE;
}

static gboolean
gst_undistort_set_info (GstVideoFilter * filter, GstCaps * incaps,
    GstVideoInfo * in_info, GstCaps * outcaps, GstVideoInfo * out_info)
{
  GstUndistort *undistort = GST_UNDISTORT (filter);

  GST_DEBUG_OBJECT (undistort, "set_info");

  return TRUE;
}
void estimateNewCameraMatrix(const std::vector<float>& K, std::vector<float>& Knew, int width, int height, float balance = 0.0f, float fov_scale = 1.0f)
{
    float fx = K[0];
    float fy = K[4];
    float cx = K[2];
    float cy = K[5];

    fx *= fov_scale;
    fy *= fov_scale;

    float cx_new = (1.0f - balance) * (width / 2.0f) + balance * cx;
    float cy_new = (1.0f - balance) * (height / 2.0f) + balance * cy;

    Knew.resize(9);
    Knew[0] = fx;  Knew[1] = 0;   Knew[2] = cx_new;
    Knew[3] = 0;   Knew[4] = fy;  Knew[5] = cy_new;
    Knew[6] = 0;   Knew[7] = 0;   Knew[8] = 1;
}


void initFisheyeUndistortRectifyMap(
    const std::vector<float>& K, const std::vector<float>& D,
    const std::vector<float>& R, const std::vector<float>& Knew,
    int width, int height,
    std::vector<uint16_t>& map_x, std::vector<uint16_t>& map_y)
{
    map_x.resize(width * height);
    map_y.resize(width * height);

    float fx_new = Knew[0];
    float fy_new = Knew[4];
    float cx_new = Knew[2];
    float cy_new = Knew[5];

    float fx = K[0];
    float fy = K[4];
    float cx = K[2];
    float cy = K[5];

    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            float x = (u - cx_new) / fx_new;
            float y = (v - cy_new) / fy_new;

            float X = R[0]*x + R[1]*y + R[2];
            float Y = R[3]*x + R[4]*y + R[5];
            float Z = R[6]*x + R[7]*y + R[8];

            x = X / Z;
            y = Y / Z;

            float r = std::sqrt(x * x + y * y);
            float theta = std::atan(r);
            float theta2 = theta * theta;
            float theta_d = theta * (1 + D[0]*theta2 + D[1]*theta2*theta2 +
                                     D[2]*theta2*theta2*theta2 + D[3]*theta2*theta2*theta2*theta2);

            float scale = (r > 1e-8f) ? (theta_d / r) : 1.0f;

            float xd = x * scale;
            float yd = y * scale;

            map_x[v * width + u] = fx * xd + cx;
            map_y[v * width + u] = fy * yd + cy;
        }
    }
}

void resize_and_remap_bilinear(
  const uint8_t* src, int src_w, int src_h, int src_stride,
  uint8_t* dst, int dst_w, int dst_h, int dst_stride,
  const std::vector<uint16_t>& map_x, const std::vector<uint16_t>& map_y,
  int resized_w, int resized_h, int channels)
{
    float scale_x = static_cast<float>(src_w) / static_cast<float>(resized_w);
    float scale_y = static_cast<float>(src_h) / static_cast<float>(resized_h);

    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; ++x) {
          uint16_t fx = map_x[y * dst_w + x];
          uint16_t fy = map_y[y * dst_w + x];

            float sx = fx * scale_x;
            float sy = fy * scale_y;

            int x0 = static_cast<int>(sx);
            int y0 = static_cast<int>(sy);
            float dx = sx - x0;
            float dy = sy - y0;

            for (int c = 0; c < channels; ++c) {
                uint8_t value = 0;

                if (x0 >= 0 && x0 + 1 < src_w && y0 >= 0 && y0 + 1 < src_h) {
                    uint8_t p00 = src[y0 * src_stride + x0 * channels + c];
                    uint8_t p10 = src[y0 * src_stride + (x0 + 1) * channels + c];
                    uint8_t p01 = src[(y0 + 1) * src_stride + x0 * channels + c];
                    uint8_t p11 = src[(y0 + 1) * src_stride + (x0 + 1) * channels + c];

                    float val = (1 - dx) * (1 - dy) * p00 +
                                dx * (1 - dy) * p10 +
                                (1 - dx) * dy * p01 +
                                dx * dy * p11;

                    value = static_cast<uint8_t>(val + 0.5f);
                }
                int rr = y * dst_stride + x * channels + c;

                dst[rr] = value;
            }
        }
    }
}

#if defined(__aarch64__)

void resize_and_remap_bilinear_neon(
  const uint8_t* __restrict src, int src_w, int src_h, int src_stride,
  uint8_t* __restrict dst, int dst_w, int dst_h, int dst_stride,
  const std::vector<uint16_t>& map_x, const std::vector<uint16_t>& map_y,
  int resized_w, int resized_h, int channels)
{
    const float scale_x = static_cast<float>(src_w) / resized_w;
    const float scale_y = static_cast<float>(src_h) / resized_h;

    if (channels != 1) {
        printf("Only GRAY8 supported.\\n");
        return;
    }

    for (int y = 0; y < dst_h; ++y) {
        int map_row_offset = y * dst_w;
        int dst_row_offset = y * dst_stride;

        for (int x = 0; x < dst_w; x += 8) {
            // Load 8 raw pixel coordinates from uint16_t maps
            uint16x8_t mapx_u16 = vld1q_u16(&map_x[map_row_offset + x]);
            uint16x8_t mapy_u16 = vld1q_u16(&map_y[map_row_offset + x]);

            float32x4_t fx_low = vcvtq_f32_u32(vmovl_u16(vget_low_u16(mapx_u16)));
            float32x4_t fx_high = vcvtq_f32_u32(vmovl_u16(vget_high_u16(mapx_u16)));
            float32x4_t fy_low = vcvtq_f32_u32(vmovl_u16(vget_low_u16(mapy_u16)));
            float32x4_t fy_high = vcvtq_f32_u32(vmovl_u16(vget_high_u16(mapy_u16)));

            // Apply scaling (optional, depending on remap math)
            fx_low = vmulq_n_f32(fx_low, scale_x);
            fx_high = vmulq_n_f32(fx_high, scale_x);
            fy_low = vmulq_n_f32(fy_low, scale_y);
            fy_high = vmulq_n_f32(fy_high, scale_y);

            int32x4_t x0_low = vcvtq_s32_f32(fx_low);
            int32x4_t x0_high = vcvtq_s32_f32(fx_high);
            int32x4_t y0_low = vcvtq_s32_f32(fy_low);
            int32x4_t y0_high = vcvtq_s32_f32(fy_high);

            float32x4_t dx_low = vsubq_f32(fx_low, vcvtq_f32_s32(x0_low));
            float32x4_t dx_high = vsubq_f32(fx_high, vcvtq_f32_s32(x0_high));
            float32x4_t dy_low = vsubq_f32(fy_low, vcvtq_f32_s32(y0_low));
            float32x4_t dy_high = vsubq_f32(fy_high, vcvtq_f32_s32(y0_high));

            for (int i = 0; i < 4; ++i) {
                int xi = x + i;
                int x0 = vgetq_lane_s32(x0_low, i);
                int y0 = vgetq_lane_s32(y0_low, i);
                float dx = vgetq_lane_f32(dx_low, i);
                float dy = vgetq_lane_f32(dy_low, i);

                uint8_t value = 0;
                if (x0 >= 0 && x0 + 1 < src_w && y0 >= 0 && y0 + 1 < src_h) {
                    uint8_t p00 = src[y0 * src_stride + x0];
                    uint8_t p10 = src[y0 * src_stride + x0 + 1];
                    uint8_t p01 = src[(y0 + 1) * src_stride + x0];
                    uint8_t p11 = src[(y0 + 1) * src_stride + x0 + 1];

                    float val = (1 - dx) * (1 - dy) * p00 +
                                dx * (1 - dy) * p10 +
                                (1 - dx) * dy * p01 +
                                dx * dy * p11;

                    value = static_cast<uint8_t>(val + 0.5f);
                }
                dst[dst_row_offset + xi] = value;
            }

            for (int i = 0; i < 4; ++i) {
                int xi = x + 4 + i;
                int x0 = vgetq_lane_s32(x0_high, i);
                int y0 = vgetq_lane_s32(y0_high, i);
                float dx = vgetq_lane_f32(dx_high, i);
                float dy = vgetq_lane_f32(dy_high, i);

                uint8_t value = 0;
                if (x0 >= 0 && x0 + 1 < src_w && y0 >= 0 && y0 + 1 < src_h) {
                    uint8_t p00 = src[y0 * src_stride + x0];
                    uint8_t p10 = src[y0 * src_stride + x0 + 1];
                    uint8_t p01 = src[(y0 + 1) * src_stride + x0];
                    uint8_t p11 = src[(y0 + 1) * src_stride + x0 + 1];

                    float val = (1 - dx) * (1 - dy) * p00 +
                                dx * (1 - dy) * p10 +
                                (1 - dx) * dy * p01 +
                                dx * dy * p11;

                    value = static_cast<uint8_t>(val + 0.5f);
                }
                dst[dst_row_offset + xi] = value;
            }
        }
    }
}
#endif


void resize_and_remap_nearest(
  const uint8_t* src, int src_w, int src_h, int src_stride,
  uint8_t* dst, int dst_w, int dst_h, int dst_stride,
  const std::vector<uint16_t>& map_x, const std::vector<uint16_t>& map_y,
  int resized_w, int resized_h, int channels)
{
  // dst.resize(dst_h * dst_stride);  // Ensure output buffer is big enough

  float scale_x = static_cast<float>(src_w) / static_cast<float>(resized_w);
  float scale_y = static_cast<float>(src_h) / static_cast<float>(resized_h);

  for (int y = 0; y < dst_h; ++y) {
      for (int x = 0; x < dst_w; ++x) {
          uint16_t fx = map_x[y * dst_w + x];
          uint16_t fy = map_y[y * dst_w + x];

          // Apply scale to map from normalized rectified to source image
          int sx = static_cast<int>(fx * scale_x + 0.5f);
          int sy = static_cast<int>(fy * scale_y + 0.5f);

          for (int c = 0; c < channels; ++c) {
              uint8_t value = 0;

              if (sx >= 0 && sx < src_w && sy >= 0 && sy < src_h) {
                  value = src[sy * src_stride + sx * channels + c];
              }

              dst[y * dst_stride + x * channels + c] = value;
          }
      }
  }
}

#if defined(__aarch64__)
void resize_and_remap_nearest_neon(
  const uint8_t* __restrict src, int src_w, int src_h, int src_stride,
  uint8_t* __restrict dst, int dst_w, int dst_h, int dst_stride,
  const std::vector<uint16_t>& map_x, const std::vector<uint16_t>& map_y,
  int /*resized_w*/, int /*resized_h*/, int channels)
{
    if (channels != 1) {
        printf("Only GRAY8 supported in NEON version.\n");
        return;
    }

    for (int y = 0; y < dst_h; ++y) {
        int map_row_offset = y * dst_w;
        int dst_row_offset = y * dst_stride;

        for (int x = 0; x < dst_w; x += 8) {
            // Load 8 fixed-point values (assume already in pixel coords)
            uint16x8_t fx_fixed = vld1q_u16(&map_x[map_row_offset + x]);
            uint16x8_t fy_fixed = vld1q_u16(&map_y[map_row_offset + x]);

            // Convert to int32 (assume integer values, no scaling)
            int32x4_t sx_low = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(fx_fixed)));
            int32x4_t sx_high = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(fx_fixed)));
            int32x4_t sy_low = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(fy_fixed)));
            int32x4_t sy_high = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(fy_fixed)));

            // Per-pixel fetch (no gather support in NEON)
            for (int i = 0; i < 4; ++i) {
                int xi = x + i;
                int sx = vgetq_lane_s32(sx_low, i);
                int sy = vgetq_lane_s32(sy_low, i);

                // Clamp to avoid black frames
                sx = std::min(std::max(sx, 0), src_w - 1);
                sy = std::min(std::max(sy, 0), src_h - 1);

                dst[dst_row_offset + xi] = src[sy * src_stride + sx];
            }

            for (int i = 0; i < 4; ++i) {
                int xi = x + 4 + i;
                int sx = vgetq_lane_s32(sx_high, i);
                int sy = vgetq_lane_s32(sy_high, i);

                sx = std::min(std::max(sx, 0), src_w - 1);
                sy = std::min(std::max(sy, 0), src_h - 1);

                dst[dst_row_offset + xi] = src[sy * src_stride + sx];
            }
        }
    }
}
#endif

// // /* transform */
static GstFlowReturn
gst_undistort_transform_frame (GstVideoFilter * filter, GstVideoFrame * inframe, GstVideoFrame * outframe)
{
  GstUndistort *undistortClass = GST_UNDISTORT (filter);

  GST_DEBUG_OBJECT (undistortClass, "transform_frame");

  static int i=0;

  GstMapInfo map_info;
  GstMapInfo info;

  gst_buffer_map ((inframe->buffer), &map_info, GST_MAP_READ);
  gst_buffer_map((outframe->buffer), &info, GST_MAP_WRITE);

  if (i ==0)
  {
    undistortClass->width = GST_VIDEO_FRAME_WIDTH(inframe);
    undistortClass->height = GST_VIDEO_FRAME_HEIGHT(inframe);
    undistortClass->channels = 1;
    undistortClass->stride =  undistortClass->width *  undistortClass->channels;

    
    undistortClass->K = {undistortClass->fx, 0.0, undistortClass->cx, 0.0, undistortClass->fy, undistortClass->cy, 0.0, 0.0, 1.0};
    undistortClass->D = {undistortClass->d0, undistortClass->d1, 0.0, 0.0}; // 4 distortion coeffs

    estimateNewCameraMatrix(undistortClass->K, undistortClass->Knew, undistortClass->width, undistortClass->height, 0.0f, 1.0f);
    initFisheyeUndistortRectifyMap(undistortClass->K, undistortClass->D, undistortClass->R, undistortClass->Knew, undistortClass->width, undistortClass->height, undistortClass->map_x, undistortClass->map_y);
    std::cout << "INIT_MAPS on the fly" << std::endl;
  }
  if (i > 0)
  {
    if (undistortClass->interpolation == INTERPOLATION_K_NEAREST)
    #if defined(__aarch64__)
    resize_and_remap_nearest_neon(
      map_info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      undistortClass->map_x, undistortClass->map_y,
      undistortClass->width, undistortClass->height, undistortClass->channels);
    #else
    resize_and_remap_nearest(
      map_info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      undistortClass->map_x, undistortClass->map_y,
      undistortClass->width, undistortClass->height, undistortClass->channels);
    #endif
    else if (undistortClass->interpolation == INTERPOLATION_BILINEAR)
    #if defined(__aarch64__)
    resize_and_remap_bilinear_neon(
      map_info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      undistortClass->map_x, undistortClass->map_y,
      undistortClass->width, undistortClass->height, undistortClass->channels);
    #else
    resize_and_remap_bilinear(
      map_info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      undistortClass->map_x, undistortClass->map_y,
      undistortClass->width, undistortClass->height, undistortClass->channels);
    #endif
  }

  i = i+1;
  
  gst_buffer_unmap ((inframe->buffer), &map_info);
  gst_buffer_unmap ((outframe->buffer), &info);

  return GST_FLOW_OK;
}

static GstFlowReturn
gst_undistort_transform_frame_ip (GstVideoFilter * filter, GstVideoFrame * frame)
{
  GstUndistort *undistortClass = GST_UNDISTORT (filter);

  GST_DEBUG_OBJECT (undistortClass, "transform_frame");

  return GST_FLOW_OK;
}

static gboolean
plugin_init (GstPlugin * plugin)
{

  /* FIXME Remember to set the rank if it's an element that is meant
     to be autoplugged by decodebin. */
  return gst_element_register (plugin, "undistort", GST_RANK_NONE,
      GST_TYPE_UNDISTORT);
}

/* FIXME: these are normally defined by the GStreamer build system.
   If you are creating an element to be included in gst-plugins-*,
   remove these, as they're always defined.  Otherwise, edit as
   appropriate for your external plugin package. */
#ifndef VERSION
#define VERSION "1.0.0"
#endif
#ifndef PACKAGE
#define PACKAGE "UNDISTORT"
#endif
#ifndef PACKAGE_NAME
#define PACKAGE_NAME "UNDISTORT_package_name"
#endif
#ifndef GST_PACKAGE_ORIGIN
#define GST_PACKAGE_ORIGIN "http://xtend.me/"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    undistort,
    "FIXME plugin description",
    plugin_init, VERSION, "LGPL", PACKAGE_NAME, GST_PACKAGE_ORIGIN)

