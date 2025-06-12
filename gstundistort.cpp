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
  gst-launch-1.0 --gst-plugin-path=/home/amither/Downloads/calibartion/build/ filesrc location=/home/amither/Downloads/calibartion/2025-05-06_14-21-32.mp4 ! \
  decodebin ! videoconvert ! video/x-raw,format=GRAY8 ! \
  undistort fx=277.7090664510777 cx=290.6865358454301 fy=277.5687125242331 cy=240.1597738541476 d0=-0.006319990900085937 d1=0.00155893534296691 algo=1 balance=1.0 fov=1.0 \
  ! videoconvert ! autovideosink
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
#include <array>
#include <limits>
#include <cstdint>  // or use <stdint.h>

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
  PROP_P1,
  PROP_P2,
  PROP_ALGO,
  PROP_FOV,
  PROP_BALANCE,
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

  g_object_class_install_property (gobject_class, PROP_FOV,
  g_param_spec_float ("fov", "Field of view",
  "Field of view can simulate zooming (e.g., 0.8 for zoom-in)",
  0.0, 100000.0, 0.0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_BALANCE,
  g_param_spec_float ("balance", "Balance fot init",
  "(0)  principal point moves to image center balance\n(1) keeps original principal point",
  0.0, 100000.0, 0.0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

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

  g_object_class_install_property (gobject_class, PROP_P1,
  g_param_spec_float ("p1", "k1",
  "First radial distortion coefficient",
  -1.0, 1.0, 0.0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_P2,
  g_param_spec_float ("p2", "k2",
  "Second radial distortion coefficient",
  -1.0, 1.0, 0.0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_ALGO,
  g_param_spec_uint("algo", "Interpolation",
  "The Interpolation:\n"
  "0 - K nearest\n"
  "1 - Bilinear"
  "2 - Bicubic "
  "3 - Opencv",
  0, 3, 0,
  (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

}

static void
gst_undistort_init (GstUndistort *undistort)
{
  undistort->R = {1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0}; // Identity rotation
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

    case PROP_P1:
      undistort->p1 = g_value_get_float(value);
      break;

    case PROP_P2:
      undistort->p2 = g_value_get_float(value);
      break;

    case PROP_BALANCE:
      undistort->balance = g_value_get_float(value);
      break;

    case PROP_FOV:
      undistort->fov = g_value_get_float(value);
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
      if (undistort->interpolation == INTERPOLATION_BICUBIC)
      {
        g_print("Using INTERPOLATION_BICUBIC interpolation\n");
      }
      if (undistort->interpolation == INTERPOLATION_OPENCV)
      {
        g_print("Using OPENCV_CHOOSE interpolation\n");
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

    case PROP_P1:
    g_value_set_float(value, undistort->p1);
      break;

    case PROP_P2:
    g_value_set_float(value, undistort->p2);
      break;

    case PROP_BALANCE:
    g_value_set_float(value, undistort->balance);
      break;

    case PROP_FOV:
    g_value_set_float(value, undistort->fov);
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
  std::cout << "Dispose the Undistort" << std::endl;

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (gst_undistort_parent_class)->dispose (object);
}

void
gst_undistort_finalize (GObject * object)
{
  GstUndistort *undistort = GST_UNDISTORT (object);

  GST_DEBUG_OBJECT (undistort, "finalize");

  /* clean up object here */
  std::cout << "finalize the Undistort" << std::endl;

  G_OBJECT_CLASS (gst_undistort_parent_class)->finalize (object);
}

static gboolean
gst_undistort_start (GstBaseTransform * trans)
{
  GstUndistort *undistort = GST_UNDISTORT (trans);

  GST_DEBUG_OBJECT (undistort, "start");
  std::cout << "Start the Undistort" << std::endl;

  return TRUE;
}

static gboolean
gst_undistort_stop (GstBaseTransform * trans)
{
  GstUndistort *undistort = GST_UNDISTORT (trans);

  GST_DEBUG_OBJECT (undistort, "stop");
  undistort->initialized = FALSE;
  std::cout << "Stop the Undistort" << std::endl;

  return TRUE;
}


template <typename T>
inline T clamp(T val, T lo, T hi) {
    return std::max(lo, std::min(val, hi));
}

// void convert_cv_maps_to_uint16(const cv::Mat& map1, std::vector<uint16_t>& map_x, std::vector<uint16_t>& map_y) {
//     map_x.resize(map1.rows * map1.cols);
//     map_y.resize(map1.rows * map1.cols);

//     for (int y = 0; y < map1.rows; ++y) {
//         for (int x = 0; x < map1.cols; ++x) {
//             const cv::Vec2s& pt = map1.at<cv::Vec2s>(y, x);
//             map_x[y * map1.cols + x] = static_cast<uint16_t>(pt[0]);
//             map_y[y * map1.cols + x] = static_cast<uint16_t>(pt[1]);
//         }
//     }
// }

#if defined(USE_CV)

void convert_cv_maps_to_uint16(const cv::Mat& map1, std::vector<uint16_t>& map_x, std::vector<uint16_t>& map_y) {
    int rows = map1.rows;
    int cols = map1.cols;
    map_x.resize(rows * cols);
    map_y.resize(rows * cols);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            const cv::Vec2s& pt = map1.at<cv::Vec2s>(y, x);

            // Flip coordinates (180° rotate)
            int flipped_index = (rows - 1 - y) * cols + (cols - 1 - x);
            map_x[flipped_index] = static_cast<uint16_t>(pt[0]);
            map_y[flipped_index] = static_cast<uint16_t>(pt[1]);
        }
    }
}


void compute_undistort_maps_from_opencv(
    GstUndistort *undistort,
    const std::vector<float>& K,
    const std::vector<float>& D,
    const std::vector<float>& R,
    std::vector<float>& Knew,
    int width,
    int height,
    float balance,
    float fov_scale,
    std::vector<uint16_t>& map_x,
    std::vector<uint16_t>& map_y) {


    std::cout << "Input parameters:" << std::endl;
    std::cout << "K: ";
    for (auto v : K) std::cout << v << " ";
    std::cout << "\nD: ";
    for (auto v : D) std::cout << v << " ";
    std::cout << "\nR: ";
    for (auto v : R) std::cout << v << " ";
    std::cout << "\nWidth: " << width << ", Height: " << height << std::endl;
    std::cout << "Balance: " << balance << ", FOV Scale: " << fov_scale << std::endl;

    cv::Mat K_cv(3, 3, CV_32F, const_cast<float*>(K.data()));
    cv::Mat D_cv(4, 1, CV_32F, const_cast<float*>(D.data()));
    cv::Mat R_cv(3, 3, CV_32F, const_cast<float*>(R.data()));
    cv::Mat Knew_cv;
    cv::Size imageSize(width, height);

    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_cv, D_cv, imageSize, R_cv, Knew_cv, balance, imageSize, fov_scale);

    Knew.assign((float*)Knew_cv.datastart, (float*)Knew_cv.dataend);
    std::cout << "Knew: ";
    for (auto v : Knew) std::cout << v << " ";
    std::cout << std::endl;
    cv::fisheye::initUndistortRectifyMap(K_cv, D_cv, R_cv, Knew_cv, imageSize, CV_16SC2, undistort->map1CV, undistort->map2CV);

    convert_cv_maps_to_uint16(undistort->map1CV, map_x, map_y);
}

#endif

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

    std::cout << "Knew:\n" << "(" << Knew[0] << "," << Knew[1] << "," <<  Knew[2] << "," << Knew[3] << "," << Knew[4] << "," << Knew[5] << "," << Knew[6] << "," << Knew[7] << "," << Knew[8] << "," << ")" << std::endl;
}

void undistortFisheyePoint(const std::array<float, 2>& p,
                           const std::vector<float>& K,
                           const std::vector<float>& D,
                           const std::vector<float>& R,
                           std::array<float, 2>& out,
                           float epsilon = 1e-9f,
                           int max_iter = 10) {
    float fx = K[0], fy = K[4], cx = K[2], cy = K[5];

    // Convert to normalized coordinates
    double x = (p[0] - cx) / fx;
    double y = (p[1] - cy) / fy;
    double theta_d = std::sqrt(x * x + y * y);

    if (theta_d < 1e-8) {
        out[0] = static_cast<float>(x);
        out[1] = static_cast<float>(y);
        return;
    }

    // Distortion coefficients
    double k1 = D[0], k2 = D[1], k3 = D[2], k4 = D[3];

    // Iteratively solve for theta using Newton-Raphson
    double theta = theta_d;
    bool converged = false;
    for (int i = 0; i < max_iter; ++i) {
        double t2 = theta * theta;
        double t4 = t2 * t2;
        double t6 = t4 * t2;
        double t8 = t6 * t2;

        // double theta_distorted = theta * (1 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8);
        double d_theta = (theta * (1 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8) - theta_d) /
                         (1 + 3 * k1 * t2 + 5 * k2 * t4 + 7 * k3 * t6 + 9 * k4 * t8);

        theta -= d_theta;

        if (std::abs(d_theta) < epsilon) {
            converged = true;
            break;
        }
    }

    if (!converged) {
        out[0] = out[1] = -1e6f; // fallback
        return;
    }

    double scale = std::tan(theta) / theta_d;
    double xu = x * scale;
    double yu = y * scale;

    // Apply rotation
    double x_r = R[0] * xu + R[1] * yu + R[2];
    double y_r = R[3] * xu + R[4] * yu + R[5];
    double z_r = R[6] * xu + R[7] * yu + R[8];

    out[0] = static_cast<float>(x_r / z_r);
    out[1] = static_cast<float>(y_r / z_r);
}

void estimateNewCameraMatrixForUndistortRectify(
    const std::vector<float>& K,         // 3x3 camera matrix
    const std::vector<float>& D,         // 4x1 distortion coefficients
    const std::vector<float>& R,         // 3x3 rectification rotation
    std::vector<float>& Knew,            // 3x3 new intrinsic output
    float width,
    float height,
    float balance = 0.0f,
    float fov_scale = 1.0f
) {
    using Vec2 = std::array<float, 2>;

    std::array<Vec2, 4> corners = {{
        {width / 2.0f, 0},
        {width, height / 2.0f},
        {width / 2.0f, height},
        {0, height / 2.0f}
    }};

    std::array<Vec2, 4> undistorted;
    Vec2 center = {0, 0};

    for (int i = 0; i < 4; ++i) {
        undistortFisheyePoint(corners[i], K, D, R, undistorted[i]);
        center[0] += undistorted[i][0];
        center[1] += undistorted[i][1];
    }

    center[0] /= 4.0f;
    center[1] /= 4.0f;

    float aspect = K[0] / K[4];

    // scale y for aspect ratio matching
    for (auto& pt : undistorted) pt[1] *= aspect;
    center[1] *= aspect;

    float min_x = std::numeric_limits<float>::max(), max_x = -min_x;
    float min_y = min_x, max_y = -min_x;

    for (const auto& pt : undistorted) {
        min_x = std::min(min_x, pt[0]);
        max_x = std::max(max_x, pt[0]);
        min_y = std::min(min_y, pt[1]);
        max_y = std::max(max_y, pt[1]);
    }

    float f1 = 0.5f * width / (center[0] - min_x);
    float f2 = 0.5f * width / (max_x - center[0]);
    float f3 = 0.5f * height * aspect / (center[1] - min_y);
    float f4 = 0.5f * height * aspect / (max_y - center[1]);

    float fmin = std::min({f1, f2, f3, f4});
    float fmax = std::max({f1, f2, f3, f4});

    float f = (1.0f - balance) * fmin + balance * fmax;
    f *= (fov_scale > 0.0f) ? (1.0f / fov_scale) : 1.0f;

    float fx = f;
    float fy = f / aspect;
    float cx = -center[0] * f + width * 0.5f;
    float cy = (-center[1] * f + height * aspect * 0.5f) / aspect;

    Knew.resize(9);
    Knew[0] = fx;  Knew[1] = 0.0f; Knew[2] = cx;
    Knew[3] = 0.0f; Knew[4] = fy;  Knew[5] = cy;
    Knew[6] = 0.0f; Knew[7] = 0.0f; Knew[8] = 1.0f;

    std::cout << "Knew:\n(" << Knew[0] << ", " << Knew[1] << ", " << Knew[2] << ", "
              << Knew[3] << ", " << Knew[4] << ", " << Knew[5] << ", "
              << Knew[6] << ", " << Knew[7] << ", " << Knew[8] << ")\n";
}

void initFisheyeUndistortRectifyMap(
    const std::vector<float>& K, const std::vector<float>& D,
    const std::vector<float>& R, const std::vector<float>& Knew,
    int width, int height,
    std::vector<uint16_t>& map_x, std::vector<uint16_t>& map_y,
    float balance = 0.0f,
    float fov_scale = 1.0f)
{
    map_x.resize(width * height);
    map_y.resize(width * height);

    float fx_new = Knew[0] * fov_scale;
    float fy_new = Knew[4] * fov_scale;
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

            // float mapped_x = fx * xd + cx;
            // float mapped_y = fy * yd + cy;
            map_x[v * width + u] = static_cast<uint16_t>(clamp(fx * xd + cx, 0.0f, static_cast<float>(width - 1)));
            map_y[v * width + u] = static_cast<uint16_t>(clamp(fy * yd + cy, 0.0f, static_cast<float>(height - 1)));
        }
    }
}

void apply_unsharp_mask(uint8_t* data, int w, int h, int stride) {
  std::vector<uint8_t> tmp(data, data + h * stride);
  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      int idx = y * stride + x;
      int val = 5 * tmp[idx]
              - tmp[idx - 1]
              - tmp[idx + 1]
              - tmp[idx - stride]
              - tmp[idx + stride];
      data[idx] = clamp(val, 0, 255);
    }
  }
}

float cubic_weight(float x) {
    x = std::fabs(x);
    if (x <= 1.0f)
        return (1.5f * x - 2.5f) * x * x + 1.0f;
    else if (x < 2.0f)
        return ((-0.5f * x + 2.5f) * x - 4.0f) * x + 2.0f;
    else
        return 0.0f;
}

void resize_and_remap_bicubic(
    const uint8_t* src, int src_w, int src_h, int src_stride,
    uint8_t* dst, int dst_w, int dst_h, int dst_stride,
    const std::vector<uint16_t>& map_x, const std::vector<uint16_t>& map_y,
    int, int, int channels)
{
    if (channels != 1) {
        printf("Only GRAY8 supported in bicubic.\n");
        return;
    }

    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; ++x) {
            float fx = static_cast<float>(map_x[y * dst_w + x]);
            float fy = static_cast<float>(map_y[y * dst_w + x]);

            // Clamp within valid region for 4x4 window
            fx = clamp(fx, 1.0f, static_cast<float>(src_w - 3));
            fy = clamp(fy, 1.0f, static_cast<float>(src_h - 3));

            int x_int = static_cast<int>(fx);
            int y_int = static_cast<int>(fy);
            float dx = fx - x_int;
            float dy = fy - y_int;

            float result = 0.0f;
            for (int m = -1; m <= 2; ++m) {
                float wy = cubic_weight(m - dy);
                int yy = clamp(y_int + m, 0, src_h - 1);
                for (int n = -1; n <= 2; ++n) {
                    float wx = cubic_weight(n - dx);
                    int xx = clamp(x_int + n, 0, src_w - 1);
                    result += wx * wy * src[yy * src_stride + xx];
                }
            }

            result = clamp(result, 0.0f, 255.0f);
            dst[y * dst_stride + x] = static_cast<uint8_t>(result + 0.5f);
        }
    }
}

void resize_and_remap_bilinear(
  const uint8_t* src, int src_w, int src_h, int src_stride,
  uint8_t* dst, int dst_w, int dst_h, int dst_stride,
  const std::vector<uint16_t>& map_x, const std::vector<uint16_t>& map_y,
  int, int, int channels)
{
  for (int y = 0; y < dst_h; ++y) {
    for (int x = 0; x < dst_w; ++x) {
      int idx = y * dst_w + x;

      float sx = static_cast<float>(map_x[idx]);
      float sy = static_cast<float>(map_y[idx]);

      sx = clamp(sx, 0.0f, (float)(src_w - 2));
      sy = clamp(sy, 0.0f, (float)(src_h - 2));

      int x0 = static_cast<int>(sx);
      int y0 = static_cast<int>(sy);
      float dx = sx - x0;
      float dy = sy - y0;

      for (int c = 0; c < channels; ++c) {
        uint8_t p00 = src[y0 * src_stride + x0 * channels + c];
        uint8_t p10 = src[y0 * src_stride + (x0 + 1) * channels + c];
        uint8_t p01 = src[(y0 + 1) * src_stride + x0 * channels + c];
        uint8_t p11 = src[(y0 + 1) * src_stride + (x0 + 1) * channels + c];

        float val = (1 - dx) * (1 - dy) * p00 +
                    dx * (1 - dy) * p10 +
                    (1 - dx) * dy * p01 +
                    dx * dy * p11;

        dst[y * dst_stride + x * channels + c] = static_cast<uint8_t>(std::round(val));
      }
    }
  }
}

#if defined(__aarch64__)
#include <arm_neon.h>
#include <cmath>
#include <algorithm>

static inline uint8_t sharpen(uint8_t center, uint8_t left, uint8_t right, uint8_t top, uint8_t bottom) {
    int result = 5 * center - left - right - top - bottom;
    return static_cast<uint8_t>(clamp(result, 0, 255));
}

// Existing bilinear function remains here...

// void resize_and_remap_nearest_neon(
//   const uint8_t* __restrict src, uint src_w, uint src_h, int src_stride,
//   uint8_t* __restrict dst, int dst_w, int dst_h, int dst_stride,
//   const std::vector<uint16_t>& map_x, const std::vector<uint16_t>& map_y,
//   int /*resized_w*/, int /*resized_h*/, int channels)
// {
//     if (channels != 1) {
//         printf("Only GRAY8 supported in NEON version.\n");
//         return;
//     }

//     for (int y = 0; y < dst_h; ++y) {
//         int map_row_offset = y * dst_w;
//         int dst_row_offset = y * dst_stride;

//         for (int x = 0; x < dst_w; x += 8) {
//             uint16x8_t fx_fixed = vld1q_u16(&map_x[map_row_offset + x]);
//             uint16x8_t fy_fixed = vld1q_u16(&map_y[map_row_offset + x]);

//             uint32x4_t sx_low = (vmovl_u16(vget_low_u16(fx_fixed)));
//             uint32x4_t sx_high = (vmovl_u16(vget_high_u16(fx_fixed)));
//             uint32x4_t sy_low = (vmovl_u16(vget_low_u16(fy_fixed)));
//             uint32x4_t sy_high = (vmovl_u16(vget_high_u16(fy_fixed)));

//             for (int i = 0; i < 4; ++i) {
//                 int xi = x + i;
//                 int sx = std::min<int>(std::max<int>(static_cast<int>(vgetq_lane_u32(sx_low, i)), 0), src_w - 1);
//                 int sy = std::min<int>(std::max<int>(static_cast<int>(vgetq_lane_u32(sy_low, i)), 0), src_h - 1);
//                 uint8_t center = src[sy * src_stride + sx];
//                 uint8_t left   = sx > 0 ? src[sy * src_stride + sx - 1] : center;
//                 uint8_t right  = sx < src_w - 1 ? src[sy * src_stride + sx + 1] : center;
//                 uint8_t top    = sy > 0 ? src[(sy - 1) * src_stride + sx] : center;
//                 uint8_t bottom = sy < src_h - 1 ? src[(sy + 1) * src_stride + sx] : center;
//                 dst[dst_row_offset + xi] = sharpen(center, left, right, top, bottom);
//             }
//             for (int i = 0; i < 4; ++i) {
//                 int xi = x + 4 + i;
//                 int sx = std::min(std::max(vgetq_lane_u32(sx_high, i), 0), src_w - 1);
//                 int sy = std::min(std::max(vgetq_lane_u32(sy_high, i), 0), src_h - 1);
//                 uint8_t center = src[sy * src_stride + sx];
//                 uint8_t left   = sx > 0 ? src[sy * src_stride + sx - 1] : center;
//                 uint8_t right  = sx < src_w - 1 ? src[sy * src_stride + sx + 1] : center;
//                 uint8_t top    = sy > 0 ? src[(sy - 1) * src_stride + sx] : center;
//                 uint8_t bottom = sy < src_h - 1 ? src[(sy + 1) * src_stride + sx] : center;
//                 dst[dst_row_offset + xi] = sharpen(center, left, right, top, bottom);
//             }
//         }
//     }
// }

// resize_and_remap_nearest_neon.cpp
#include <arm_neon.h>
#include <vector>
#include <stdint.h>
#include <algorithm>

// Fixed-point nearest-neighbor remap with NEON for GRAY8 (CV_8UC1)
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
            // Load 8 x-coordinates and 8 y-coordinates
            uint16x8_t mapx_u16 = vld1q_u16(&map_x[map_row_offset + x]);
            uint16x8_t mapy_u16 = vld1q_u16(&map_y[map_row_offset + x]);

            // Convert to 32-bit for indexing
            uint32x4_t sx_lo_u32 = vmovl_u16(vget_low_u16(mapx_u16));
            uint32x4_t sx_hi_u32 = vmovl_u16(vget_high_u16(mapx_u16));
            uint32x4_t sy_lo_u32 = vmovl_u16(vget_low_u16(mapy_u16));
            uint32x4_t sy_hi_u32 = vmovl_u16(vget_high_u16(mapy_u16));

            for (int i = 0; i < 4; ++i) {
                int xi = x + i;
                int sx = std::min((int)vgetq_lane_u32(sx_lo_u32, i), src_w - 1);
                int sy = std::min((int)vgetq_lane_u32(sy_lo_u32, i), src_h - 1);
                dst[dst_row_offset + xi] = src[sy * src_stride + sx];
            }
            for (int i = 0; i < 4; ++i) {
                int xi = x + 4 + i;
                int sx = std::min((int)vgetq_lane_u32(sx_hi_u32, i), src_w - 1);
                int sy = std::min((int)vgetq_lane_u32(sy_hi_u32, i), src_h - 1);
                dst[dst_row_offset + xi] = src[sy * src_stride + sx];
            }
        }
    }
}


inline float32x4_t cubic_weight_neon(float32x4_t x) {
    float32x4_t abs_x = vabsq_f32(x);
    uint32x4_t mask1 = vcleq_f32(abs_x, vdupq_n_f32(1.0f));
    uint32x4_t mask2 = vandq_u32(vcgtq_f32(abs_x, vdupq_n_f32(1.0f)), vcltq_f32(abs_x, vdupq_n_f32(2.0f)));

    float32x4_t x2 = vmulq_f32(abs_x, abs_x);
    float32x4_t x3 = vmulq_f32(x2, abs_x);

    float32x4_t w1 = vmlaq_f32(vdupq_n_f32(-2.5f), abs_x, vdupq_n_f32(1.5f));
    w1 = vmlaq_f32(vdupq_n_f32(1.0f), x2, w1);

    float32x4_t w2 = vmlaq_f32(vdupq_n_f32(2.5f), abs_x, vdupq_n_f32(-0.5f));
    w2 = vmlaq_f32(vdupq_n_f32(-4.0f), x2, w2);
    w2 = vmlaq_f32(vdupq_n_f32(2.0f), x3, w2);

    return vbslq_f32(mask1, w1, vbslq_f32(mask2, w2, vdupq_n_f32(0.0f)));
}

void resize_and_remap_bicubic_neon(
    const uint8_t* __restrict src, int src_w, int src_h, int src_stride,
    uint8_t* __restrict dst, int dst_w, int dst_h, int dst_stride,
    const std::vector<uint16_t>& map_x, const std::vector<uint16_t>& map_y,
    int, int, int channels)
{
    if (channels != 1) {
        printf("Only GRAY8 supported in bicubic NEON.\n");
        return;
    }

    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; x += 4) {
            float fx[4], fy[4];
            int x_int[4], y_int[4];
            float dx[4], dy[4];

            for (int i = 0; i < 4; ++i) {
                int idx = y * dst_w + (x + i);
                fx[i] = clamp((float)map_x[idx], 1.0f, (float)(src_w - 3));
                fy[i] = clamp((float)map_y[idx], 1.0f, (float)(src_h - 3));
                x_int[i] = (int)fx[i];
                y_int[i] = (int)fy[i];
                dx[i] = fx[i] - x_int[i];
                dy[i] = fy[i] - y_int[i];
            }

            float32x4_t dx_v = vld1q_f32(dx);
            float32x4_t dy_v = vld1q_f32(dy);

            float result[4] = {0};

            for (int m = -1; m <= 2; ++m) {
                float32x4_t dy_off = vsubq_f32(vdupq_n_f32((float)m), dy_v);
                float32x4_t wy = cubic_weight_neon(dy_off);

                for (int n = -1; n <= 2; ++n) {
                    float32x4_t dx_off = vsubq_f32(vdupq_n_f32((float)n), dx_v);
                    float32x4_t wx = cubic_weight_neon(dx_off);
                    float32x4_t weight = vmulq_f32(wy, wx);

                    for (int i = 0; i < 4; ++i) {
                        int sx = clamp(x_int[i] + n, 0, src_w - 1);
                        int sy = clamp(y_int[i] + m, 0, src_h - 1);
                        uint8_t p = src[sy * src_stride + sx];
                        result[i] += vgetq_lane_f32(weight, i) * p;
                    }
                }
            }

            for (int i = 0; i < 4; ++i) {
                result[i] = clamp(result[i], 0.0f, 255.0f);
                int cx = clamp(x_int[i], 0, src_w - 1);
                int cy = clamp(y_int[i], 0, src_h - 1);
                uint8_t center = static_cast<uint8_t>(result[i] + 0.5f);
                uint8_t left   = cx > 0 ? src[cy * src_stride + cx - 1] : center;
                uint8_t right  = cx < src_w - 1 ? src[cy * src_stride + cx + 1] : center;
                uint8_t top    = cy > 0 ? src[(cy - 1) * src_stride + cx] : center;
                uint8_t bottom = cy < src_h - 1 ? src[(cy + 1) * src_stride + cx] : center;
                dst[y * dst_stride + x + i] = sharpen(center, left, right, top, bottom);
            }
        }
    }
}
#endif

void resize_and_remap_nearest(
  const uint8_t* src, int src_w, int src_h, int src_stride,
  uint8_t* dst, int dst_w, int dst_h, int dst_stride,
  const std::vector<uint16_t>& map_x, const std::vector<uint16_t>& map_y,
  int, int, int channels)
{
  for (int y = 0; y < dst_h; ++y) {
    for (int x = 0; x < dst_w; ++x) {
      int idx = y * dst_w + x;

      int sx = std::min(std::max((int)map_x[idx], 0), src_w - 1);
      int sy = std::min(std::max((int)map_y[idx], 0), src_h - 1);

      for (int c = 0; c < channels; ++c) {
        dst[y * dst_stride + x * channels + c] =
            src[sy * src_stride + sx * channels + c];
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
        printf("Only GRAY8 supported.\n");
        return;
    }

    for (int y = 0; y < dst_h; ++y) {
        int map_row_offset = y * dst_w;
        int dst_row_offset = y * dst_stride;

        for (int x = 0; x < dst_w; x += 8) {
            uint16x8_t mapx_u16 = vld1q_u16(&map_x[map_row_offset + x]);
            uint16x8_t mapy_u16 = vld1q_u16(&map_y[map_row_offset + x]);

            float fx_array[8], fy_array[8];
            int x0_array[8], y0_array[8];
            float dx_array[8], dy_array[8];

            for (int i = 0; i < 8; ++i) {
                fx_array[i] = static_cast<float>(map_x[map_row_offset + x + i]) * scale_x;
                fy_array[i] = static_cast<float>(map_y[map_row_offset + x + i]) * scale_y;
                x0_array[i] = static_cast<int>(fx_array[i]);
                y0_array[i] = static_cast<int>(fy_array[i]);
                dx_array[i] = fx_array[i] - x0_array[i];
                dy_array[i] = fy_array[i] - y0_array[i];
            }

            for (int i = 0; i < 8; ++i) {
                int xi = x + i;
                int x0i = x0_array[i];
                int y0i = y0_array[i];
                float dxi = dx_array[i];
                float dyi = dy_array[i];

                uint8_t value = 0;
                if (x0i >= 1 && x0i + 1 < src_w - 1 && y0i >= 1 && y0i + 1 < src_h - 1) {
                    uint8_t p00 = src[y0i * src_stride + x0i];
                    uint8_t p10 = src[y0i * src_stride + x0i + 1];
                    uint8_t p01 = src[(y0i + 1) * src_stride + x0i];
                    uint8_t p11 = src[(y0i + 1) * src_stride + x0i + 1];

                    float val = (1 - dxi) * (1 - dyi) * p00 +
                                dxi * (1 - dyi) * p10 +
                                (1 - dxi) * dyi * p01 +
                                dxi * dyi * p11;

                    uint8_t interp = static_cast<uint8_t>(val + 0.5f);

                    uint8_t left = src[y0i * src_stride + x0i - 1];
                    uint8_t right = src[y0i * src_stride + x0i + 1];
                    uint8_t top = src[(y0i - 1) * src_stride + x0i];
                    uint8_t bottom = src[(y0i + 1) * src_stride + x0i];

                    value = sharpen(interp, left, right, top, bottom);
                }
                dst[dst_row_offset + xi] = value;
            }
        }
    }
}
#endif


// // /* transform */
static GstFlowReturn
gst_undistort_transform_frame(GstVideoFilter * filter, GstVideoFrame * inframe, GstVideoFrame * outframe)
{
  GstUndistort *undistortClass = GST_UNDISTORT (filter);

  GST_DEBUG_OBJECT (undistortClass, "transform_frame");

  GstMapInfo map_info;
  GstMapInfo info;

  gst_buffer_map ((inframe->buffer), &map_info, GST_MAP_READ);
  gst_buffer_map((outframe->buffer), &info, GST_MAP_WRITE);

  undistortClass->width = GST_VIDEO_FRAME_WIDTH(inframe);
  undistortClass->height = GST_VIDEO_FRAME_HEIGHT(inframe);
  undistortClass->channels = 1;
  undistortClass->stride =  undistortClass->width *  undistortClass->channels;
  if (undistortClass->initialized == FALSE)
  {
    undistortClass->K = {undistortClass->fx, 0.0, undistortClass->cx, 0.0, undistortClass->fy, undistortClass->cy, 0.0, 0.0, 1.0};
    undistortClass->D = {undistortClass->d0, undistortClass->d1, undistortClass->p1, undistortClass->p2}; // 4 distortion coeffs

    estimateNewCameraMatrixForUndistortRectify(undistortClass->K, undistortClass->D, undistortClass->R, undistortClass->Knew, undistortClass->width, undistortClass->height, undistortClass->balance, undistortClass->fov);
    initFisheyeUndistortRectifyMap(undistortClass->K, undistortClass->D, undistortClass->R, undistortClass->Knew, undistortClass->width, undistortClass->height, undistortClass->map_x, undistortClass->map_y);
    std::cout << "LOOP: INIT_MAPS on the fly with balance: " << undistortClass->balance << " FOV: " << undistortClass->fov << std::endl;
    undistortClass->initialized = TRUE;
  }
  else
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
    else if (undistortClass->interpolation == INTERPOLATION_BICUBIC)
    #if defined(__aarch64__)
    resize_and_remap_bicubic_neon(
      map_info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      undistortClass->map_x, undistortClass->map_y,
      undistortClass->width, undistortClass->height, undistortClass->channels);
    #else
    resize_and_remap_bicubic(
      map_info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      info.data, undistortClass->width, undistortClass->height, undistortClass->stride,
      undistortClass->map_x, undistortClass->map_y,
      undistortClass->width, undistortClass->height, undistortClass->channels);
    #endif
    else if (undistortClass->interpolation == INTERPOLATION_OPENCV)
    {
    #if defined(USE_CV)
      cv::Mat input(undistortClass->height, undistortClass->width, CV_8UC1, map_info.data, undistortClass->stride);
      cv::Mat output(undistortClass->height, undistortClass->width, CV_8UC1, info.data, undistortClass->stride);
      cv::remap(input, output, undistortClass->map1CV, undistortClass->map2CV, cv::INTER_NEAREST);
    #else
    std::cout << "Opencv not defined " << std::endl;
    #endif
    }
  }
  // apply_unsharp_mask(info.data, undistortClass->width, undistortClass->height, undistortClass->stride);  
  gst_buffer_unmap ((inframe->buffer), &map_info);
  gst_buffer_unmap ((outframe->buffer), &info);

  return GST_FLOW_OK;
}



static gboolean
gst_undistort_set_info (GstVideoFilter * filter, GstCaps * incaps,
    GstVideoInfo * in_info, GstCaps * outcaps, GstVideoInfo * out_info)
{
  GstUndistort *undistort = GST_UNDISTORT (filter);
  GST_DEBUG_OBJECT (undistort, "set_info");

  undistort->width = GST_VIDEO_INFO_WIDTH(in_info);
  undistort->height = GST_VIDEO_INFO_HEIGHT(in_info);
  undistort->channels = 1;
  undistort->stride = undistort->width * undistort->channels;

  undistort->K = { undistort->fx, 0.0f, undistort->cx,
                   0.0f, undistort->fy, undistort->cy,
                   0.0f, 0.0f, 1.0f };

  undistort->D = { undistort->d0, undistort->d1, undistort->p1, undistort->p2 };



  #if defined(USE_CV)
    compute_undistort_maps_from_opencv(undistort, undistort->K, undistort->D, undistort->R, undistort->Knew, undistort->width , undistort->height, undistort->balance, undistort->fov, undistort->map_x, undistort->map_y);
  #else
  estimateNewCameraMatrixForUndistortRectify(
      undistort->K, undistort->D, undistort->R,
      undistort->Knew, undistort->width, undistort->height,
      undistort->balance, undistort->fov);

  initFisheyeUndistortRectifyMap(
      undistort->K, undistort->D, undistort->R, undistort->Knew,
      undistort->width, undistort->height,
      undistort->map_x, undistort->map_y);
  #endif
  undistort->initialized = TRUE;
  std::cout << "Map initialized in set_info()\n";

  return TRUE;
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

