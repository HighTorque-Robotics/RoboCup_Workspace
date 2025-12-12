#ifdef USE_GST_CAMERA
#pragma once

#include <gst/gst.h>
#include <string>

/**
 * gstCodec enumeration
 */
enum gstCodec { GST_CODEC_H264 = 0, GST_CODEC_H265 };

/**
 * LOG_GSTREAMER printf prefix
 */
#define LOG_GSTREAMER "[gstreamer] "

/**
 * gstreamerInit
 */
bool gstreamerInit();

/**
 * gst_message_print
 */
gboolean gst_message_print(_GstBus* bus, _GstMessage* message, void* user_data);
#endif
