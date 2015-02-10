/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <iostream>
#include <fstream>
#include <cmath>

#include <libfreenect2/async_packet_processor.h>
#include <libfreenect2/depth_packet_processor.h>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/resource.h>

void loadBufferFromFile(const char *filename, unsigned char *buffer, size_t n)
{
  std::ifstream in(filename, std::ios::binary);

  in.read(reinterpret_cast<char*>(buffer), n);
  if(in.gcount() != n) throw std::exception();

  in.close();
}

#include <sys/time.h>
double now()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec/1e6;
}

void saveFloatsToFile(const char *filename, const char *label, const unsigned char *buffer, size_t n)
{
  std::ofstream out((std::string(filename) + "." + label + ".floats").c_str(), std::ios::binary);
  out.write(reinterpret_cast<const char*>(buffer), n*sizeof(float));
  out.close();
}
void savePPMToFile(const char *filename, const char *label, const unsigned char *buffer, size_t n)
{
  std::ofstream out((std::string(filename) + "." + label + ".ppm").c_str(), std::ios::binary);
  char header[] = "P5\n512 424\n255\n";
  out.write(header, sizeof(header) - 1);

  const float *pixel = reinterpret_cast<const float *>(buffer);
  unsigned char ppm[512*424];
  for (size_t i = 0; i < 424; i++) {
    for (size_t j = 0; j < 512; j++) {
      ppm[i * 512 + j] = (unsigned char)round((double)pixel[i * 512 + j] / 65535.0 * 255.0);
    }
  }
  out.write(reinterpret_cast<char*>(ppm), sizeof(ppm));

  out.close();
}

double compare(const libfreenect2::Frame *a, const libfreenect2::Frame *b, size_t size)
{
  const float *x = reinterpret_cast<const float *>(a->data);
  const float *y = reinterpret_cast<const float *>(b->data);
  double error = 0;
  float x_max=-1.0f, y_max=-1.0f;
  float x_min=1e10f, y_min=1e10f;
  for (size_t i = 0; i < size; i++)
  {
    error += fabsf(x[i] - y[i]);
    if (x[i] > x_max) x_max = x[i];
    if (x[i] < x_min) x_min = x[i];
    if (y[i] > y_max) y_max = y[i];
    if (y[i] < y_min) y_min = y[i];
  }
  std::cerr << "max: " << x_max << " " << y_max << " min: " << x_min << " " << y_min << std::endl;
  return error;
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " DEPTH_PACKET..." << std::endl;
    return 1;
  }
  libfreenect2::DepthPacketProcessor::Config cfg;
  cfg.EnableBilateralFilter = false;
  cfg.EnableEdgeAwareFilter = false;

  libfreenect2::SyncMultiFrameListener test_listener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap test_frames;
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
  libfreenect2::CudaDepthPacketProcessor test_processor;
#elif LIBFREENECT2_WITH_OPENCL_SUPPORT
  libfreenect2::OpenCLDepthPacketProcessor test_processor;
#else
#error no gpu to test
#endif
  test_processor.setConfiguration(cfg);
  test_processor.setFrameListener(&test_listener);
  test_processor.load11To16LutFromFile("11to16.bin");
  test_processor.loadXTableFromFile("xTable.bin");
  test_processor.loadZTableFromFile("zTable.bin");

  libfreenect2::SyncMultiFrameListener ref_listener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap ref_frames;
  libfreenect2::CpuDepthPacketProcessor ref_processor;
  ref_processor.setConfiguration(cfg);
  ref_processor.setFrameListener(&ref_listener);
  ref_processor.load11To16LutFromFile("11to16.bin");
  ref_processor.loadXTableFromFile("xTable.bin");
  ref_processor.loadZTableFromFile("zTable.bin");

  libfreenect2::DepthPacket p;
  p.buffer_length = 352*424*10*2;
  p.buffer = new unsigned char[p.buffer_length];
  libfreenect2::Frame *test_ir, *test_depth, *ref_ir, *ref_depth;

  for (int i = 1; i < argc; i++)
  {
    loadBufferFromFile(argv[i], p.buffer, p.buffer_length);

    double start = now();
    test_processor.process(p);
    std::cout << "test_processor " << (now() - start) * 1e3 << " msec" << std::endl;
    test_listener.waitForNewFrame(test_frames);
    test_ir = test_frames[libfreenect2::Frame::Ir];
    test_depth = test_frames[libfreenect2::Frame::Depth];
/*
    ref_processor.process(p);
    ref_listener.waitForNewFrame(ref_frames);
    ref_ir = ref_frames[libfreenect2::Frame::Ir];
    ref_depth = ref_frames[libfreenect2::Frame::Depth];

    size_t image_size = 512 * 424;

    savePPMToFile(argv[i], "test_ir", test_ir->data, image_size);
    savePPMToFile(argv[i], "ref_ir", ref_ir->data, image_size);

    saveFloatsToFile(argv[i], "test_ir", test_ir->data, image_size);
    saveFloatsToFile(argv[i], "ref_ir", ref_ir->data, image_size);

    savePPMToFile(argv[i], "test_depth", test_depth->data, image_size);
    savePPMToFile(argv[i], "ref_depth", ref_depth->data, image_size);

    saveFloatsToFile(argv[i], "test_depth", test_depth->data, image_size);
    saveFloatsToFile(argv[i], "ref_depth", ref_depth->data, image_size);

    //compare
    double ir_error = compare(test_ir, ref_ir, image_size);
    double depth_error = compare(test_depth, ref_depth, image_size);
    std::cout << argv[i] << ": ir sum error " << ir_error  << ", " << ir_error / image_size << std::endl;
    std::cout << argv[i] << ": depth sum error " << ir_error << ", " << depth_error / image_size << std::endl;
*/    
    test_listener.release(test_frames);
//    ref_listener.release(ref_frames);
  }

  return 0;
}
