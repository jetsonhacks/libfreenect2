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
#include <libfreenect2/depth_packet_processor.h>
#include <libfreenect2/protocol/response.h>

#include <iostream>
#include <cstdio>
#include <vector_types.h>

#include <sys/time.h>
static inline double now()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec/1e6;
}

void loadRaw(const char *filename, void *buffer, size_t n)
{
  FILE *f = fopen(filename, "rb");
  if (fread(buffer, 1, n, f) == 0)
    throw "too short";
  fclose(f);
}

#include <fstream>
#include <string>
#include <cstring>
#include <cmath>
void HSVtoRGB(float h, unsigned char *pixel3)
{
    int i;
    float f, p, q, t;
    float r, g, b;
    const float s = 1.0f;
    const float v = 1.0f;
    h *= 360;
    h /= 60;            // sector 0 to 5
    i = floor( h );
    f = h - i;          // factorial part of h
    p = v * ( 1 - s );
    q = v * ( 1 - s * f );
    t = v * ( 1 - s * ( 1 - f ) );
    switch( i ) {
        case 0:         r = v;          g = t;          b = p;          break;
        case 1:         r = q;          g = v;          b = p;          break;
        case 2:         r = p;          g = v;          b = t;          break;
        case 3:         r = p;          g = q;          b = v;          break;
        case 4:         r = t;          g = p;          b = v;          break;
        default:        r = v;          g = p;          b = q;          break;
    }
  pixel3[0] = r * 255;
  pixel3[1] = g * 255;
  pixel3[2] = b * 255;
}

void savePPM(const char *label, void *buffer)
{
  std::ofstream out((std::string("/dev/shm/") + label + ".ppm").c_str(), std::ios::binary);
  char header[] = "P6\n512 424\n255\n";
  out.write(header, sizeof(header) - 1);

  const float *pixel = reinterpret_cast<const float *>(buffer);
  static unsigned char ppm[512*424*3];
  float max = -1, min = 1e10;

  for (size_t i = 0; i < 424; i++) {
    for (size_t j = 0; j < 512; j++) {
      float f = pixel[i * 512 + j];
      if (f > max) max = f;
      if (f < min) min = f;
    }
  }

  for (size_t i = 0; i < 424; i++) {
    for (size_t j = 0; j < 512; j++) {
      float f = (pixel[i * 512 + j] - min) / (max - min);
      HSVtoRGB(f, &ppm[3*(i*512+j)]);
    }
  }
  out.write(reinterpret_cast<char*>(ppm), sizeof(ppm));

  out.close();

}
  static short lut11to16[2048];
  static float x_table[512 * 424];
  static float z_table[512 * 424];
  static float4 p0_table[512 * 424];

inline int bfi(int width, int offset, int src2, int src3)
{
  int bitmask = (((1 << width)-1) << offset) & 0xffffffff;
  return ((src2 << offset) & bitmask) | (src3 & ~bitmask);
}

int32_t decodePixelMeasurement(unsigned char* data, int sub, int x, int y)
  {
    // 298496 = 512 * 424 * 11 / 8 = number of bytes per sub image
    uint16_t *ptr = reinterpret_cast<uint16_t *>(data + 298496 * sub);
    int i = y < 212 ? y + 212 : 423 - y;
    ptr += 352*i;

    /**
     r1.yz = r2.xxyx < l(0, 1, 0, 0) // ilt
     r1.y = r1.z | r1.y // or
     */
    bool r1y = x < 1 || y < 0;
    /*
    r1.zw = l(0, 0, 510, 423) < r2.xxxy // ilt
    r1.z = r1.w | r1.z // or
    */
    bool r1z = 511 < x || 423 < y;
    /*
    r1.y = r1.z | r1.y // or
    */
    r1y = r1y || r1z;
    /*
    r1.y = r1.y & l(0x1fffffff) // and
    */
    int r1yi = r1y ? 0xffffffff : 0x0;
    r1yi &= 0x1fffffff;

    /*
    bfi r1.z, l(2), l(7), r2.x, l(0)
    ushr r1.w, r2.x, l(2)
    r1.z = r1.w + r1.z // iadd
    */
    int r1zi = bfi(2, 7, x, 0);
    int r1wi = x >> 2;
    r1zi = r1wi + r1zi;

    /*
    imul null, r1.z, r1.z, l(11)
    ushr r1.w, r1.z, l(4)
    r1.y = r1.w + r1.y // iadd
    r1.w = r1.y + l(1) // iadd
    r1.z = r1.z & l(15) // and
    r4.w = -r1.z + l(16) // iadd
     */
    r1zi = (r1zi * 11L) & 0xffffffff;
    r1wi = r1zi >> 4;
    r1yi = r1yi + r1wi;
    r1zi = r1zi & 15;
    int r4wi = -r1zi + 16;

    if(r1yi > 352)
    {
      return lut11to16[0];
    }

    int i1 = ptr[r1yi];
    int i2 = ptr[r1yi + 1];
if (x >= 511) {
std::cerr << x << " " << y <<  " " << ((i1 | i2) & 2047) << std::endl;
}
    i1 = i1 >> r1zi;
    i2 = i2 << r4wi;
    return lut11to16[((i1 | i2) & 2047)];
  }

int main(int argc, char **argv)
{
  if (argc <= 5)
  {
    fprintf(stderr, "Usage: %s lut x z p0 packets...\n", argv[0]);
    return 1;
  }

  libfreenect2::CudaDepthPacketProcessorKernel kernel;

  size_t block_size = 128;
  kernel.initDevice(-1, 512*424, block_size);

  libfreenect2::DepthPacketProcessor::Parameters params;
  libfreenect2::DepthPacketProcessor::Config config;
  config.EnableBilateralFilter = true;
  config.EnableEdgeAwareFilter = true;
  kernel.generateOptions(params, config);

  loadRaw(argv[1], lut11to16, sizeof(lut11to16));
  loadRaw(argv[2], x_table, sizeof(x_table));
  loadRaw(argv[3], z_table, sizeof(z_table));
  loadRaw(argv[4], p0_table, sizeof(p0_table));

  kernel.loadTables(lut11to16, p0_table, x_table, z_table);

  static unsigned char buffer[2984960];

  libfreenect2::DepthPacket packet;
  packet.buffer = buffer;
  packet.buffer_length = sizeof(buffer);

  libfreenect2::Frame ir_frame(512, 424, 4), depth_frame(512, 424, 4);
/*loadRaw(argv[5], buffer, sizeof(buffer));
float *pixel = reinterpret_cast<float *>(ir_frame.data);
char filename [128];
strcpy(filename, "sub0");
for (size_t sub = 0; sub <= 9; sub++)
{
    for(size_t y = 0; y < 424; ++y)
    {
      for(size_t x = 0; x < 512; ++x)
      {
        pixel[y*512+x] = (float)decodePixelMeasurement(buffer, sub, x, y);
      }
    }
    filename[3] = '0' + sub;
    savePPM(filename, ir_frame.data);
}
return 0;
*/
  double total_duration = 0;
  int count = 0;
  for (int i = 5; i < argc; i++)
  {
    loadRaw(argv[i], buffer, sizeof(buffer));
    double start = now();
    kernel.run(packet, &ir_frame, &depth_frame, config);
    double duration = now() - start;
    total_duration += duration;
    count += 1;
    std::cerr << count << std::endl;
  }

  std::cout << (total_duration / count) *1e3 << "ms " << count / total_duration << "Hz" <<std::endl;

  savePPM("depth", depth_frame.data);
  return 0;
}
