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

void loadBufferFromFile(const char *filename, float *buffer, size_t n)
{
  std::ifstream in(filename, std::ios::binary);

  in.read(reinterpret_cast<char*>(buffer), n);
  if(in.gcount() != n) throw std::exception();

  in.close();
}

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage: " << argv[0] << " file1 file2" << std::endl;
    return 1;
  }
  
  size_t image_size = 512 * 424;
  float x[image_size], y[image_size];
  loadBufferFromFile(argv[1], x, sizeof(x));
  loadBufferFromFile(argv[2], y, sizeof(y));

  double error = 0;
  float x_max=-1.0f, y_max=-1.0f;
  float x_min=1e10f, y_min=1e10f;
  for (size_t i = 0; i < image_size; i++)
  {
    error += fabsf(x[i] - y[i]);
    if (x[i] > x_max) x_max = x[i];
    if (x[i] < x_min) x_min = x[i];
    if (y[i] > y_max) y_max = y[i];
    if (y[i] < y_min) y_min = y[i];
  }
  std::cout << "max: " << x_max << " " << y_max << " min: " << x_min << " " << y_min << std::endl;
  std::cout << "sum error " << error  << ", " << error / image_size << std::endl;

  return 0;
}
