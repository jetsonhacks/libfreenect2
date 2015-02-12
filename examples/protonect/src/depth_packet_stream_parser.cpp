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

#include <libfreenect2/depth_packet_stream_parser.h>
#include <iostream>
#include <memory.h>
#include <algorithm>

namespace libfreenect2
{

DepthPacketStreamParser::DepthPacketStreamParser() :
    processor_(noopProcessor<DepthPacket>()),
    started_(false),
    current_sequence_(-1),
    current_subsequence_(0)
{
  size_t single_image = 512*424*11/8;

  buffer_.allocate((single_image) * 10);
  buffer_.front().length = buffer_.front().capacity;
  buffer_.back().length = buffer_.back().capacity;

  work_buffer_.data = buffer_.front().data;
  work_buffer_.capacity = single_image;
  work_buffer_.length = 0;
}

DepthPacketStreamParser::~DepthPacketStreamParser()
{
}

void DepthPacketStreamParser::setPacketProcessor(libfreenect2::BaseDepthPacketProcessor *processor)
{
  processor_ = (processor != 0) ? processor : noopProcessor<DepthPacket>();
}

void DepthPacketStreamParser::onDataReceived(unsigned char* buffer, size_t length)
{
  if (length == 0)
    return;

  Buffer &fb = buffer_.front();
  Buffer &wb = work_buffer_;

  DepthSubPacketFooter *footer = reinterpret_cast<DepthSubPacketFooter *>(&buffer[length - sizeof(DepthSubPacketFooter)]);
  bool new_subpacket = false;
  if (footer->magic0 == 0x0 && footer->magic1 == 0x9)
  {
    new_subpacket = true;
    length -= sizeof(DepthSubPacketFooter);
  }

  if (wb.length + length > wb.capacity)
  {
    std::cerr << "[DepthPacketStreamParser::handleNewData] subpacket too large" << std::endl;
    goto drop_packet;
  }

  memcpy(wb.data + wb.length, buffer, length);
  wb.length += length;

  if (!new_subpacket)
    return;

  //subpacket size is fixed
  if (wb.length != wb.capacity)
  {
    std::cerr << "[DepthPacketStreamParser::handleNewData] subpacket incomplete (needed "
              << wb.capacity << " received " << wb.length << ")" << std::endl;
    goto drop_packet;
  }

  if (!started_)
  {
    if (footer->subsequence == 0)
      current_sequence_ = footer->sequence;
    else
      goto drop_packet;
  }

  if (footer->sequence != current_sequence_ || footer->subsequence != current_subsequence_)
  {
    std::cerr << "[DepthPacketStreamParser::handleNewData] packet loss (sequence expected "
              << current_sequence_ << "." << current_subsequence_ << " actual " << footer->sequence << "." << footer->subsequence << ")" << std::endl;
    goto drop_packet;
  }

  started_ = true;
  current_subsequence_ = (current_subsequence_ + 1) % 10;
  wb.length = 0;
  wb.data = &fb.data[wb.capacity*current_subsequence_];

  if (current_subsequence_ != 0)
    return;
  current_sequence_++;

  if (!processor_->ready())
  {
    std::cerr << "[DepthPacketStreamParser::handleNewData] skipping depth packet because processor is not ready" << std::endl;
    return;
  }

  buffer_.swap();
  DepthPacket packet;
  packet.sequence = current_sequence_ - 1;
  packet.buffer = buffer_.back().data;
  packet.buffer_length = buffer_.back().capacity;
  processor_->process(packet);
  return;

drop_packet:
  current_subsequence_ = 0;
  current_sequence_ = footer->sequence + 1;
  wb.length = 0;
  wb.data = &fb.data[wb.capacity*current_subsequence_];
}
} /* namespace libfreenect2 */
