// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Originally based on:
// https://github.com/PX4/px4_ros_com/blob/69bdf6e70f3832ff00f2e9e7f17d9394532787d6/templates/microRTPS_transport.cpp
// Modified to use the DJI UART framing protocol (0xA5 SOF, CRC8 over header
// bytes 0-3, CRC16 over header+payload).

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>

#include "ros2_serial_example/transporter.hpp"

namespace ros2_to_serial_bridge
{

namespace transport
{

// ── Construction / destruction ────────────────────────────────────────────────

Transporter::Transporter(size_t ring_buffer_size)
: ringbuf_(ring_buffer_size)
{}

Transporter::~Transporter() = default;

// ── Receive path ──────────────────────────────────────────────────────────────

ssize_t Transporter::find_and_copy_message(topic_id_size_t * topic_ID,
                                           uint8_t * out_buffer,
                                           size_t buffer_len)
{
    // We need at least a full header + the two CRC16 bytes even for a
    // zero-payload frame.
    constexpr size_t MIN_FRAME = sizeof(DJIHeader) + sizeof(uint16_t);

    while (ringbuf_.bytes_used() >= MIN_FRAME)
    {
        // ── 1. Synchronise on SOF byte ─────────────────────────────────────
        uint8_t sof = 0;
        if (ringbuf_.peek(&sof, 1) < 0)
        {
            return -ENODATA;
        }
        if (sof != FRAME_HEAD)
        {
            // Discard one byte and keep scanning.
            uint8_t dummy;
            ringbuf_.memcpy_from(&dummy, 1);
            continue;
        }

        // ── 2. Peek the full header ────────────────────────────────────────
        if (ringbuf_.bytes_used() < sizeof(DJIHeader))
        {
            return -ENODATA;  // wait for more bytes
        }

        std::unique_ptr<uint8_t[]> hdr_buf(new uint8_t[sizeof(DJIHeader)]);
        if (ringbuf_.peek(hdr_buf.get(), sizeof(DJIHeader)) < 0)
        {
            return -ENODATA;
        }

        DJIHeader header;
        ::memcpy(&header, hdr_buf.get(), sizeof(DJIHeader));

        // ── 3. Validate CRC8 (covers bytes 0 .. CRC8_LEN-1) ───────────────
        uint8_t calc_crc8 = calculateCRC8(hdr_buf.get(), CRC8_LEN);
        if (calc_crc8 != header.crc8)
        {
            ::fprintf(stderr,
                      "[DJI transporter] CRC8 mismatch (seq=%u): "
                      "got 0x%02X expected 0x%02X – dropping SOF byte\n",
                      header.seq, header.crc8, calc_crc8);
            uint8_t dummy;
            ringbuf_.memcpy_from(&dummy, 1);
            continue;
        }

        // ── 4. Check we have the complete frame in the ring buffer ─────────
        size_t total_frame = sizeof(DJIHeader) + header.dataLength + sizeof(uint16_t);
        if (ringbuf_.bytes_used() < total_frame)
        {
            return -ENODATA;  // partial frame – wait for more bytes
        }

        // ── 5. Check the output buffer is large enough ─────────────────────
        if (buffer_len < header.dataLength)
        {
            ::fprintf(stderr,
                      "[DJI transporter] payload %u bytes won't fit in buffer "
                      "(%zu bytes) – dropping frame\n",
                      header.dataLength, buffer_len);
            // Consume the whole frame so we can recover.
            std::vector<uint8_t> trash(total_frame);
            ringbuf_.memcpy_from(trash.data(), total_frame);
            return -EMSGSIZE;
        }

        // ── 6. Read the complete frame out of the ring buffer ──────────────
        std::unique_ptr<uint8_t[]> frame(new uint8_t[total_frame]);
        if (ringbuf_.memcpy_from(frame.get(), total_frame) < 0)
        {
            throw std::runtime_error("[DJI transporter] unexpected ring buffer failure");
        }

        // ── 7. Validate CRC16 (covers header + payload) ───────────────────
        size_t crc16_offset = sizeof(DJIHeader) + header.dataLength;
        uint16_t recv_crc16 = static_cast<uint16_t>(frame[crc16_offset]) |
                              (static_cast<uint16_t>(frame[crc16_offset + 1]) << 8U);
        uint16_t calc_crc16 = calculateCRC16(frame.get(), crc16_offset);

        if (recv_crc16 != calc_crc16)
        {
            ::fprintf(stderr,
                      "[DJI transporter] CRC16 mismatch (seq=%u, msgType=0x%04X): "
                      "got 0x%04X expected 0x%04X\n",
                      header.seq, header.msgType, recv_crc16, calc_crc16);
            // We already consumed the frame; the ring is consistent. Report bad
            // message so the caller can track it, but don't return the payload.
            return -EBADMSG;
        }

        // ── 8. All good – hand the payload back to the caller ──────────────
        *topic_ID = header.msgType;
        if (header.dataLength > 0)
        {
            ::memcpy(out_buffer, frame.get() + sizeof(DJIHeader), header.dataLength);
        }
        return static_cast<ssize_t>(header.dataLength);
    }

    return -ENODATA;
}

ssize_t Transporter::read(topic_id_size_t * topic_ID,
                          uint8_t * out_buffer,
                          size_t buffer_len)
{
    if (nullptr == out_buffer || nullptr == topic_ID || !fds_OK())
    {
        return -1;
    }

    *topic_ID = std::numeric_limits<topic_id_size_t>::max();

    // First attempt: try to parse from whatever is already in the ring buffer.
    constexpr size_t MIN_FRAME = sizeof(DJIHeader) + sizeof(uint16_t);
    if (ringbuf_.bytes_used() >= MIN_FRAME)
    {
        ssize_t len = find_and_copy_message(topic_ID, out_buffer, buffer_len);
        if (len >= 0)
        {
            return len;
        }
    }

    // Pull more bytes from the physical transport.
    ssize_t len = node_read();
    if (len < 0)
    {
        if (errno != 0 && errno != EAGAIN && errno != ETIMEDOUT)
        {
            ::fprintf(stderr, "[DJI transporter] node_read() failed: %d\n", errno);
        }
        return len;
    }
    if (len == 0)
    {
        return -ENODATA;
    }

    // Second attempt after fresh data.
    if (ringbuf_.bytes_used() >= MIN_FRAME)
    {
        ssize_t msg_len = find_and_copy_message(topic_ID, out_buffer, buffer_len);
        if (msg_len >= 0)
        {
            return msg_len;
        }
    }

    return -ENODATA;
}

// ── Transmit path ─────────────────────────────────────────────────────────────

ssize_t Transporter::write(topic_id_size_t topic_ID,
                           uint8_t const * buffer,
                           size_t data_length)
{
    if (!fds_OK())
    {
        return -1;
    }

    // Allow zero-length payloads but not a non-null buffer paired with zero
    // length, or a null buffer paired with non-zero length.
    if ((buffer == nullptr && data_length > 0) ||
        (buffer != nullptr && data_length == 0))
    {
        return -1;
    }

    std::lock_guard<std::mutex> lock(write_mutex_);

    // ── Assemble header ────────────────────────────────────────────────────
    DJIHeader header{};
    header.head       = FRAME_HEAD;
    header.dataLength = static_cast<uint16_t>(data_length);
    header.seq        = seq_++;
    header.crc8       = calculateCRC8(reinterpret_cast<uint8_t *>(&header), CRC8_LEN);
    header.msgType    = topic_ID;

    // ── Allocate frame buffer: header + payload + CRC16 ───────────────────
    size_t frame_len = sizeof(DJIHeader) + data_length + sizeof(uint16_t);
    std::unique_ptr<uint8_t[]> frame(new uint8_t[frame_len]);

    ::memcpy(frame.get(), &header, sizeof(DJIHeader));
    if (buffer != nullptr && data_length > 0)
    {
        ::memcpy(frame.get() + sizeof(DJIHeader), buffer, data_length);
    }

    // ── Append CRC16 (little-endian) ──────────────────────────────────────
    size_t crc16_offset = sizeof(DJIHeader) + data_length;
    uint16_t crc16 = calculateCRC16(frame.get(), crc16_offset);
    frame[crc16_offset]     = static_cast<uint8_t>(crc16 & 0xFFU);
    frame[crc16_offset + 1] = static_cast<uint8_t>((crc16 >> 8U) & 0xFFU);

    // ── Send ───────────────────────────────────────────────────────────────
    ssize_t written = node_write(frame.get(), frame_len);
    if (written < 0)
    {
        return written;
    }

    return static_cast<ssize_t>(data_length);
}

}  // namespace transport
}  // namespace ros2_to_serial_bridge
