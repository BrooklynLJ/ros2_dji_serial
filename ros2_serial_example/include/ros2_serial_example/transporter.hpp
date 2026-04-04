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
// https://github.com/PX4/px4_ros_com/blob/69bdf6e70f3832ff00f2e9e7f17d9394532787d6/templates/microRTPS_transport.h
// Modified to use the DJI UART framing protocol (0xA5 header, CRC8 header
// check, CRC16 full-frame check) compatible with the MCB serial protocol.
//
// DJI Frame Layout (all multi-byte fields little-endian):
// +---------+---------------------------------------------------------------+
// | Byte(s) | Description                                                   |
// +=========+===============================================================+
// | 0       | Frame Head Byte (0xA5)                                        |
// | 1-2     | Data Length – payload only (uint16_t LE)                      |
// | 3       | Sequence number                                                |
// | 4       | CRC8  of bytes 0-3                                            |
// | 5-6     | Message Type / Topic ID (uint16_t LE)                         |
// | 7..N    | Payload (dataLength bytes)                                     |
// | N+1,N+2 | CRC16 of bytes 0..N (header + payload), LE                   |
// +---------+---------------------------------------------------------------+

#ifndef ROS2_SERIAL_EXAMPLE__TRANSPORTER_HPP_
#define ROS2_SERIAL_EXAMPLE__TRANSPORTER_HPP_

#include <cstdint>
#include <mutex>
#include <string>

#include "ros2_serial_example/ring_buffer.hpp"
#include "ros2_serial_example/crc_dji.hpp"

// topic_id_size_t maps directly to the 16-bit DJI msgType field.
typedef uint16_t topic_id_size_t;

namespace ros2_to_serial_bridge
{

namespace transport
{

/**
 * Abstract transporter using the DJI UART framing protocol.
 *
 * Derived classes provide node_read() / node_write() / fds_OK() for the
 * specific physical transport (UART, UDP, …).  All framing, CRC calculation,
 * and ring-buffer management live here.
 */
class Transporter
{
public:
    /**
     * @param[in] ring_buffer_size  Bytes for the receive ring buffer (8192
     *                              is a good starting point).
     */
    explicit Transporter(size_t ring_buffer_size);
    virtual ~Transporter();

    Transporter(Transporter const &) = delete;
    Transporter & operator=(Transporter const &) = delete;
    Transporter(Transporter &&) = delete;
    Transporter & operator=(Transporter &&) = delete;

    /** Transport-specific init. Returns 0 on success, -1 on error. */
    virtual int init() {return 0;}

    /** Transport-specific teardown. Returns 0 on success, -1 on error. */
    virtual int close() {return 0;}

    /**
     * Read one DJI frame from the underlying transport.
     *
     * @param[out] topic_ID    Filled with the msgType field on success.
     * @param[out] out_buffer  Destination for the payload bytes.
     * @param[in]  buffer_len  Capacity of out_buffer.
     * @returns Payload length (>= 0) on success, or a negative errno value.
     */
    ssize_t read(topic_id_size_t * topic_ID, uint8_t * out_buffer, size_t buffer_len);

    /**
     * Wrap payload in a DJI frame and transmit it.
     *
     * @param[in] topic_ID    Written into the frame's msgType field.
     * @param[in] buffer      Payload bytes (may be nullptr if data_length==0).
     * @param[in] data_length Number of payload bytes.
     * @returns Payload length written on success, or -1 on error.
     */
    ssize_t write(topic_id_size_t topic_ID, uint8_t const * buffer, size_t data_length);

protected:
    /** Pull bytes from the physical transport into ringbuf_. */
    virtual ssize_t node_read() = 0;

    /** Push len bytes from buffer to the physical transport (blocking). */
    virtual ssize_t node_write(void * buffer, size_t len) = 0;

    /** Return true when the underlying file descriptors are usable. */
    virtual bool fds_OK() = 0;

    impl::RingBuffer ringbuf_;

    /**
     * Scan ringbuf_ for a complete, CRC-validated DJI frame and copy its
     * payload into out_buffer.  Exposed so unit tests can call it directly.
     */
    ssize_t find_and_copy_message(topic_id_size_t * topic_ID,
                                  uint8_t * out_buffer,
                                  size_t buffer_len);

private:
    static constexpr uint8_t FRAME_HEAD = 0xA5;

    // DJI wire header – packed, exactly 7 bytes.
    struct __attribute__((packed)) DJIHeader
    {
        uint8_t  head;        // 0xA5
        uint16_t dataLength;  // payload length (LE)
        uint8_t  seq;
        uint8_t  crc8;        // CRC8 over bytes [0, crc8)
        uint16_t msgType;     // topic ID (LE)
    };
    static_assert(sizeof(DJIHeader) == 7, "DJIHeader must be exactly 7 bytes");

    // Bytes covered by CRC8 = everything before the crc8 field.
    static constexpr size_t CRC8_LEN = offsetof(DJIHeader, crc8);  // == 4

    uint8_t seq_{0};
    std::mutex write_mutex_;
};

}  // namespace transport
}  // namespace ros2_to_serial_bridge

#endif  // ROS2_SERIAL_EXAMPLE__TRANSPORTER_HPP_
