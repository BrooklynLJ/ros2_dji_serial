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
// Unit tests for the DJI-protocol Transporter.
// These replace the original PX4/COBS tests which are no longer applicable.

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <unistd.h>

#include <gtest/gtest.h>

#include "ros2_serial_example/transporter.hpp"

// ── Concrete in-process transporter for testing ──────────────────────────────
//
// The RingBuffer only accepts data via read(int fd), so we maintain a pipe.
// node_read() writes rx_data_ into the write-end of the pipe, then calls
// ringbuf_.read() on the read-end so the data lands in the ring buffer.
// node_write() captures the framed bytes into written_data for inspection.

class TransporterPassThrough final : public ros2_to_serial_bridge::transport::Transporter
{
public:
    explicit TransporterPassThrough(size_t ring_buffer_size = 4096)
    : Transporter(ring_buffer_size)
    {
        if (::pipe(pipe_fds_) < 0)
        {
            throw std::runtime_error("pipe() failed in TransporterPassThrough");
        }
    }

    ~TransporterPassThrough() override
    {
        ::close(pipe_fds_[0]);
        ::close(pipe_fds_[1]);
    }

    // Bytes written by Transporter::write() accumulate here for assertions.
    std::vector<uint8_t> written_data;

    // Populate this before calling read(); node_read() will push it into the ring.
    std::vector<uint8_t> rx_data;

protected:
    ssize_t node_write(void * buffer, size_t len) override
    {
        const uint8_t * p = static_cast<const uint8_t *>(buffer);
        written_data.insert(written_data.end(), p, p + len);
        return static_cast<ssize_t>(len);
    }

    ssize_t node_read() override
    {
        if (rx_data.empty()) { return 0; }

        // Write all rx bytes into the pipe write-end.
        ssize_t written = ::write(pipe_fds_[1], rx_data.data(), rx_data.size());
        rx_data.clear();
        if (written < 0) { return -1; }

        // Drain the pipe read-end into the ring buffer.
        ssize_t total = 0;
        ssize_t n;
        while ((n = ringbuf_.read(pipe_fds_[0])) > 0)
        {
            total += n;
        }
        return total;
    }

    bool fds_OK() override { return true; }

private:
    int pipe_fds_[2]{-1, -1};
};

// ── Helpers ───────────────────────────────────────────────────────────────────

// Build a valid DJI frame matching the wire layout exactly:
//   [0xA5][dataLen_L][dataLen_H][seq][crc8][msgType_L][msgType_H][payload...][crc16_L][crc16_H]
static std::vector<uint8_t> make_dji_frame(uint16_t msg_type,
                                            uint8_t  seq,
                                            const std::vector<uint8_t> & payload)
{
    uint16_t dataLength = static_cast<uint16_t>(payload.size());

    // The 4 bytes covered by CRC8: head, dataLen_L, dataLen_H, seq
    uint8_t pre_crc[4] = {
        0xA5,
        static_cast<uint8_t>(dataLength & 0xFF),
        static_cast<uint8_t>((dataLength >> 8) & 0xFF),
        seq
    };
    uint8_t crc8 = calculateCRC8(pre_crc, 4);

    std::vector<uint8_t> frame;
    frame.push_back(0xA5);
    frame.push_back(pre_crc[1]);
    frame.push_back(pre_crc[2]);
    frame.push_back(seq);
    frame.push_back(crc8);
    frame.push_back(static_cast<uint8_t>(msg_type & 0xFF));
    frame.push_back(static_cast<uint8_t>((msg_type >> 8) & 0xFF));
    frame.insert(frame.end(), payload.begin(), payload.end());

    uint16_t crc16 = calculateCRC16(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc16 & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc16 >> 8) & 0xFF));
    return frame;
}

// ── TX path tests: Transporter::write() ──────────────────────────────────────

TEST(TransporterWrite, EmptyPayload)
{
    TransporterPassThrough t;
    ssize_t ret = t.write(42, nullptr, 0);
    ASSERT_EQ(ret, 0);
    // 7-byte header + 0 payload + 2-byte CRC16 = 9 bytes on the wire
    ASSERT_EQ(t.written_data.size(), 9u);
    ASSERT_EQ(t.written_data[0], 0xA5u);
}

TEST(TransporterWrite, SmallPayload)
{
    TransporterPassThrough t;
    std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
    ssize_t ret = t.write(7, payload.data(), payload.size());
    ASSERT_EQ(ret, static_cast<ssize_t>(payload.size()));

    // 7 + 4 + 2 = 13 bytes
    ASSERT_EQ(t.written_data.size(), 13u);
    ASSERT_EQ(t.written_data[0], 0xA5u);

    // dataLength (bytes 1-2, LE) == 4
    uint16_t dlen = static_cast<uint16_t>(t.written_data[1]) |
                    (static_cast<uint16_t>(t.written_data[2]) << 8);
    ASSERT_EQ(dlen, 4u);

    // msgType (bytes 5-6, LE) == 7
    uint16_t mtype = static_cast<uint16_t>(t.written_data[5]) |
                     (static_cast<uint16_t>(t.written_data[6]) << 8);
    ASSERT_EQ(mtype, 7u);
}

TEST(TransporterWrite, CRC8IsValid)
{
    TransporterPassThrough t;
    std::vector<uint8_t> payload = {0xDE, 0xAD, 0xBE, 0xEF};
    t.write(1, payload.data(), payload.size());

    const auto & d = t.written_data;
    // CRC8 covers the first 4 bytes (head, dataLen_L, dataLen_H, seq)
    uint8_t expected = calculateCRC8(d.data(), 4);
    ASSERT_EQ(d[4], expected);
}

TEST(TransporterWrite, CRC16IsValid)
{
    TransporterPassThrough t;
    std::vector<uint8_t> payload = {0x11, 0x22, 0x33};
    t.write(5, payload.data(), payload.size());

    const auto & d = t.written_data;
    size_t body_len = d.size() - 2;
    uint16_t expected = calculateCRC16(d.data(), body_len);
    uint16_t actual   = static_cast<uint16_t>(d[body_len]) |
                        (static_cast<uint16_t>(d[body_len + 1]) << 8);
    ASSERT_EQ(actual, expected);
}

TEST(TransporterWrite, SequenceIncrements)
{
    TransporterPassThrough t;
    std::vector<uint8_t> payload = {0x00};
    t.write(1, payload.data(), payload.size());
    t.write(1, payload.data(), payload.size());
    t.write(1, payload.data(), payload.size());

    // Each frame is 10 bytes (7 hdr + 1 payload + 2 CRC16); seq is byte 3
    ASSERT_EQ(t.written_data[3],  0u);
    ASSERT_EQ(t.written_data[13], 1u);
    ASSERT_EQ(t.written_data[23], 2u);
}

// ── RX path tests: Transporter::read() ───────────────────────────────────────

TEST(TransporterRead, ValidFrame)
{
    TransporterPassThrough t;
    std::vector<uint8_t> payload = {0xAA, 0xBB, 0xCC};
    t.rx_data = make_dji_frame(3, 0, payload);

    uint8_t out[64];
    topic_id_size_t topic_id = 0xFFFF;
    ssize_t len = t.read(&topic_id, out, sizeof(out));

    ASSERT_EQ(len, static_cast<ssize_t>(payload.size()));
    ASSERT_EQ(topic_id, 3u);
    ASSERT_EQ(out[0], 0xAAu);
    ASSERT_EQ(out[1], 0xBBu);
    ASSERT_EQ(out[2], 0xCCu);
}

TEST(TransporterRead, EmptyPayloadFrame)
{
    TransporterPassThrough t;
    t.rx_data = make_dji_frame(99, 7, {});

    uint8_t out[64];
    topic_id_size_t topic_id = 0xFFFF;
    ssize_t len = t.read(&topic_id, out, sizeof(out));

    ASSERT_EQ(len, 0);
    ASSERT_EQ(topic_id, 99u);
}

TEST(TransporterRead, GarbageBeforeSOF)
{
    TransporterPassThrough t;
    auto frame = make_dji_frame(2, 0, {0x42});

    // Prepend garbage bytes that don't look like a valid SOF
    t.rx_data = {0x00, 0xFF, 0x12, 0x34};
    t.rx_data.insert(t.rx_data.end(), frame.begin(), frame.end());

    uint8_t out[64];
    topic_id_size_t topic_id = 0xFFFF;
    ssize_t len = t.read(&topic_id, out, sizeof(out));

    ASSERT_EQ(len, 1);
    ASSERT_EQ(topic_id, 2u);
    ASSERT_EQ(out[0], 0x42u);
}

TEST(TransporterRead, BadCRC8Rejected)
{
    TransporterPassThrough t;
    auto frame = make_dji_frame(1, 0, {0x01, 0x02});

    // Corrupt the CRC8 byte (index 4)
    frame[4] ^= 0xFF;
    t.rx_data = frame;

    uint8_t out[64];
    topic_id_size_t topic_id = 0xFFFF;
    ssize_t len = t.read(&topic_id, out, sizeof(out));

    ASSERT_LT(len, 0);
}

TEST(TransporterRead, BadCRC16Rejected)
{
    TransporterPassThrough t;
    auto frame = make_dji_frame(1, 0, {0x01, 0x02});

    // Corrupt the CRC16 MSB (last byte)
    frame.back() ^= 0xFF;
    t.rx_data = frame;

    uint8_t out[64];
    topic_id_size_t topic_id = 0xFFFF;
    ssize_t len = t.read(&topic_id, out, sizeof(out));

    ASSERT_LT(len, 0);
}

TEST(TransporterRead, PartialFrameReturnsNoData)
{
    TransporterPassThrough t;
    auto frame = make_dji_frame(5, 0, {0xDE, 0xAD, 0xBE, 0xEF});

    // Feed only the 7-byte header — no payload, no CRC16
    t.rx_data.assign(frame.begin(), frame.begin() + 7);

    uint8_t out[64];
    topic_id_size_t topic_id = 0xFFFF;
    ssize_t len = t.read(&topic_id, out, sizeof(out));

    ASSERT_LT(len, 0);
}

TEST(TransporterRead, TwoConsecutiveFrames)
{
    TransporterPassThrough t;
    auto frame1 = make_dji_frame(10, 0, {0x01});
    auto frame2 = make_dji_frame(20, 1, {0x02, 0x03});

    t.rx_data.insert(t.rx_data.end(), frame1.begin(), frame1.end());
    t.rx_data.insert(t.rx_data.end(), frame2.begin(), frame2.end());

    uint8_t out[64];
    topic_id_size_t topic_id;

    ssize_t len1 = t.read(&topic_id, out, sizeof(out));
    ASSERT_EQ(len1, 1);
    ASSERT_EQ(topic_id, 10u);
    ASSERT_EQ(out[0], 0x01u);

    // Second frame is already in the ring buffer — no new rx_data needed
    ssize_t len2 = t.read(&topic_id, out, sizeof(out));
    ASSERT_EQ(len2, 2);
    ASSERT_EQ(topic_id, 20u);
    ASSERT_EQ(out[0], 0x02u);
    ASSERT_EQ(out[1], 0x03u);
}

// ── Round-trip: write() → read() ─────────────────────────────────────────────

TEST(TransporterRoundTrip, WriteAndReadBack)
{
    TransporterPassThrough tx;
    TransporterPassThrough rx;

    std::vector<uint8_t> payload = {0x10, 0x20, 0x30, 0x40, 0x50};
    uint16_t msg_type = 42;

    tx.write(msg_type, payload.data(), payload.size());

    // Feed the raw bytes tx produced directly into rx's receive path
    rx.rx_data = tx.written_data;

    uint8_t out[64];
    topic_id_size_t topic_id;
    ssize_t len = rx.read(&topic_id, out, sizeof(out));

    ASSERT_EQ(len, static_cast<ssize_t>(payload.size()));
    ASSERT_EQ(topic_id, msg_type);
    for (size_t i = 0; i < payload.size(); ++i)
    {
        ASSERT_EQ(out[i], payload[i]) << "mismatch at byte " << i;
    }
}

TEST(TransporterRoundTrip, LargePayload)
{
    TransporterPassThrough tx(16384);
    TransporterPassThrough rx(16384);

    std::vector<uint8_t> payload(512);
    for (size_t i = 0; i < payload.size(); ++i) { payload[i] = static_cast<uint8_t>(i & 0xFF); }

    tx.write(7, payload.data(), payload.size());
    rx.rx_data = tx.written_data;

    std::vector<uint8_t> out(payload.size());
    topic_id_size_t topic_id;
    ssize_t len = rx.read(&topic_id, out.data(), out.size());

    ASSERT_EQ(len, static_cast<ssize_t>(payload.size()));
    ASSERT_EQ(topic_id, 7u);
    ASSERT_EQ(out, payload);
}
