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
#include <vector>

#include <gtest/gtest.h>

#include "ros2_serial_example/transporter.hpp"

// ── Concrete in-process transporter for testing ──────────────────────────────
// node_write() captures bytes into a vector so tests can inspect the wire data.
// node_read() pushes a pre-loaded rx_data_ vector into the ring buffer.

class TransporterPassThrough final : public ros2_to_serial_bridge::transport::Transporter
{
public:
    explicit TransporterPassThrough(size_t ring_buffer_size = 1024)
    : Transporter(ring_buffer_size) {}

    // Data written by the Transporter's write() path ends up here.
    std::vector<uint8_t> written_data;

    // Bytes to be fed into the ring buffer on the next node_read() call.
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
        ssize_t n = ringbuf_.write(rx_data.data(), rx_data.size());
        rx_data.clear();
        return n;
    }

    bool fds_OK() override { return true; }
};

// ── Helpers ───────────────────────────────────────────────────────────────────

// Build a valid DJI frame by hand so we can feed it to find_and_copy_message.
static std::vector<uint8_t> make_dji_frame(uint16_t msg_type,
                                            uint8_t seq,
                                            const std::vector<uint8_t> & payload)
{
    // Header fields
    uint8_t  head       = 0xA5;
    uint16_t dataLength = static_cast<uint16_t>(payload.size());

    // Build raw header bytes (head, dataLen_L, dataLen_H, seq) for CRC8
    uint8_t pre_crc[4];
    pre_crc[0] = head;
    pre_crc[1] = static_cast<uint8_t>(dataLength & 0xFF);
    pre_crc[2] = static_cast<uint8_t>((dataLength >> 8) & 0xFF);
    pre_crc[3] = seq;

    uint8_t crc8 = calculateCRC8(pre_crc, 4);

    std::vector<uint8_t> frame;
    frame.push_back(head);
    frame.push_back(pre_crc[1]);   // dataLen L
    frame.push_back(pre_crc[2]);   // dataLen H
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

// ── Tests: Transporter::write() (TX path) ─────────────────────────────────────

TEST(TransporterWrite, EmptyPayload)
{
    TransporterPassThrough t;
    ssize_t ret = t.write(42, nullptr, 0);
    ASSERT_EQ(ret, 0);

    // Frame should be 7 (header) + 0 (payload) + 2 (CRC16) = 9 bytes
    ASSERT_EQ(t.written_data.size(), 9u);
    ASSERT_EQ(t.written_data[0], 0xA5u);  // SOF
}

TEST(TransporterWrite, SmallPayload)
{
    TransporterPassThrough t;
    std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
    ssize_t ret = t.write(7, payload.data(), payload.size());
    ASSERT_EQ(ret, static_cast<ssize_t>(payload.size()));

    // Total: 7 + 4 + 2 = 13 bytes
    ASSERT_EQ(t.written_data.size(), 13u);
    ASSERT_EQ(t.written_data[0], 0xA5u);

    // dataLength field (bytes 1-2, LE) == 4
    uint16_t dlen = static_cast<uint16_t>(t.written_data[1]) |
                    (static_cast<uint16_t>(t.written_data[2]) << 8);
    ASSERT_EQ(dlen, 4u);

    // msgType field (bytes 5-6, LE) == 7
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
    // CRC8 should cover first 4 bytes (head, dataLen_L, dataLen_H, seq)
    uint8_t expected_crc8 = calculateCRC8(d.data(), 4);
    ASSERT_EQ(d[4], expected_crc8);
}

TEST(TransporterWrite, CRC16IsValid)
{
    TransporterPassThrough t;
    std::vector<uint8_t> payload = {0x11, 0x22, 0x33};
    t.write(5, payload.data(), payload.size());

    const auto & d = t.written_data;
    size_t frame_body_len = d.size() - 2;  // everything before CRC16 bytes
    uint16_t expected_crc16 = calculateCRC16(d.data(), frame_body_len);
    uint16_t written_crc16 = static_cast<uint16_t>(d[frame_body_len]) |
                             (static_cast<uint16_t>(d[frame_body_len + 1]) << 8);
    ASSERT_EQ(written_crc16, expected_crc16);
}

TEST(TransporterWrite, SequenceIncrements)
{
    TransporterPassThrough t;
    std::vector<uint8_t> payload = {0x00};
    t.write(1, payload.data(), payload.size());
    t.write(1, payload.data(), payload.size());
    t.write(1, payload.data(), payload.size());

    // seq is byte 3 of each 10-byte frame (7 hdr + 1 payload + 2 crc)
    ASSERT_EQ(t.written_data[3],  0u);
    ASSERT_EQ(t.written_data[13], 1u);
    ASSERT_EQ(t.written_data[23], 2u);
}

// ── Tests: Transporter::read() (RX path) ──────────────────────────────────────

TEST(TransporterRead, ValidFrame)
{
    TransporterPassThrough t;
    std::vector<uint8_t> payload = {0xAA, 0xBB, 0xCC};
    uint16_t msg_type = 3;

    t.rx_data = make_dji_frame(msg_type, 0, payload);

    uint8_t out[64];
    topic_id_size_t topic_id = 0xFFFF;
    ssize_t len = t.read(&topic_id, out, sizeof(out));

    ASSERT_EQ(len, static_cast<ssize_t>(payload.size()));
    ASSERT_EQ(topic_id, msg_type);
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
    std::vector<uint8_t> payload = {0x42};
    auto frame = make_dji_frame(2, 0, payload);

    // Prepend garbage bytes
    t.rx_data.insert(t.rx_data.end(), {0x00, 0xFF, 0x12, 0x34});
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

    // Corrupt the last byte (CRC16 MSB)
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

    // Feed only the header (no payload, no CRC16)
    t.rx_data.assign(frame.begin(), frame.begin() + 7);

    uint8_t out[64];
    topic_id_size_t topic_id = 0xFFFF;
    ssize_t len = t.read(&topic_id, out, sizeof(out));

    ASSERT_LT(len, 0);  // -ENODATA
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

    ssize_t len2 = t.read(&topic_id, out, sizeof(out));
    ASSERT_EQ(len2, 2);
    ASSERT_EQ(topic_id, 20u);
    ASSERT_EQ(out[0], 0x02u);
    ASSERT_EQ(out[1], 0x03u);
}

// ── Tests: round-trip write → read ────────────────────────────────────────────

TEST(TransporterRoundTrip, WriteAndReadBack)
{
    TransporterPassThrough tx;
    TransporterPassThrough rx;

    std::vector<uint8_t> payload = {0x10, 0x20, 0x30, 0x40, 0x50};
    uint16_t msg_type = 42;

    tx.write(msg_type, payload.data(), payload.size());

    // Feed the bytes tx produced directly into rx's receive path
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
    TransporterPassThrough tx;
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
