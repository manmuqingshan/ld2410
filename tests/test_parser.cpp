// Native host-side unit test for ld2410::parse_data_frame_().
// Feeds known-good frames (from the HLK-LD2410C V1.00 protocol document
// in docs/HLK-LD2410C_protocol.md) through the public read() API via a
// mock Stream, and asserts that the resulting field values match the
// protocol specification.
//
// Build & run:  bash tests/run.sh   (from the repo root)
//
// On the host ESP32 is not defined, so the FreeRTOS task path and the
// portMUX critical sections are compiled out. The test exercises only
// the parser, which is the surface this branch actually changed.

#include <Arduino.h>
#include <ld2410.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include <cassert>
#include <initializer_list>

class MockSerial : public Stream {
    std::vector<uint8_t> q_;
    size_t pos_ = 0;
public:
    void inject(std::initializer_list<uint8_t> bytes) {
        q_.insert(q_.end(), bytes.begin(), bytes.end());
    }
    void inject(const std::vector<uint8_t>& bytes) {
        q_.insert(q_.end(), bytes.begin(), bytes.end());
    }
    int available() override { return (int)(q_.size() - pos_); }
    int read() override {
        if (pos_ >= q_.size()) return -1;
        return q_[pos_++];
    }
    void clear() { q_.clear(); pos_ = 0; }
};

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
} while (0)

#define CHECK_EQ(a, b) do { \
    auto _a = (a); auto _b = (b); \
    if (!(_a == _b)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s == %s : got %lld vs %lld\n", \
                     __FILE__, __LINE__, #a, #b, (long long)_a, (long long)_b); \
        failures++; \
    } \
} while (0)

// Helper: pump the parser by calling read() until all queued bytes are drained.
static void drain(ld2410& r, MockSerial& s) {
    while (s.available() > 0) {
        r.read();
    }
}

// ---------------------------------------------------------------------------
// Test 1: basic frame from protocol §2.3.2 (Table 12 example).
//   F4 F3 F2 F1 | 0D 00 | 02 AA 02 51 00 00 00 00 3B 00 00 55 00 | F8 F7 F6 F5
// Decoded:
//   data_type=0x02 (basic), target_status=0x02,
//   moving distance=0x0051=81 cm, moving energy=0,
//   stationary distance=0, stationary energy=0x3B=59,
//   detection distance=0
// ---------------------------------------------------------------------------
static void test_basic_frame() {
    std::printf("test_basic_frame ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,             // header
        0x0D, 0x00,                         // intra-frame data length = 13
        0x02, 0xAA,                         // data_type basic, head 0xAA
        0x02,                               // target status
        0x51, 0x00,                         // moving distance LE = 81
        0x00,                               // moving energy
        0x00, 0x00,                         // stationary distance = 0
        0x3B,                               // stationary energy = 59
        0x00, 0x00,                         // detection distance = 0
        0x55, 0x00,                         // tail + calibration
        0xF8, 0xF7, 0xF6, 0xF5              // footer
    });
    drain(r, s);

    CHECK(r.presenceDetected());                     // target_status != 0
    CHECK(!r.movingTargetDetected());                // moving energy 0 -> not detected
    // Note: r.stationaryTargetDetected() is intentionally NOT asserted.
    // The doc's contrived basic-frame example has stationary_distance=0,
    // and the helper requires both distance > 0 and energy > 0. Parser is
    // correct; the doc example is just synthetic. We assert raw fields.
    CHECK_EQ((int)r.movingTargetDistance(), 81);
    CHECK_EQ((int)r.movingTargetEnergy(), 0);
    CHECK_EQ((int)r.stationaryTargetDistance(), 0);
    CHECK_EQ((int)r.stationaryTargetEnergy(), 59);
    CHECK_EQ((int)r.detectionDistance(), 0);
    CHECK(!r.engineeringRetrieved());                // no eng frame yet
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 2: engineering frame from protocol §2.3.2 (Table 14 example).
//   F4 F3 F2 F1 | 23 00 | 01 AA 03 1E 00 3C 00 00 39 00 00
//                          08 08
//                          3C 22 05 03 03 04 03 06 05
//                          00 00 39 10 13 06 06 08 04
//                          03 05    (retain bytes)
//                          55 00 |
//   F8 F7 F6 F5
// Decoded: target_status=0x03, moving=30 cm @ 60, stationary=0 @ 57,
//   detection=0; per-gate motion energies = [60, 34, 5, 3, 3, 4, 3, 6, 5];
//   per-gate stationary energies = [0, 0, 57, 16, 19, 6, 6, 8, 4].
// Pre-fix: the strict 0x02 check rejected this frame and the buffer-size
// guard (LD2410_MAX_FRAME_LENGTH=40) would have dropped it before parsing.
// ---------------------------------------------------------------------------
static void test_engineering_frame() {
    std::printf("test_engineering_frame ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x23, 0x00,                                // intra length = 35
        0x01, 0xAA,                                // engineering, head
        0x03,                                      // target status: both
        0x1E, 0x00,                                // moving dist = 30
        0x3C,                                      // moving energy = 60
        0x00, 0x00,                                // stationary dist = 0
        0x39,                                      // stationary energy = 57
        0x00, 0x00,                                // detection dist = 0
        0x08, 0x08,                                // max moving N=8, max stationary N=8
        0x3C, 0x22, 0x05, 0x03, 0x03, 0x04, 0x03, 0x06, 0x05,  // motion gate 0..8
        0x00, 0x00, 0x39, 0x10, 0x13, 0x06, 0x06, 0x08, 0x04,  // stationary gate 0..8
        0x03, 0x05,                                // retain (M=2)
        0x55, 0x00,                                // tail + cal at idx 39, 40
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);

    CHECK_EQ((int)r.movingTargetDistance(), 30);
    CHECK_EQ((int)r.movingTargetEnergy(), 60);
    CHECK_EQ((int)r.stationaryTargetDistance(), 0);
    CHECK_EQ((int)r.stationaryTargetEnergy(), 57);
    CHECK_EQ((int)r.detectionDistance(), 0);

    // Per-gate energies: this is the new surface that pre-fix did not exist.
    int expected_motion[9]     = {60, 34,  5,  3,  3,  4,  3,  6,  5};
    int expected_stationary[9] = { 0,  0, 57, 16, 19,  6,  6,  8,  4};
    for (uint8_t g = 0; g < 9; g++) {
        CHECK_EQ((int)r.movingEnergyAtGate(g), expected_motion[g]);
        CHECK_EQ((int)r.stationaryEnergyAtGate(g), expected_stationary[g]);
    }
    // Out-of-range gate index must return 0, not out-of-bounds-read.
    CHECK_EQ((int)r.movingEnergyAtGate(9), 0);
    CHECK_EQ((int)r.stationaryEnergyAtGate(255), 0);

    CHECK(r.engineeringRetrieved());
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 3: malformed frame (bad data_type) must be rejected.
// Same length as a basic frame but data_type = 0x99 (neither 0x01 nor 0x02).
// ---------------------------------------------------------------------------
static void test_invalid_data_type() {
    std::printf("test_invalid_data_type ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x0D, 0x00,
        0x99, 0xAA,                                // INVALID data_type
        0x02, 0x51, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00,
        0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);

    // Fields must remain at their initial zero values - no spurious update.
    CHECK_EQ((int)r.movingTargetDistance(), 0);
    CHECK_EQ((int)r.stationaryTargetEnergy(), 0);
    CHECK_EQ((int)r.detectionDistance(), 0);
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 4: bad trailer byte must be rejected (anchor on dynamic position).
// Use a basic frame but corrupt the 0x55 tail.
// ---------------------------------------------------------------------------
static void test_invalid_trailer() {
    std::printf("test_invalid_trailer ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x0D, 0x00,
        0x02, 0xAA,
        0x02, 0x51, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00,
        0x77, 0x00,                                // bogus tail (must be 0x55 0x00)
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);

    CHECK_EQ((int)r.movingTargetDistance(), 0);    // not updated
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 5: two consecutive frames - basic then engineering - should reuse
// the same library instance correctly and reflect the latest values.
// ---------------------------------------------------------------------------
static void test_sequential_frames() {
    std::printf("test_sequential_frames ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // First: a basic frame with moving distance 100 cm
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1, 0x0D, 0x00,
        0x02, 0xAA, 0x01, 0x64, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.movingTargetDistance(), 100);
    CHECK_EQ((int)r.movingTargetEnergy(), 50);
    CHECK(!r.engineeringRetrieved());

    // Then: engineering frame, moving distance 200, gate 5 motion energy 42
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1, 0x23, 0x00,
        0x01, 0xAA, 0x01,
        0xC8, 0x00,                                // moving dist = 200
        0x14,                                      // moving energy = 20
        0x00, 0x00, 0x00, 0x00, 0x00,              // stationary + detection
        0x08, 0x08,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x00, 0x00,  // gate 5 motion = 0x2A=42
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.movingTargetDistance(), 200);
    CHECK_EQ((int)r.movingTargetEnergy(), 20);
    CHECK_EQ((int)r.movingEnergyAtGate(5), 42);
    CHECK(r.engineeringRetrieved());
    std::printf("ok\n");
}

int main() {
    test_basic_frame();
    test_engineering_frame();
    test_invalid_data_type();
    test_invalid_trailer();
    test_sequential_frames();

    if (failures == 0) {
        std::printf("\nALL TESTS PASS\n");
        return 0;
    }
    std::printf("\n%d FAILURE(S)\n", failures);
    return 1;
}
