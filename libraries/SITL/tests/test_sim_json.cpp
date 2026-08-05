/*
    HIL-F24-N regression tests for the JSON+PPP boot watchdog fix.

    HIL_F24_M_json_ppp_reboot_loop_diagnosis.md found that on real hardware
    (CONFIG_HAL_BOARD != HAL_BOARD_SITL), JSON::recv_fdm() ran synchronously
    on the main vehicle thread and blocked forever waiting for a UDP reply,
    starving the ChibiOS watchdog pat and forcing a hardfault/reset after
    ~1.8-2.1s whenever no reply arrived in time. HIL-F24-N bounded that wait
    to JSON_HW_RECV_TIMEOUT_MS (see SIM_JSON.cpp) by factoring the bounded
    receive attempt out into JSON::recv_fdm_bounded() -- an unconditionally
    compiled method (unlike the `#if CONFIG_HAL_BOARD != HAL_BOARD_SITL`
    branch that calls it) so it can be exercised directly here even though
    this test binary is always built for CONFIG_HAL_BOARD == HAL_BOARD_SITL.

    These tests prove, over real loopback UDP sockets:
      1. recv_fdm_bounded() gives up within its bound when no reply ever
         arrives, instead of blocking forever (RecvFdmBoundedTimesOut...).
      2. ...without busy-looping: CPU time consumed is far less than the
         wall-clock time elapsed, showing the wait blocks on the socket's
         own timeout rather than spinning (same test).
      3. recv_fdm_bounded() returns as soon as data is available, well
         inside the bound (RecvFdmBoundedReturnsPromptly...).
      4. recv_fdm()'s desktop SITL branch (the `#else` in SIM_JSON.cpp,
         left byte-for-byte unchanged by HIL-F24-N) still parses a valid
         reply correctly, via the parsing tail shared by both platforms
         (RecvFdmParsesQueuedValidReply...).
*/
#include <AP_gtest.h>

#include <SITL/SIM_JSON.h>
#include <AP_HAL/utility/Socket.h>

#include <string.h>
#include <time.h>
#include <string>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace SITL;

namespace SITL {

// Minimal friend-only accessor (mirrors SIM_JSON.h's `friend class
// JSONTestAccess;`) giving these tests just enough private access to drive
// JSON end-to-end over a real socket, without widening JSON's public API.
class JSONTestAccess {
public:
    // JSON::sock is SocketAPM_native on this board (CONFIG_HAL_BOARD ==
    // HAL_BOARD_SITL) and SocketAPM on real hardware; return by auto& so
    // this accessor compiles against whichever type is active.
    static auto sock(JSON &j) -> decltype(j.sock)& { return j.sock; }
    static ssize_t recv_fdm_bounded(JSON &j, uint32_t timeout_ms) { return j.recv_fdm_bounded(timeout_ms); }
    static void recv_fdm(JSON &j, const struct sitl_input &input) { j.recv_fdm(input); }
    static double timestamp_s(JSON &j) { return j.state.timestamp_s; }
};

} // namespace SITL

// Wall-clock elapsed time, milliseconds, monotonic.
static uint64_t now_wall_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

// This process's CPU time consumed so far, milliseconds. A busy-loop spends
// CPU time roughly equal to wall-clock time elapsed; a real blocking wait on
// a socket timeout spends close to none.
static uint64_t now_cpu_ms()
{
    return (uint64_t)((uint64_t)clock() * 1000ULL / CLOCKS_PER_SEC);
}

TEST(JSON, RecvFdmBoundedTimesOutWithoutBusyLooping)
{
    JSON j("json:127.0.0.1");
    ASSERT_TRUE(JSONTestAccess::sock(j).bind("127.0.0.1", 39101));

    const uint64_t wall_start = now_wall_ms();
    const uint64_t cpu_start = now_cpu_ms();

    // Nothing is ever sent to this socket.
    const ssize_t ret = JSONTestAccess::recv_fdm_bounded(j, 200);

    const uint64_t wall_elapsed = now_wall_ms() - wall_start;
    const uint64_t cpu_elapsed = now_cpu_ms() - cpu_start;

    // No reply arrived, so recv_fdm_bounded() must give up rather than
    // block forever -- this is exactly the behavior whose absence, prior
    // to HIL-F24-N, starved the ChibiOS watchdog pat.
    EXPECT_LE(ret, 0);

    // Bound enforced: two 100ms-timeout recv() attempts (~200ms), not an
    // unbounded wait. Generous upper bound to absorb CI scheduling jitter.
    EXPECT_GE(wall_elapsed, 180u);
    EXPECT_LE(wall_elapsed, 1000u);

    // Not a busy-loop: a spin-wait would consume CPU time close to
    // wall_elapsed; blocking on the socket's own poll()-based timeout
    // consumes close to none.
    EXPECT_LT(cpu_elapsed, wall_elapsed / 2 + 5);
}

TEST(JSON, RecvFdmBoundedReturnsPromptlyWhenReplyAlreadyQueued)
{
    JSON j("json:127.0.0.1");
    ASSERT_TRUE(JSONTestAccess::sock(j).bind("127.0.0.1", 39102));

    SocketAPM_native peer(true);
    const char *msg = "not parsed by recv_fdm_bounded() -- only recv_fdm() parses";
    ASSERT_GT(peer.sendto(msg, strlen(msg), "127.0.0.1", 39102), 0);

    const uint64_t wall_start = now_wall_ms();
    const ssize_t ret = JSONTestAccess::recv_fdm_bounded(j, 200);
    const uint64_t wall_elapsed = now_wall_ms() - wall_start;

    EXPECT_GT(ret, 0);
    // Must return as soon as data is available, not wait out the full bound.
    EXPECT_LT(wall_elapsed, 150u);
}

TEST(JSON, RecvFdmParsesQueuedValidReplyOnUnchangedSitlPath)
{
    JSON j("json:127.0.0.1");
    ASSERT_TRUE(JSONTestAccess::sock(j).bind("127.0.0.1", 39103));

    SocketAPM_native peer(true);
    // Same wire format produced by the real SR75 bench responder's
    // SimState.to_json_bytes() (Tools/autotest/sr75_hil_layer2/sim_json/
    // sr75_sim_json_responder.py), trimmed to just the mandatory fields
    // JSON::parse_sensors() requires (timestamp, imu/gyro, imu/accel_body,
    // velocity) plus attitude (parse_sensors also requires euler or
    // quaternion attitude).
    //
    // recv_fdm() only parses a message once a *second* '\n'-terminated
    // message has arrived after it (see the p1/p2 memrchr logic below the
    // #if/#endif in SIM_JSON.cpp -- this lets the real protocol tell a
    // complete record apart from one still being written). So two copies
    // are sent back-to-back in a single datagram; recv_fdm() then parses
    // the first of the two.
    const char *payload =
        "{\"timestamp\":12.5,"
        "\"imu\":{\"gyro\":[0.1,0.2,0.3],\"accel_body\":[0.0,0.0,-9.8]},"
        "\"velocity\":[1.0,2.0,3.0],"
        "\"attitude\":[0.0,0.0,0.0]}\n";
    std::string two_records = std::string(payload) + std::string(payload);
    ASSERT_GT(peer.sendto(two_records.data(), two_records.size(), "127.0.0.1", 39103), 0);

    struct sitl_input input {};
    // Queued data means the very first sock.recv() inside recv_fdm() (the
    // code above the `#if CONFIG_HAL_BOARD != HAL_BOARD_SITL` block, shared
    // by both platforms and untouched by HIL-F24-N) returns immediately
    // with ret > 0, so this exercises the `#else` (desktop SITL) branch,
    // left byte-for-byte unchanged by HIL-F24-N, plus the shared parsing
    // tail common to both platforms.
    JSONTestAccess::recv_fdm(j, input);

    EXPECT_FLOAT_EQ((float)JSONTestAccess::timestamp_s(j), 12.5f);
}

AP_GTEST_MAIN()
