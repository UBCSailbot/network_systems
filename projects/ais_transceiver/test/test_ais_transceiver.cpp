#include <boost/asio/io_service.hpp>
#include <chrono>
#include <random>
#include <thread>
#include <vector>

#include "ais_ship.h"
#include "gtest/gtest.h"
#include "mock_ais_sim.h"
#include "shared_constants.h"

constexpr uint32_t   NUM_SHIPS         = 20;
constexpr uint32_t   TICK_RATE_MS      = 200;
constexpr Vec2DFloat POLARIS_START_POS = {
  49.28397458822112, -123.6525841364974};  // some point in the Strait of Georgia

static std::random_device g_rd        = std::random_device();  // random number sampler
static uint32_t           g_rand_seed = g_rd();                // seed used for random number generation

class TestMockAisSim : public ::testing::Test
{
protected:
    TestMockAisSim() {}
    ~TestMockAisSim(){};
};

void checkAisShipInBounds(Vec2DFloat ais_ship_lat_lon, Vec2DFloat polaris_lat_lon)
{
    EXPECT_LT(std::abs(ais_ship_lat_lon[0] - polaris_lat_lon[0]), MAX_AIS_SHIP_DIST);
    EXPECT_LT(std::abs(ais_ship_lat_lon[1] - polaris_lat_lon[1]), MAX_AIS_SHIP_DIST);
    EXPECT_GT(std::abs(ais_ship_lat_lon[0] - polaris_lat_lon[0]), MIN_AIS_SHIP_DIST);
    EXPECT_GT(std::abs(ais_ship_lat_lon[1] - polaris_lat_lon[1]), MIN_AIS_SHIP_DIST);
}

void checkAisShipTickUpdateLimits(AisShip updated_ship, AisShip past_ship)
{
    EXPECT_LE(updated_ship.speed_, SPEED_UBND);
    EXPECT_GE(updated_ship.speed_, SPEED_LBND);
    EXPECT_LT(updated_ship.heading_, HEADING_UBND);
    EXPECT_GE(updated_ship.heading_, HEADING_LBND);

    EXPECT_LE(std::abs(updated_ship.speed_ - past_ship.speed_), MAX_SPEED_CHANGE);
    EXPECT_LE(std::abs(updated_ship.heading_ - past_ship.heading_), MAX_HEADING_CHANGE);
}

TEST_F(TestMockAisSim, TestBasic)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));
    boost::asio::io_service io;
    MockAisSim              sim        = MockAisSim(g_rand_seed, NUM_SHIPS, TICK_RATE_MS, POLARIS_START_POS, io);
    std::vector<AisShip>    curr_ships = sim.ships();
    std::thread             io_thread([&io]() { io.run(); });
    // Delay to give the sim time to process callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  //NOLINT clang-tidy(readability-magic-numbers)
    for (uint32_t i = 0; i < 3; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(TICK_RATE_MS));
        std::vector<AisShip> updated_ships = sim.ships();
        for (uint32_t i = 0; i < NUM_SHIPS; i++) {
            EXPECT_EQ(updated_ships[i].id_, curr_ships[i].id_);
            checkAisShipTickUpdateLimits(updated_ships[i], curr_ships[i]);
            checkAisShipInBounds(updated_ships[i].lat_lon_, POLARIS_START_POS);
        }
        curr_ships = updated_ships;
    }
    io.stop();
    io_thread.join();
}

TEST_F(TestMockAisSim, TestMovingPolaris)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));
    Vec2DFloat              polaris_lat_lon = POLARIS_START_POS;
    boost::asio::io_service io;
    MockAisSim              sim        = MockAisSim(g_rand_seed, NUM_SHIPS, TICK_RATE_MS, POLARIS_START_POS, io);
    std::vector<AisShip>    curr_ships = sim.ships();
    std::thread             io_thread([&io]() { io.run(); });
    // Delay to give the sim time to process callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  //NOLINT clang-tidy(readability-magic-numbers)
    for (uint32_t i = 0; i < 3; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(TICK_RATE_MS));
        std::vector<AisShip> updated_ships = sim.ships();
        for (uint32_t i = 0; i < NUM_SHIPS; i++) {
            EXPECT_EQ(updated_ships[i].id_, curr_ships[i].id_);
            checkAisShipTickUpdateLimits(updated_ships[i], curr_ships[i]);
            checkAisShipInBounds(updated_ships[i].lat_lon_, POLARIS_START_POS);
        }
        curr_ships = updated_ships;
        polaris_lat_lon[0] += 2 * MIN_AIS_SHIP_DIST;
        polaris_lat_lon[1] += 2 * MIN_AIS_SHIP_DIST;
        sim.updatePolarisPos(polaris_lat_lon);
    }
    io.stop();
    io_thread.join();
}
