#include <gtest/gtest.h>

#include <boost/asio/io_service.hpp>
#include <chrono>
#include <random>
#include <thread>
#include <vector>

#include "cmn_hdrs/shared_constants.h"
#include "mock_ais.h"

using defaults::MAX_AIS_SHIP_DIST;
using defaults::MAX_HEADING_CHANGE;
using defaults::MAX_SPEED_CHANGE;
using defaults::MIN_AIS_SHIP_DIST;

constexpr uint32_t   NUM_SHIPS       = 50;
constexpr uint32_t   NUM_TEST_CYCLES = 100;
constexpr Vec2DFloat POLARIS_START_POS{49.28397458822112, -123.6525841364974};  // some point in the Strait of Georgia

static std::random_device g_rd        = std::random_device();  // random number sampler
static uint32_t           g_rand_seed = g_rd();                // seed used for random number generation

class TestMockAisSim : public ::testing::Test
{
protected:
    TestMockAisSim() {}
    ~TestMockAisSim(){};
};

/**
 * @brief Verify that an Ais ship is within acceptable distance from Polaris
 *
 * @param ais_ship_lat_lon ais ship position
 * @param polaris_lat_lon  polaris position
 */
void checkAisShipInBounds(Vec2DFloat ais_ship_lat_lon, Vec2DFloat polaris_lat_lon)
{
    EXPECT_LT(std::abs(ais_ship_lat_lon[0] - polaris_lat_lon[0]), MAX_AIS_SHIP_DIST);
    EXPECT_LT(std::abs(ais_ship_lat_lon[1] - polaris_lat_lon[1]), MAX_AIS_SHIP_DIST);
    EXPECT_GT(std::abs(ais_ship_lat_lon[0] - polaris_lat_lon[0]), MIN_AIS_SHIP_DIST);
    EXPECT_GT(std::abs(ais_ship_lat_lon[1] - polaris_lat_lon[1]), MIN_AIS_SHIP_DIST);
}

/**
 * @brief Verify that ais ships operate within specified constraints
 *
 * @param updated_ship ais ship instance
 * @param past_ship    same ais ship instance but saved from the previous tick
 */
void checkAisShipTickUpdateLimits(AisShip updated_ship, AisShip past_ship)
{
    EXPECT_LE(updated_ship.speed_, SPEED_UBND);
    EXPECT_GE(updated_ship.speed_, SPEED_LBND);
    EXPECT_LT(updated_ship.heading_, HEADING_UBND);
    EXPECT_GE(updated_ship.heading_, HEADING_LBND);

    if (std::abs(updated_ship.speed_ - past_ship.speed_) > MAX_SPEED_CHANGE) {
        if (updated_ship.speed_ < past_ship.speed_) {
            EXPECT_LE(updated_ship.speed_ + SPEED_UBND - past_ship.speed_, MAX_SPEED_CHANGE);
        } else {
            EXPECT_LE(past_ship.speed_ + SPEED_UBND - updated_ship.speed_, MAX_SPEED_CHANGE);
        }
    } else {
        // Passes check
    }
    if (std::abs(updated_ship.heading_ - past_ship.heading_) > MAX_HEADING_CHANGE) {
        if (updated_ship.heading_ < past_ship.heading_) {
            EXPECT_LE(updated_ship.heading_ + HEADING_UBND - past_ship.heading_, MAX_HEADING_CHANGE);
        } else {
            EXPECT_LE(past_ship.heading_ + HEADING_UBND - updated_ship.heading_, MAX_HEADING_CHANGE);
        }
    } else {
        // Passes check
    }

    EXPECT_LE(updated_ship.rot_, ROT_UBND);
    EXPECT_GE(updated_ship.rot_, ROT_LBND);
}

/**
 * @brief Test basic operation when Polaris is not moving
 *
 */
TEST_F(TestMockAisSim, TestBasic)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));
    MockAis              sim        = MockAis(g_rand_seed, NUM_SHIPS, POLARIS_START_POS);
    std::vector<AisShip> curr_ships = sim.ships();
    for (uint32_t i = 0; i < NUM_TEST_CYCLES; i++) {
        sim.tick();
        std::vector<AisShip> updated_ships = sim.ships();
        for (uint32_t i = 0; i < NUM_SHIPS; i++) {
            EXPECT_EQ(updated_ships[i].id_, curr_ships[i].id_);
            checkAisShipTickUpdateLimits(updated_ships[i], curr_ships[i]);
            checkAisShipInBounds(updated_ships[i].lat_lon_, POLARIS_START_POS);
        }
        curr_ships = updated_ships;
    }
}

/**
 * @brief Test operation when Polaris is moving
 *
 */
TEST_F(TestMockAisSim, TestMovingPolaris)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));
    Vec2DFloat           polaris_lat_lon = POLARIS_START_POS;
    MockAis              sim             = MockAis(g_rand_seed, NUM_SHIPS, POLARIS_START_POS);
    std::vector<AisShip> curr_ships      = sim.ships();
    for (uint32_t i = 0; i < NUM_TEST_CYCLES; i++) {
        sim.tick();
        std::vector<AisShip> updated_ships = sim.ships();
        for (uint32_t i = 0; i < NUM_SHIPS; i++) {
            EXPECT_EQ(updated_ships[i].id_, curr_ships[i].id_);
            checkAisShipTickUpdateLimits(updated_ships[i], curr_ships[i]);
            checkAisShipInBounds(updated_ships[i].lat_lon_, polaris_lat_lon);
        }
        curr_ships = updated_ships;
        polaris_lat_lon[0] += 2 * MIN_AIS_SHIP_DIST;
        polaris_lat_lon[1] += 2 * MIN_AIS_SHIP_DIST;
        sim.updatePolarisPos(polaris_lat_lon);
    }
}
