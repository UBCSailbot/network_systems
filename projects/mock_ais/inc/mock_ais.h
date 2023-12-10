#pragma once

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <cstdint>
#include <mutex>
#include <random>

using Vec2DFloat = std::array<float, 2>;  // Convenience alias
namespace defaults
{
static constexpr float      MAX_HEADING_CHANGE = 5.0;     // Max degree change per tick
static constexpr float      MAX_SPEED_CHANGE   = 1.0;     // Min degree change per tick
static constexpr float      MIN_AIS_SHIP_DIST  = 0.001;   // Min 111m (at equator) distance of ais ships from Polaris
static constexpr float      MAX_AIS_SHIP_DIST  = 0.1;     // Max 11.1km (at equator) distance of ais ships from Polaris
static constexpr int        UPDATE_RATE_MS     = 500;     // Update frequency
static constexpr int        SEED               = 123456;  // Randomization seed
static constexpr int        NUM_SIM_SHIPS      = 20;      // Number of ais ships to simulate
static constexpr Vec2DFloat POLARIS_START_POS{
  49.28397458822112, -123.6525841364974};  // some point in the Strait of Georgia;
}  // namespace defaults

// TODO(): Add width and length
struct AisShip
{
    Vec2DFloat lat_lon_;
    float      speed_;
    float      heading_;
    uint32_t   id_;
};

/**
 * @brief Extra per ship simulation parameters
 */
struct SimShipConfig
{
    float max_heading_change_;  // Max degree change per tick
    float max_speed_change_;    // Min degree change per tick
    float max_ship_dist_;       // Maximum distance from Polaris (Difference b/w lats and lons)
    float min_ship_dist_;       // Minimum distance from Poalris (Difference b/w lats and lons)
};

class MockAisShip : public AisShip
{
public:
    /**
     * @brief Construct a new Mock Ais Ship object
     *
     * @param seed            Random seed
     * @param id              ID of the new sim ship
     * @param polaris_lat_lon Position of Poalris
     * @param config          Sim ship constraints
     */
    MockAisShip(uint32_t seed, uint32_t id, Vec2DFloat polaris_lat_lon, SimShipConfig config);

    /**
     * @brief Update the AIS Ship instance
     *
     * @param polaris_lat_lon Current position of Polaris
     */
    void tick(const Vec2DFloat & polaris_lat_lon);

private:
    std::mt19937  mt_rng_;  // Random number generator
    SimShipConfig config_;  // Sim ship constraints
};

/**
 * Simulate Ais Ships
 */
class MockAis
{
public:
    /**
     * @brief Construct a new Mock AIS simulation
     *
     * @param seed            Random seeds
     * @param num_ships       Number of sim ships to spawn
     * @param polaris_lat_lon Position of Polaris
     */
    MockAis(uint32_t seed, uint32_t num_ships, Vec2DFloat polaris_lat_lon);

    /**
     * @brief Construct a new Mock AIS simulation
     *
     * @param seed            Random seeds
     * @param num_ships       Number of sim ships to spawn
     * @param polaris_lat_lon Position of Polaris
     * @param config          Extra sim ship constraints
     */
    MockAis(uint32_t seed, uint32_t num_ships, Vec2DFloat polaris_lat_lon, SimShipConfig config);

    /**
     * @brief Get the current AIS ships
     *
     * @return A vector of AisShip objects
     */
    std::vector<AisShip> ships();

    /**
     * @brief Update the current position of Polaris
     *
     * @param lat_lon Polaris' position
     */
    void updatePolarisPos(const Vec2DFloat & lat_lon);

    /**
     * @brief Update every simulated AIS ship
     *
     */
    void tick();

private:
    Vec2DFloat               polaris_lat_lon_;  // Polaris' current position
    std::vector<MockAisShip> ships_;            // Vector of all simulated Ais ships
};
