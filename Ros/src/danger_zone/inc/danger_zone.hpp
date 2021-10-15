/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#pragma once

#include <memory>
#include <string>
#include <vector>

/**
 * @brief Interface used to construct a collection of obstacle messages from inside Gaia rules.
 */

class obstacles_t
{
public:
    /**
     * Class factory, this is the only method allowed for obtaining an instance
     * of this class within a Gaia rule.
     *
     * @return std::shared_ptr<obstacles_t>
     * @throws
     * @exceptsafe yes
     */
    static std::shared_ptr<obstacles_t> new_instance();

    virtual ~obstacles_t() = default;

    /**
     * Call this from within a Gaia rule to add an obstacle message.
     *
     * @param[in] std::string : type_name : the name of the class of object detected
     * @param[in] uint8_t : roi : the roi code
     * @param[in] uint8_t : direction : the direction code
     * @param[in] double : pos_x : the object position X
     * @param[in] double : pos_y : the object position Y
     * @param[in] double : pos_z : the object position Z
     * @param[in] double : size_x : the object size X
     * @param[in] double : size_y : the object size Y
     * @param[in] double : size_z : the object size Z
     * @param[in] double : orient_x : the object orientation quaternion X
     * @param[in] double : orient_y : the object orientation quaternion Y
     * @param[in] double : orient_z : the object orientation quaternion Y
     * @param[in] double : orient_w : the object orientation quaternion W
     * @return void
     * @throws
     * @exceptsafe yes
     */
    virtual void add(
        std::string type_name, uint8_t roi, uint8_t direction,
        double pos_x, double pos_y, double pos_z,
        double size_x, double size_y, double size_z,
        double orient_x, double orient_y, double orient_z, double orient_w)
        = 0;
};

/**
 * @brief Interface used to expose danger_zone core methods to Gaia rules.
 */

class danger_zone_t
{
protected:
// 1) We ignore -Wc++17-extensions here to quiet down the Gaia translator.
// 2) We mention GCC but this works for Clang as well.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc++17-extensions"
    // The pointer to the live instance of the campus core class.
    // TODO: This is not an example to good coding practice, need to make safe and modern.
    inline static danger_zone_t* danger_zone_ptr = nullptr;
#pragma GCC diagnostic pop

public:
    /**
     * Class factory, this is the only method allowed for obtaining an instance
     * of this class within a Gaia rule.
     *
     * @return danger_zone_t*
     * @throws
     * @exceptsafe yes
     */
    static danger_zone_t* get_instance()
    {
        return danger_zone_ptr;
    }

    /**
     * Call this from within a Gaia rule to send a ROS2 obstacleArray message.
     *
     * @param[in] std::shared_ptr<obstacles_t> : obstacles : the collection of obstacles messages
     * @param[in] std::string : frame_id : the ROS frame name
     * @param[in] int32_t : sec : the time in seconds
     * @param[in] uint32_t : nsec : the number of nanoseconds since sec
     * @return void
     * @throws
     * @exceptsafe yes
     */
    virtual void send_obstacle_array_message(
        std::shared_ptr<obstacles_t> obstacles,
        std::string frame_id, int32_t sec, uint32_t nsec)
        = 0;

    /**
     * Call this from within a ruleset rule to trigger a log event.
     *
     * @param[in] int start_sec
     * @param[in] uint32_t start_nsec
     * @param[in] int end_sec
     * @param[in] uint32_t end_nsec
     * @param[in] std::string file_name
     * @param[in] std::vector<std::string>topics
     * @return void
     * @throws
     * @exceptsafe yes
     */
    virtual void trigger_log(int start_sec, uint32_t start_nsec,
        int end_sec, uint32_t end_nsec, std::string file_name,
        std::vector<std::string>topics) = 0;

    /**
     * Call this from within a ruleset rule to trigger a log event.
     *
     * @param[in] int seconds_past
     * @param[in] int seconds_forward
     * @param[in] std::string file_name
     * @param[in] std::vector<std::string>topics
     * @return void
     * @throws
     * @exceptsafe yes
     */
    virtual void trigger_log(int seconds_past, int seconds_forward,
        std::string file_name, std::vector<std::string>topics) = 0;

    /**
     * Constructor.
     *
     * @throws
     * @exceptsafe yes
     */
    danger_zone_t() = default;

    /**
     * Destructor, to get rid of annoying build warnings.
     *
     * @throws
     * @exceptsafe yes
     */
    virtual ~danger_zone_t() = default;

    /**
     * Unit test, needs to be implemented.
     *
     * @throws
     * @exceptsafe yes
     */
    int demo_test()
    {
        return 0;
    }
};
