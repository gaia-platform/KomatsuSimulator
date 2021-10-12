#pragma once

#include <string>

/**
 * @brief Interface used to expose danger_zone core methods inside a Gaia ruleset rules
 */

class danger_zone_t
{
protected:
// 1) We ignore -Wc++17-extensions here to quiet down the Gaia translator
// 2) We mention GCC but this works for Clang as well
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc++17-extensions"
    // inline static const std::shared_ptr<ismartpallet> campus_ruleset_p_campus = nullptr;

    // The pointer to the live instance of the campus core class
    // TODO : This is not an example to good coding practice, need to make safe and modern
    inline static danger_zone_t* danger_zone_p = nullptr;
#pragma GCC diagnostic pop

public:
    /**
     * Class factory, this is the only method allowed for obtaining an instance of the
     * class withing a ruleset rule
     *
     * @return danger_zone_t*
     * @throws
     * @exceptsafe yes
     */
    static danger_zone_t* get_callback_class()
    {
        return danger_zone_p;
    }

    /*static std::shared_ptr<ismartpallet> get_callback_classy(){
        return std::make_shared<i_Campus>(campus_ruleset_p_campus);
    }*/

    /**
     * Call this from within a ruleset rule to send a ROS2 obstacleArray message
     *
     * @param[in] std::string : type_name : the name of the class of object detected
     * @param[in] uint : roi : the roi code
     * @param[in] uint : direction : the direction code
     * @param[in] double : posx : the object position X
     * @param[in] double : posy : the object position Y
     * @param[in] double : posz : the object position Z
     * @param[in] double : sizex : the object size X
     * @param[in] double : sizey : the object size Y
     * @param[in] double : sizez : the object size Z
     * @param[in] double : orientx : the object orientation quaternion X
     * @param[in] double : orienty : the object orientation quaternion Y
     * @param[in] double : orientz : the object orientation quaternion Y
     * @param[in] double : orientw : the object orientation quaternion W
     * @param[in] std::string : frame_id : the ROS frame name
     * @param[in] int32_t : sec : the time in seconds
     * @param[in] uint32_t : nsec : the number of nanoseconds since sec
     * @return void
     * @throws
     * @exceptsafe yes
     */
    virtual void cb_send_obstacle_array_message(
        std::string type_name, uint roi, uint direction,
        double posx, double posy, double posz,
        double sizex, double sizey, double sizez,
        double orientx, double orienty, double orientz, double orientw,
        std::string frame_id, int32_t sec, uint32_t nsec)
        = 0;

    /**
     * Constructor
     *
     * @throws
     * @exceptsafe yes
     */
    danger_zone_t() = default;

    /**
     * Destructor, to get rid of annoying build warnings
     *
     * @throws
     * @exceptsafe yes
     */
    virtual ~danger_zone_t() = default;

    /**
     * Unit test, needs to be implemented
     *
     * @throws
     * @exceptsafe yes
     */
    int demo_test()
    {
        return 0;
    }
};
