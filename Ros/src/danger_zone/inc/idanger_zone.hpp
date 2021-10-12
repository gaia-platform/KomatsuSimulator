#pragma once

#include <string>
#include <vector>

/**
* @brief Interface used to expose danger_zone core methods inside a Gaia ruleset rules
*/

class iDangerZone
{
protected:
// 1) We ignore -Wc++17-extensions here to quiet down the Gaia translator
// 2) We mention GCC but this works for Clang as well
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc++17-extensions"
    //inline static const std::shared_ptr<ismartpallet> campus_ruleset_p_campus = nullptr;

    // The pointer to the live instance of the campus core class
    // TODO : This is not an example to good coding practice, need to make safe and modern
    inline static iDangerZone* danger_zone_p = nullptr;
#pragma GCC diagnostic pop

public:
    /**
     * Class factory, this is the only method allowed for obtaining an instance of the 
     * class withing a ruleset rule
     * 
     * @return iDangerZone*
     * @throws 
     * @exceptsafe yes
     */
    static iDangerZone* get_callback_class()
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
    virtual void cb_send_obstacleArray_message(
        std::string type_name, uint roi, uint direction,
        double posx, double posy, double posz, 
        double sizex, double sizey, double sizez, 
        double orientx, double orienty, double orientz, double orientw,
        std::string frame_id, int32_t sec, uint32_t nsec 
        ) = 0;

    /**
     * Call this from within a ruleset rule to trigger a log event
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
    virtual void cb_trigger_log( int start_sec, uint32_t start_nsec, 
        int end_sec, uint32_t end_nsec, std::string file_name, 
        std::vector<std::string>topics) = 0;

    /**
     * Call this from within a ruleset rule to trigger a log event
     * 
     * @param[in] int seconds_past
     * @param[in] int seconds_forward
     * @param[in] std::string file_name
     * @param[in] std::vector<std::string>topics
     * @return void
     * @throws 
     * @exceptsafe yes
     */    
    virtual void cb_trigger_log( int seconds_past, int seconds_forward, 
        std::string file_name, std::vector<std::string>topics) = 0;

    /**
     * Constructor
     * 
     * @throws 
     * @exceptsafe yes
     */
    iDangerZone() = default;

    /**
     * Destructor, to get rid of annoying build warnings
     * 
     * @throws 
     * @exceptsafe yes
     */
    virtual ~iDangerZone() = default;

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