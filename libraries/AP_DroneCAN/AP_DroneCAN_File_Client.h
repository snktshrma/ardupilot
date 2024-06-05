#pragma once
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>


#include <AP_Common/Bitmask.h>
#include <StorageManager/StorageManager.h>
#include <AP_CANManager/AP_CANManager.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include "AP_Canard_iface.h"
#include <dronecan_msgs.h>

class AP_DroneCAN;
//Forward declaring classes
class AP_DroneCAN_File_Client
{

    HAL_Semaphore storage_sem;
    AP_DroneCAN &_ap_dronecan;
    CanardInterface &_canard_iface;

    Canard::ObjCallback<AP_DroneCAN_File_Client, uavcan_protocol_file_WriteResponse> file_wr_cb{this, &AP_DroneCAN_File_Client::handle_get_info_response};
    Canard::Client<uavcan_protocol_file_WriteResponse> _write_client;

public:
    AP_DroneCAN_File_Client(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface, uint8_t driver_index);


    // Do not allow copies
    CLASS_NO_COPY(AP_DroneCAN_File_Client);

    //Initialises publisher and Server Record for specified uavcan driver
    bool init(uint8_t own_unique_id[], uint8_t own_unique_id_len, uint8_t node_id);

    //Reset the Server Record
    void reset();

    /* Checks if the node id has been verified against the record
    Specific CAN drivers are expected to check use this method to 
    verify if the node is healthy and has static node_id against 
    hwid in the records */
    bool isNodeIDVerified(uint8_t node_id) const;

    /* Subscribe to the messages to be handled for maintaining and allocating
    Node ID list */
    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);

    //report the server state, along with failure message if any
    bool prearm_check(char* fail_msg, uint8_t fail_msg_len) const;

    //Callbacks
    void handle_get_info_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteResponse& req);

    //Run through the list of seen node ids for verification
    void verify_nodes();
};