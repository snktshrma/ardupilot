#include "AP_DroneCAN_File_Client.h"
#include <AP_Filesystem/AP_Filesystem.h>

AP_DroneCAN_File_Client::AP_DroneCAN_File_Client(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface) :
    _ap_dronecan(ap_dronecan),
    _canard_iface(canard_iface)
{}

bool AP_DroneCAN_File_Client::init(uint8_t server_node_id)
{
    _server_node_id = server_node_id;
    return true;
}

void AP_DroneCAN_File_Client::handle_write_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteResponse &msg){
    if (file_request_cb) {
        (*file_request_cb)(&_ap_dronecan, msg.error.value);
    }
}

int AP_DroneCAN_File_Client::open(const char *path, uint16_t mode)
{
    if (_server_node_id == 0) {
        errno = ENODEV;
        return -1;
    }
    WITH_SEMAPHORE(_sem);

    if (is_opened) {
        errno = EBUSY;
        return -1;
    }

    // we only support readonly and writeonly modes
    if (mode == O_RDONLY) {
        is_opened = true;
    } else if (mode == O_WRONLY) {
        is_opened = false;
    } else {
        errno = EINVAL;
        return -1;
    }
    _total_transaction = 0;
    // make sure we don't have a hangover from last close call
    file_request_cb = nullptr;

    is_incomplete = false;

    return 0;
}

bool AP_DroneCAN_File_Client::write_async(uint8_t data[], uint16_t data_len, FileRequestCb *cb) {
    if (_server_node_id == 0) {
        return false;
    }
    WITH_SEMAPHORE(_sem);

    uavcan_protocol_file_WriteRequest req;

    req.offset = _total_transaction;
    req.data.len = data_len;
    memcpy(req.data.data, data, sizeof(req.data.data));
    file_request_cb = cb;
    _write_client.request(_server_node_id, req);

    // increment total transaction
    _total_transaction += data_len;
    return true;
}

void AP_DroneCAN_File_Client::close_async(FileRequestCb *cb) {
    if (_server_node_id == 0) {
        return;
    }
    WITH_SEMAPHORE(_sem);

    if (!is_opened) {
        // mark the end of the file by sending an empty write request with the offset set to the total transaction
        uavcan_protocol_file_WriteRequest req;
        req.offset = _total_transaction;
        req.data.len = 0;
        file_request_cb = cb;
        _write_client.request(_server_node_id, req);
        _total_transaction = 0;
    }
}





void AP_DroneCAN_File_Client::handle_read_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_ReadResponse &msg){
    if (file_request_cb) {
        (*file_request_cb)(&_ap_dronecan, msg.error.value);
    }
    is_incomplete = msg.data.len < sizeof(uavcan_protocol_file_ReadResponse::data.data);
    
    if (!is_incomplete){
        _total_transaction += msg.data.len;
    }
}

bool AP_DroneCAN_File_Client::read_async(uavcan_protocol_file_Path path, uint16_t len, FileRequestCb *cb) {
    if (_server_node_id == 0) {
        return false;
    }
    WITH_SEMAPHORE(_sem);

    uavcan_protocol_file_ReadRequest req;

    if (!is_incomplete) {
        req.offset = _total_transaction;
        req.path = path;
        file_request_cb = cb;
        _read_client.request(_server_node_id, req);

        // increment total transaction
        return true;
    }

    return false;
    // uavcan_protocol_file_ReadRequest req;

    // req.offset = _total_transaction;
    // req.path

}

// void AP_DroneCAN_File_Client::stop_async(FileRequestCb *cb) {
//     if (_server_node_id == 0) {
//         return;
//     }
//     WITH_SEMAPHORE(_sem);
// }