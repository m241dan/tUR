#ifndef RAM_COMMANDS_H
#define RAM_COMMANDS_H



constexpr uint8_t RESET_BYTE = '\x31';

constexpr uint8_t NETWORK_RESET[2]          = { '\x01', '\x31' };
constexpr uint8_t NETWORK_SYSTEM[2]         = { '\x02', '\x32' };

constexpr uint8_t CAMERA_VIDEO_TOGGLE[2]    = { '\x0A', '\x00' };
constexpr uint8_t CAMERA_VIDEO_OFF[2]       = { '\x0A', '\x01' };
constexpr uint8_t CAMERA_VIDEO_ON[2]        = { '\x0A', '\x05' };
constexpr uint8_t CAMERA_CAM_1_PIC[2]       = { '\x0B', '\x02' };
constexpr uint8_t CAMERA_CAM_2_PIC[2]       = { '\x0C', '\x03' };
constexpr uint8_t CAMERA_CAM_3_PIC[2]       = { '\x0D', '\x04' };
constexpr uint8_t CAMERA_CAM_4_PIC[2]       = { '\x0E', '\x05' };
constexpr uint8_t CAMERA_CAM_ALL_PIC[2]     = { '\x0F', '\x06' };
constexpr uint8_t CAMERA_RESET[2]           = { '\x13', '\x31' };

constexpr uint8_t AMBIENT_WRITE_RATE[2]     = { '\x14', '\x00' };
constexpr uint8_t AMBIENT_RESET[2]          = { '\x18', '\x31' };

constexpr uint8_t BBOX_WRITE_RATE[2]        = { '\x19', '\x00' };
constexpr uint8_t BBOX_RESET[2]             = { '\x1D', '\x31' };

constexpr uint8_t ARM_MAN_X[2]              = { '\x1E', '\x00' };
constexpr uint8_t ARM_MAN_Y[2]              = { '\x1F', '\x00' };
constexpr uint8_t ARM_MAN_Z[2]              = { '\x20', '\x00' };
constexpr uint8_t ARM_MAN_E[2]              = { '\x21', '\x00' };
constexpr uint8_t ARM_EXEC_MAN[2]           = { '\x22', '\x01' };
constexpr uint8_t ARM_MODE[2]               = { '\x23', '\x00' };
constexpr uint8_t ARM_ADD_TRIAL[2]          = { '\x24', '\x00' };
constexpr uint8_t ARM_RESET_TRIAL[2]        = { '\x26', '\x01' };
constexpr uint8_t ARM_ROT_INC[2]            = { '\x27', '\x00' };
constexpr uint8_t ARM_ROT_DEC[2]            = { '\x28', '\x00' };
constexpr uint8_t ARM_SHO_INC[2]            = { '\x29', '\x00' };
constexpr uint8_t ARM_SHO_DEC[2]            = { '\x2A', '\x00' };
constexpr uint8_t ARM_ELB_INC[2]            = { '\x2B', '\x00' };
constexpr uint8_t ARM_ELB_DEC[2]            = { '\x2C', '\x00' };
constexpr uint8_t ARM_WRI_INC[2]            = { '\x2D', '\x00' };
constexpr uint8_t ARM_WRI_DEC[2]            = { '\x2E', '\x00' };
constexpr uint8_t ARM_WRR_INC[2]            = { '\x2F', '\x00' };
constexpr uint8_t ARM_WRR_DEC[2]            = { '\x30', '\x00' };
constexpr uint8_t ARM_GRP_INC[2]            = { '\x31', '\x00' };
constexpr uint8_t ARM_GRP_DEC[2]            = { '\x32', '\x00' };
constexpr uint8_t ARM_CAM_ONE[2]            = { '\x33', '\x01' };
constexpr uint8_t ARM_CAM_TWO[2]            = { '\x34', '\x02' };
constexpr uint8_t ARM_CAM_THR[2]            = { '\x35', '\x03' };
constexpr uint8_t ARM_E_STOP[2]             = { '\x40', '\x01' };
constexpr uint8_t ARM_RESET[2]              = {static_cast<uint8_t>('\xFF'), static_cast<uint8_t>('\x31') };

#endif
