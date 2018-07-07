#ifndef RAM_COMMANDS_H
#define RAM_COMMANDS_H

constexpr unsigned char NETWORK_RESET[2]          = { '\x01', '\x31' };
constexpr unsigned char NETWORK_SYSTEM[2]         = { '\x02', '\x32' };

constexpr unsigned char CAMERA_VIDEO_TOGGLE[2]    = { '\x0A', '\x01' };
constexpr unsigned char CAMERA_CAM_1_PIC[2]       = { '\x0B', '\x02' };
constexpr unsigned char CAMERA_CAM_2_PIC[2]       = { '\x0C', '\x03' };
constexpr unsigned char CAMERA_CAM_3_PIC[2]       = { '\x0D', '\x04' };
constexpr unsigned char CAMERA_CAM_4_PIC[2]       = { '\x0E', '\x05' };
constexpr unsigned char CAMERA_CAM_ALL_PIC[2]     = { '\x0F', '\x06' };
constexpr unsigned char CAMERA_RESET[2]           = { '\x13', '\x31' };

constexpr unsigned char AMBIENT_WRITE_RATE[2]     = { '\x14', '\x00' };
constexpr unsigned char AMBIENT_RESET[2]          = { '\x18', '\x31' };

constexpr unsigned char BBOX_WRITE_RATE[2]        = { '\x19', '\x00' };
constexpr unsigned char BBOX_RESET[2]             = { '\x1D', '\x31' };

constexpr unsigned char ARM_MAN_X[2]              = { '\x1E', '\x00' };
constexpr unsigned char ARM_MAN_Y[2]              = { '\x1F', '\x00' };
constexpr unsigned char ARM_MAN_Z[2]              = { '\x20', '\x00' };
constexpr unsigned char ARM_MAN_E[2]              = { '\x21', '\x00' };
constexpr unsigned char ARM_EXEC_MAN[2]           = { '\x22', '\x01' };
constexpr unsigned char ARM_MODE[2]               = { '\x23', '\x00' };
constexpr unsigned char ARM_ADD_TRIAL[2]          = { '\x24', '\x00' };
constexpr unsigned char ARM_RESET_TRIAL[2]        = { '\x26', '\x00' };
constexpr unsigned char ARM_ROT_INC[2]            = { '\x27', '\x00' };
constexpr unsigned char ARM_ROT_DEC[2]            = { '\x28', '\x00' };
constexpr unsigned char ARM_SHO_INC[2]            = { '\x29', '\x00' };
constexpr unsigned char ARM_SHO_DEC[2]            = { '\x2A', '\x00' };
constexpr unsigned char ARM_ELB_INC[2]            = { '\x2B', '\x00' };
constexpr unsigned char ARM_ELB_DEC[2]            = { '\x2C', '\x00' };
constexpr unsigned char ARM_WRI_INC[2]            = { '\x2D', '\x00' };
constexpr unsigned char ARM_WRI_DEC[2]            = { '\x2E', '\x00' };
constexpr unsigned char ARM_WRR_INC[2]            = { '\x2F', '\x00' };
constexpr unsigned char ARM_WRR_DEC[2]            = { '\x30', '\x00' };
constexpr unsigned char ARM_GRP_INC[2]            = { '\x31', '\x00' };
constexpr unsigned char ARM_GRP_DEC[2]            = { '\x32', '\x00' };
constexpr unsigned char ARM_RESET[2]              = { '\xFF', '\x31' };




#endif
