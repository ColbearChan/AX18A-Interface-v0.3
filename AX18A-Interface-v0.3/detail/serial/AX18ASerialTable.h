//
// Created by matthias on 11/03/18.
//

#ifndef AX18A_SERVERCLIENT_AX18SERIALTABLE_H
#define AX18A_SERVERCLIENT_AX18SERIALTABLE_H

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
//#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
//#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define DXL_NUM                         7                   
static uint16_t DXL_ID[DXL_NUM] = {1,2,3,4,5,6,7};
//#define BAUDRATE                        57600
#define BAUDRATE                        1000000
#define DEVICENAME_BARE                      "/dev/ttyUSB"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define DEVICE_ID_DEFAULT 0


#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      110                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      512                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define DXL_POS_MIN 0
#define DXL_POS_CENTER 512
#define DXL_POS_MAX 1023
#define DXL_DEG_RANGE 300.0
#define DXL_DEG_PER_POS (DXL_DEG_RANGE/DXL_POS_MAX)
#define DXL_RAD_PER_POS ((DXL_DEG_RANGE/360.0)*2*3.14159265359/DXL_POS_MAX)



#endif //AX18A_SERVERCLIENT_AX18SERIALTABLE_H
