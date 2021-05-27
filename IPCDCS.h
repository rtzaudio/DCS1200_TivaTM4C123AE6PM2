/*
  * DCS-1200 Digital Channel Switcher Command Messages
 *
 * Copyright (C) 2021, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * InterProcess Communications (IPC) Services via serial link.
 *
 */

#ifndef _IPCDCS_H_
#define _IPCDCS_H_

/*** DCS IPC Message Header ************************************************/

typedef struct _DCS_IPCMSG_HDR {
    uint16_t    opcode;                     /* the IPC message type code   */
    uint16_t    msglen;                     /* length of hdr & msg data    */
} DCS_IPCMSG_HDR;

/*** Message OpCode Types **************************************************/

/* Command Codes for DCS_IPCMSG_HDR.opcode */
#define DCS_OP_SET_TRACKS       100         /* set all 24-track states     */
#define DCS_OP_GET_TRACKS       101         /* get all 24-track states     */
#define DCS_OP_SET_TRACK        102         /* set single track state      */
#define DCS_OP_GET_TRACK        103         /* get single track state      */
#define DCS_OP_SET_SPEED        104         /* set tape speed hi/lo        */
#define DCS_OP_GET_NUMTRACKS    105         /* get num tracks supported    */

/*** TRACK STATE MODE AND FLAG BITS ****************************************/

#define DCS_NUM_TRACKS          24          /* max tracks in the machine   */

#define DCS_TRACK_REPRO         0x00        /* track is in repro mode      */
#define DCS_TRACK_SYNC          0x01        /* track is in sync mode       */
#define DCS_TRACK_INPUT         0x02        /* track is in input mode      */

#define DCS_TRACK_MASK          0x03        /* low 2-bits are track mode   */

#define DCS_TRACK_MODE(mask)    (mask & DCS_TRACK_MASK)

/* Upper bits indicate ready/record state */
#define DCS_T_RECORD            0x40        /* track is recording now      */
#define DCS_T_READY             0x80        /* track is armed for record   */

/*** SET ALL 24-TRACK STATES ***********************************************/

typedef struct _DCS_IPCMSG_SET_TRACKS {
    DCS_IPCMSG_HDR  hdr;
    uint8_t         trackState[DCS_NUM_TRACKS];
} DCS_IPCMSG_SET_TRACKS;

/*** GET ALL 24-TRACK STATES ***********************************************/

typedef struct _DCS_IPCMSG_GET_TRACKS {
    DCS_IPCMSG_HDR  hdr;
    uint8_t         trackState[DCS_NUM_TRACKS];
} DCS_IPCMSG_GET_TRACKS;

/*** SET SINGLE TRACK STATE ************************************************/

typedef struct _DCS_IPCMSG_SET_TRACK {
    DCS_IPCMSG_HDR  hdr;
    uint8_t         trackNum;
    uint8_t         trackState;
} DCS_IPCMSG_SET_TRACK;

/*** GET SINGLE TRACK STATE ************************************************/

typedef struct _DCS_IPCMSG_GET_TRACK {
    DCS_IPCMSG_HDR  hdr;
    uint8_t         trackNum;
    uint8_t         trackState;
} DCS_IPCMSG_GET_TRACK;

/*** SET TAPE SPEED ********************************************************/

typedef struct _DCS_IPCMSG_SET_SPEED {
    DCS_IPCMSG_HDR  hdr;
    uint8_t         tapeSpeed;          /* 0=low, 1=high speed  */
    uint8_t         rsvd;               /* reserved set to zero */
} DCS_IPCMSG_SET_SPEED;

/*** SET RECORD STATE ******************************************************/

typedef struct _DCS_IPCMSG_GET_NUMTRACKS {
    DCS_IPCMSG_HDR  hdr;
    uint16_t        numTracks;          /* 8, 16 or 24 */
} DCS_IPCMSG_GET_NUMTRACKS;

#endif /* _IPCDCS_H_ */
