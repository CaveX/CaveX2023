import Buffer from 'buffer';

const laserChannelAngles: number[] = [ -0.261799,  0.0174533, -0.226893, 0.0523599, -0.191986, 0.0872665, -0.15708,  0.122173, 
                                    -0.122173,  0.15708,   -0.0872665, 0.191986, -0.0523599, 0.226893, -0.0174533, 0.261799,
                                    -0.261799,  0.0174533, -0.226893, 0.0523599, -0.191986, 0.0872665, -0.15708,  0.122173, 
                                    -0.122173,  0.15708,   -0.0872665, 0.191986, -0.0523599, 0.226893, -0.0174533, 0.261799 ];

interface PointXYZ {
    x: number,
    y: number,
    z: number
};

interface VLP16DataBlock {
    points: PointXYZ[],
    azimuth: number
};

interface VLP16Packet {
    dataBlocks: VLP16DataBlock[]
};

interface VLP16Frame {
    packets: VLP16Packet[]
};

// Returns the vertical angle of a laser relative to the VLP16's horizon in radians from a laser ID/channel ID
function getLaserAngleFromChannelID(channelID: number): number {
    if(channelID < 1 || channelID > 32) return 0; // return 0 if channel ID is invalid
    else return laserChannelAngles[channelID - 1];
};

export function parsePacketToDataBlocks(packetBuf: Buffer): VLP16Packet {
    let packet: VLP16Packet = { 
        dataBlocks: [] 
    };
    for(let i = 0; i < packetBuf.length; i++) {
        const byte: number = packetBuf[i]; // byte at index i
        
    };

    return packet;
};
