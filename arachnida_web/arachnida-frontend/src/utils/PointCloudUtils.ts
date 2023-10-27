import Buffer from 'buffer';
import { Point, PointCloud } from '../components/render/PointcloudViewer';

const laserChannelAngles: number[] = [ -0.261799,  0.0174533, -0.226893, 0.0523599, -0.191986, 0.0872665, -0.15708,  0.122173, 
                                    -0.122173,  0.15708,   -0.0872665, 0.191986, -0.0523599, 0.226893, -0.0174533, 0.261799,
                                    -0.261799,  0.0174533, -0.226893, 0.0523599, -0.191986, 0.0872665, -0.15708,  0.122173, 
                                    -0.122173,  0.15708,   -0.0872665, 0.191986, -0.0523599, 0.226893, -0.0174533, 0.261799 ];

interface PointXYZ {
    x: number | null,
    y: number | null,
    z: number | null
};

interface PointCloudXYZ {
    points: PointXYZ[]
};

interface VLP16Point {
    channel: number | null,
    distance: number | null
}

interface VLP16DataBlock {
    points: VLP16Point[],
    azimuth: number | null
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

export function parsePacketToDataBlocks(packetBuf: Buffer): VLP16Packet | null {
    let packet: VLP16Packet = { 
        dataBlocks: [] 
    };
    
    let ffFlag: boolean = false;
    let firstPacketInBlockTimestamp: number = 0;

    if(packetBuf.length !== 1206) return null;
    if(packetBuf.length / 1206 > 1) return null;

    for(let i = 0; i < packetBuf.length; i++) {
        const byte: number = packetBuf[i]; // byte at index i
        if(byte == 0xFF) {
            ffFlag = true // Found the start of the packet
        } else if(byte == 0xEE) {
            ffFlag = false;
            for(let dataBlock = 0; dataBlock < 12; dataBlock++) {
                if(dataBlock > 0) i += 4;
                let curBlock: VLP16DataBlock = {
                    points: [],
                    azimuth: null,
                };
                curBlock.azimuth = (packetBuf[i+2] << 8 | packetBuf[i+1]) / 100;
                
                for(let channel = 0; channel < 32; channel++) {
                    let curPoint: VLP16Point = {
                        channel: null,
                        distance: null
                    };
                    curPoint.channel = channel;
                    curPoint.distance = (packetBuf[i+1] << 8 | packetBuf[i]) / 500;
                    curBlock.points.push(curPoint);
                };
                packet.dataBlocks.push(curBlock);
            }
            i += 4;
        };
    };

    return packet;
};

export function parsePacketToPointCloud(packetBuf: Buffer) : PointCloud | null {
    let pc: PointCloud = {
        frame: -1,
        points: []
    };

    let pkt: VLP16Packet | null = parsePacketToDataBlocks(packetBuf);

    if(!pkt) return null;

    for(let i = 0; i < pkt.dataBlocks.length; i++) {
        let curDB: VLP16DataBlock = pkt.dataBlocks[i];
        for(let j = 0; j < curDB.points.length; j++) {
            let curPoint: VLP16Point = curDB.points[j];
            let newPoint: Point = {
                x: 200,
                y: 200,
                z: 200
            };
            if(!curPoint.distance || !curDB.azimuth || !curPoint.channel || !getLaserAngleFromChannelID(curPoint.channel)) continue;

            newPoint.x = curPoint.distance * Math.cos(getLaserAngleFromChannelID(curPoint.channel)) * Math.sin(curDB.azimuth * Math.PI / 180);
            newPoint.y = curPoint.distance * Math.cos(getLaserAngleFromChannelID(curPoint.channel)) * Math.cos(curDB.azimuth * Math.PI / 180);
            newPoint.z = curPoint.distance * Math.sin(getLaserAngleFromChannelID(curPoint.channel));
            pc.points.push(newPoint);
        };
    };
    return pc;
};

export function parseFrameToPointCloud(frameBuf: Buffer) : PointCloud | null {
    let pc: PointCloud = {
        frame: -1,
        points: []
    };

    let curPacket: Buffer = Buffer.Buffer.alloc(1206);

    let packetIndexTracker: number = 0;
    let curPacketByteIndex: number = 0;
    for(let i = 0; i < frameBuf.length; i++) {
        if(i == frameBuf.length - 1) {
            curPacket[curPacketByteIndex] = frameBuf[i];
            let curPktPC: PointCloud | null = parsePacketToPointCloud(curPacket);
            if(curPktPC) pc.points.concat(curPktPC.points); // add new points to frame's point cloud (pc)
            break; // break because we're at the end of the frameBuf and therefore there's no more data to parse
        }
        if(packetIndexTracker < 1206) {
            packetIndexTracker++;
            curPacket[packetIndexTracker] = frameBuf[i];
        } else {
            packetIndexTracker = 0;
            let curPktPC: PointCloud | null = parsePacketToPointCloud(curPacket);
            if(curPktPC) pc.points.concat(curPktPC.points); // add new points to frame's point cloud (pc)
        }
    }

    return pc;
};
