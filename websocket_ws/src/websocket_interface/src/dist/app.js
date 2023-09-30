var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
import express from "express";
import bodyParser from "body-parser";
import cors from "cors";
import WebSocket, { WebSocketServer } from "ws";
import rosnodejs from "rosnodejs";
var app = express();
const port = 4000;
app.use((req, res, next) => {
    const contentType = req.headers['content-type'];
    if (contentType && contentType === "application/x-www-form-urlencoded") {
        return bodyParser.urlencoded({ extended: true })(req, res, next);
    }
    return bodyParser.json()(req, res, next);
});
var whitelist = [
    'http://localhost:5000',
    'http://localhost:3000',
];
var corsOptions = {
    credentials: true,
    origin: (origin, callback) => {
        if (!origin || whitelist.indexOf(origin) !== -1) {
            callback(null, true);
        }
        else {
            console.log(`${origin} not allowed by CORS`);
        }
    }
};
app.use(cors(corsOptions));
app.use((req, res, next) => {
    const allowedMethods = [
        "GET",
        "POST",
        "PUT",
        "DELETE",
        "OPTIONS",
        "PATCH",
        "HEAD",
        "CONNECT",
    ];
    if (!allowedMethods.includes(req.method))
        res.status(405).send("Method not allowed");
    next();
});
function createWSServer(expressServer) {
    return __awaiter(this, void 0, void 0, function* () {
        const wss = new WebSocketServer({
            noServer: true,
            path: "/ws",
        });
        expressServer.on("upgrade", (request, socket, head) => {
            wss.handleUpgrade(request, socket, head, (ws) => {
                wss.emit("connection", ws, request);
            });
        });
        wss.on("connection", (wsConnection, connectionRequest) => {
            wsConnection.on("message", (msg) => {
                const msgStr = msg.toString();
                console.log(msgStr);
                const parsedMessage = JSON.parse(msgStr);
                while (true) { // infinite loop which checks if there's data available to send and sends it if there is
                }
            });
        });
        return wss;
    });
}
;
// START: ROS Stuff
// const arachnidaMsgs = rosnodejs.require("arachnida");
// const rosNH = rosnodejs.nh; // NodeHandler
// const obstacleSub = rosNH.subscribe("arachnida/obstacle_detection/obstacles", 'arachnida/ObstacleList', (msg) => {
// 	// if frame number of msg is same as any of the messages in the frameQueue then add these obstacles to that frame
// 	// otherwise add a new Frame to the frameQueue
// 	console.log('Received obstacle msg: %j', msg);
// })
// END: ROS Stuff
const server = app.listen(port, () => {
    console.log(`[websockt_interface] Server running at port ${port}`);
    const ws = new WebSocket("wss://api.arachnida.live/ws");
    rosnodejs.initNode("/ws_node").then((n) => {
        // const sensorMsgs = rosnodejs.require("sensor_msgs").msg;
        let pcSub = n.subscribe("arachnida/point_cloud/pcl", 'sensor_msgs/PointCloud2', (msg) => {
            // if frame number of msg is same as any of the messages in the frameQueue then add this point cloud to that frame
            // otherwise add a new Frame to the frameQeue
            console.log('Received point cloud msg: %j', msg);
        });
        // let obstacleSub = n.subscribe("arachnida/obstacle_detection/obstacles", 'arachnida/ObstacleList', (msg) => {
        // if frame number of msg is same as any of the messages in the frameQueue then add these obstacles to that frame
        // otherwise add a new Frame to the frameQueue
        // console.log('Received obstacle msg: %j', msg);
        // });
    });
});
const wssp = createWSServer(server);
