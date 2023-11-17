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
var app = express();
const port = 5000;
app.use((req, res, next) => {
    const contentType = req.headers['content-type'];
    if (contentType && contentType === "application/x-www-form-urlencoded") {
        return bodyParser.urlencoded({ extended: true })(req, res, next);
    }
    return bodyParser.json()(req, res, next);
});
// CORS (Numeric IPs are for receiving Paddle webhooks) (GET RID OF LOCALHOST FOR PRODUCTION)
var whitelist = [
    'http://localhost:5000/',
    'http://localhost:3000',
];
var corsOptions = {
    credentials: true,
    origin: (origin, callback) => {
        if (!origin || whitelist.indexOf(origin) !== -1) { // remove !origin for production
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
// interface Message {
// 	frame_id: string | null,
// 	points: number[][] | null,
// 	msg: string | null,
// }
//
// function parseMessage(msg: string): Message {
// 	const parsed: JSON = JSON.parse(msg);
// 	console.log(parsed);
// 	var msgObj: Message = { frame_id: null, points: null, msg: null };
// 	if(parsed.frame_id) msgObj.frame_id = parsed.frame_id;
// 	if(parsed.points) msgObj.points = parsed.points;
// 	if(parsed.msg) msgObj.msg = parsed.msg;
// 	return msgObj;
// }
function forwardMessageToAllClients(msg, wss) {
    wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(msg);
        }
    });
}
;
function generateRandomPoint() {
    return [Math.random() * 10, Math.random() * 10, Math.random() * 10];
}
;
function generateRandomPointCloud(size) {
    if (!size)
        return;
    let pc = [];
    for (let i = 0; i < size; i++) {
        pc.push(generateRandomPoint());
    }
    return pc;
}
;
function convertFrameDataToJSONString(frameID, pc) {
    let msgObj = { frame_id: frameID, points: pc, msg: null };
    return JSON.stringify(msgObj);
}
;
function convertJSONStringToBuffer(msg) {
    return Buffer.from(msg);
}
;
function forwardRandomPointCloudToAllClients(wss) {
    const generatedPointCloud = generateRandomPointCloud(29500);
    if (!generatedPointCloud)
        return;
    const msg = convertFrameDataToJSONString(1, generatedPointCloud);
    const msgBuffer = convertJSONStringToBuffer(msg);
    forwardMessageToAllClients(msgBuffer, wss);
}
;
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
                try {
                    const parsedMessage = JSON.parse(msgStr);
                    if (parsedMessage.frame_id !== -1) { // if frame_id is not null, then the message is from the robot
                        console.log(parsedMessage.points);
                        console.log(parsedMessage.obstacles);
                        forwardMessageToAllClients(msg, wss);
                    }
                    else {
                        console.log(parsedMessage.msg);
                    }
                }
                catch (e) { // SyntaxError due to message not being JSON, therefore assume it is binary (raw point cloud data)
                    forwardMessageToAllClients(msg, wss);
                }
                wsConnection.send(JSON.stringify({ message: "Hello from server" }));
                let lastPointCloudGeneratedTime = Date.now();
                // TESTING: Send random point cloud every 500 ms
                // while(true) {
                // 	if(Date.now() - lastPointCloudGeneratedTime < 500) {
                // 		continue;
                // 	};
                // 	lastPointCloudGeneratedTime = Date.now();
                // 	forwardRandomPointCloudToAllClients(wss);
                // 	console.log("Point Cloud generation and sending took %d ms", Date.now() - lastPointCloudGeneratedTime);
                // }
                // END TESTING
            });
        });
        return wss;
    });
}
;
const server = app.listen(port, () => {
    console.log(`Server running at port ${port}`);
});
const wssp = createWSServer(server);
