import express from "express";
import bodyParser from "body-parser";
import cors from "cors";
import WebSocket, { WebSocketServer } from "ws";
import queryString from "query-string";
import dotenv from "dotenv";
import { Duplex } from "stream";
import http from "http";

var app: express.Express = express();
const port: number = 5000;

app.use((req: express.Request, res: express.Response, next: express.NextFunction) => {
	const contentType = req.headers['content-type'];

	if(contentType && contentType === "application/x-www-form-urlencoded") {
		return bodyParser.urlencoded({ extended: true })(req, res, next);
	}

	return bodyParser.json()(req, res, next);
});

// CORS (Numeric IPs are for receiving Paddle webhooks) (GET RID OF LOCALHOST FOR PRODUCTION)
var whitelist: string[] = [ 
                'http://localhost:5000/', 
                'http://localhost:3000', 
                ];
                
var corsOptions = {
    credentials: true,
    origin: (origin: string | undefined, callback: Function) => {
        if(!origin || whitelist.indexOf(origin) !== -1) { // remove !origin for production
            callback(null, true);
        } else {
            console.log(`${origin} not allowed by CORS`);
        }
    }
}
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

	if(!allowedMethods.includes(req.method)) res.status(405).send("Method not allowed");

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

function forwardMessageToAllClients(msg: Buffer, wss: WebSocketServer) {
	wss.clients.forEach((client) => {
		if(client.readyState === WebSocket.OPEN) {
			client.send(msg);
		}
	});
};

function generateRandomPoint(): number[] {
	return [Math.random() * 10, Math.random() * 10, Math.random() * 10];
};

function generateRandomPointCloud(size: number): number[][] | void {
	if(!size) return;
	let pc: number[][] = [];
	for(let i = 0; i < size; i++) {
		pc.push(generateRandomPoint());
	}
	return pc;
};

function convertFrameDataToJSONString(frameID: number, pc: number[][]): string {
	let msgObj = { frame_id: frameID, points: pc, msg: null };
	return JSON.stringify(msgObj);
};

function convertJSONStringToBuffer(msg: string): Buffer {
	return Buffer.from(msg);
};

function forwardRandomPointCloudToAllClients(wss: WebSocketServer): void {
	const generatedPointCloud = generateRandomPointCloud(29500);
	if(!generatedPointCloud) return;
	const msg = convertFrameDataToJSONString(1, generatedPointCloud);
	const msgBuffer = convertJSONStringToBuffer(msg);
	forwardMessageToAllClients(msgBuffer, wss);
};


async function createWSServer(expressServer: http.Server) {
	const wss = new WebSocketServer({
		noServer: true,
		path: "/ws",
	});

	expressServer.on("upgrade", (request: express.Request, socket: Duplex, head: Buffer) => {
		wss.handleUpgrade(request, socket, head, (ws) => {
			wss.emit("connection", ws, request);
		});
	});

	wss.on("connection", (wsConnection: WebSocket, connectionRequest) => {
		wsConnection.on("message", (msg: Buffer) => {
			const msgStr: string = msg.toString();
			console.log(msgStr);
			const parsedMessage = JSON.parse(msgStr);
			if(parsedMessage.frame_id !== -1) { // if frame_id is not null, then the message is from the robot
				console.log(parsedMessage.points);
				console.log(parsedMessage.obstacles);
				forwardMessageToAllClients(msg, wss);
			} else {
				console.log(parsedMessage.msg);
			}
			wsConnection.send(JSON.stringify({ message: "Hello from server" }));
			let lastPointCloudGeneratedTime: number = Date.now();
			
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
};

const server = app.listen(port, () => {
	console.log(`Server running at port ${port}`);
});

const wssp: Promise<WebSocketServer> = createWSServer(server);
