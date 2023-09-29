import express from "express";
import bodyParser from "body-parser";
import cors from "cors";
import WebSocket, { WebSocketServer } from "ws";
import queryString from "query-string";
import dotenv from "dotenv";
import { Duplex } from "stream";
import http from "http";

var app: express.Express = express();
const port: number = 4000;

interface Point {
	x: number;
	y: number;
	z: number;
};

interface PointCloud {
	frame: number;
	points: Point[];
};

interface Object {
	id: number;
	centroid: Point;
	vertices: Point[];
};

interface ObjectList {
	objects: Object[];
};

interface Frame {
	frame_id: number;
	pc: PointCloud;
	objects: ObjectList;
};

let frameQueue: Frame[] = []; // Stores the frames to be sent to the webserver as FIFO (i.e the first frame in the queue is the next one to be sent - new frames are pushed to the back)

function convertRawPointToPoint(rawPoint: number[]): Point | null {
	if(!rawPoint) return null;
	let p: Point = { x: rawPoint[0] ?? 0, y: rawPoint[1] ?? 0, z: rawPoint[2] ?? 0 };
	return p;
};

function convertRawPointDataToPointCloud(frameID: number, rawPointData: number[][]): PointCloud | null {
	if(!rawPointData) return null;
	let pc: PointCloud = { frame: frameID, points: [] };
	for(let i = 0; i < rawPointData.length; i++) {
		if(!rawPointData[i]) continue;
		let p = convertRawPointToPoint(rawPointData[i]);
		if(!p) continue;
		pc.points.push(p);
	}

	return pc;
};

app.use((req: express.Request, res: express.Response, next: express.NextFunction) => {
	const contentType = req.headers['content-type'];
	if(contentType && contentType === "application/x-www-form-urlencoded") {
		return bodyParser.urlencoded({ extended: true })(req, res, next);
	}

	return bodyParser.json()(req, res, next);
});

var whitelist: string[] = [
	'http://localhost:5000',
	'http://localhost:3000',
];

var corsOptions = {
	credentials: true,
	origin: (origin: string | undefined, callback: Function) => {
		if(!origin || whitelist.indexOf(origin) !== -1) {
			callback(null, true);
		} else {
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

	if(!allowedMethods.includes(req.method)) res.status(405).send("Method not allowed");

	next();
});

async function createWSServer(expressServer: http.Server): Promise<WebSocketServer> {
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
			while(true) { // infinite loop which checks if there's data available to send and sends it if there is

			}
		});
	});

	return wss;
};

const server = app.listen(port, () => {
	console.log(`[websockt_interface] Server running at port ${port}`);
});

const wssp: Promise<WebSocketServer> = createWSServer(server);
