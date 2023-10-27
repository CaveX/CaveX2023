'use client';

import { useEffect, useState } from 'react';
import { Box } from '@mui/material';
import PointcloudViewer, { Frame, PointCloud, Point, Object, ObjectList, convertRawPointDataToPointCloud, convertRawObstacleDataToObjectList } from '../components/render/PointcloudViewer';
import { parseFrameToPointCloud, parsePacketToPointCloud } from '../utils/PointCloudUtils';
import Buffer from 'buffer';

export default function Home() {
	const [pointcloudData, setPointcloudData] = useState<PointCloud | null>(null);
	const [frameData, setFrameData] = useState<Frame | null>(null);
	const [objectList, setObjectList] = useState<ObjectList | null>(null);
    
	const [ws, setWS] = useState<WebSocket | null>(null);
	const [wsConnected, setWSConnected] = useState<boolean>(false);
    const [frameBuf, setFrameBuf] = useState<Buffer>(Buffer.Buffer.from(""));

	useEffect(() => {
		if(!ws) setWS(new WebSocket('wss://api.arachnida.live/ws'));
        console.log("hello");
        const buf: Buffer = Buffer.Buffer.from("hello");
        let newPC: PointCloud | null = parseFrameToPointCloud(buf);
	}, []);

	useEffect(() => {
		if(!ws) return;
		ws.onerror = err => console.error(err);
		ws.onopen = () => {
			console.log('WebSocket connected!');
			setWSConnected(true);
		}
		ws.onclose = () => console.log('WebSocket disconnected!');
		ws.onmessage = async (msg: MessageEvent) => {
			if(typeof msg.data == 'string') { // If the type of msg.data is a string then we assume that it's stringified JSON
				// const text = await new Response(blob).text();
                const text = msg.data;
				// console.log(text);
				const json = JSON.parse(text);
				// console.log(json);
				if(typeof json.frame_id !== 'undefined' && json.points) {
					const pc: PointCloud | null = convertRawPointDataToPointCloud(json.points);
					let obstacles: ObjectList | null = null;
					if(json.obstacles) {
						let obs: ObjectList | null = convertRawObstacleDataToObjectList(json.obstacles);
						if(!obs) obs = { objects: [] };
						obstacles = obs;
					}
					const frame: Frame = { frame: json.frame_id, pc: pc, obstacles: obstacles };
					setFrameData(frame);
					setPointcloudData(pc);

                    if(json.floam_transform) {
                        // Store transforms somewhere to display path
                    }
				}

			} else if(msg.data instanceof Blob) { // Instance of blob means it's binary data, which in our case indicates its a frame (point cloud + obstacle data)
				const blob: Blob = msg.data;
                let pktBuf: Buffer;
                new Response(blob).arrayBuffer().then((buf) => {
                    pktBuf = Buffer.Buffer.from(buf);
                    if(frameBuf.length >= 94068) { 
                        let newPC: PointCloud | null = parseFrameToPointCloud(frameBuf);
                        if(newPC) setPointcloudData(newPC);
                    } else {
                        let newBuf: Buffer = frameBuf;
                        setFrameBuf(Buffer.Buffer.concat([frameBuf, pktBuf]));
                    }

                });
			}
		}

	}, [ws]);

	useEffect(() => {
		if(wsConnected && ws) {
			if(ws.readyState == ws?.OPEN) {
				ws.send(JSON.stringify({
					frame_id: -1,
					points: [],
					msg: `
                        Dillon! You son of a bitch! 

                                                                                                      
                                         #*********                                                   
                                      #********+***#                                                  
                                     ******+***#####**                                                
                                    **********++***####                                               
                                    *******+*********%                                                
                                   +#******+++*+**##**%                                               
                                   *********##*****###                                                
                                  *********++++**%###%                                                
                                  ********##*****#%###                                                
                                 +********#%%%####**##                                            #*#%
                                 +******###%@@%%%%#*%%%                                        ##%%%%%
                                 +*+*#####%@%%###%%####                                   +*#%%%%%%%%%
====                             =****#%%%%%%########%%%                               *###%#%###%%%%%
+*++=++                          =+****###%%%%#######%%%                            **#*#%##*%####%%%%
++++**+=+                         ++******%%%%#######%%%                          **#**##%##*%#####%%%
====--=++*                          ******#%%%%%####%%%%%                       *+#%##**##%########%%%
+=====-----=                        *******##%%#####%%%%%                    ++=*#**#%%#####*#%####%%%
+========*#*==                       *******%%%####%%%%%%                   *#**#%######%#%%##%%##%%%%
++====--==++**+                       *****#%%%###%%%%%%                  **%####%%%####%%%%%#%%##%%%%
+===========++*                        ****%%#####%%%%%%              =#*******##**#%%%%##%%%%%%##%%%%
+=====++****+=#                        =**#%#######%%%##            *************#####%#%###%%%%##%%%%
+++=++=+==+=-*++                        =%%###**##%%%%#*#         +*##**++++*******####%%%###%%%#%%%%%
++++++==+==-=+==#                      #%%%##***#%%%%#*****      =###*+++++********##%##%%###%%%#%%%%%
+++++=++=======*=+-                   #%%%##***#%%%%%#******#    ####**+++++******###%%##%%%#%%%#%%%%%
+++++*+++====+*==-=+                 =*####****#%%%%%*+*******  +####*********######%%%##%%%%%%%%%%%%%
++++#+++===+*+==-+=---=            =#%######*##%%%%%%**+******###***##*****#######%%%%%%%%%%%%%%%%%%%%
++*#+====+#+===-+=----=#%        +*%##########%%%%%%##*********####***##**######%%%%%##%%%%%%%%%%%%%%%
+*#+===+*+-=+===-----=+*%%#     #%####*######%%%%%%%##******************############***#%%%%@%%%%%%%%%
*#*===+==+#+=-------=+=*%%%%   #%########**#%%%%%%%%#***************######*****####**+*#%%@%@@%%%%%%%%
#+=====+##=--------=+=+#%%%%@ %%###**##***#%%%%%%@%%##************######*******###****#%%%@@@@@%%%%%%%
+====+#%*====-----====*%%##%%@%###********#%%%%%@%%%##************#*##******#####****#%%%%@  @@%%@%%%%
===+#%%#+++========+=#%#####%%###**##*#####%%%%%@%%%###**********#**####************#%%%%     @%@@%%%@
=+*#%%#**++++++++=++%%%%#####%######**####%%%%%%%%%%####*********#**##*******##***##%%%%       @@@@@@@
#%%%####**++++=+++#%%%%%%#################%%%%%@%%%%%####********#****+++***######%%%%           @@%@@
%%%%####*****+++*%%%%%%%%%###############%%%%%@@ %%%%%%######**###***+++*####%##%%%%             %@@@@
%%%%%####*****%%%%%%%%%%%%%##%#%%######%%%%%%@@   %%%%%%%%########*****#####%%%%%                 %@@@
  %%%%%#%###%@@%%%%%%%%%%%%%%%%%######%%%%%%@@     %%%%%%%%#######***####%%%%%%                   @@@@
     %%%%%@@@@@%%%%%%%%%%%%%%%%####%%%%%%%%@@@      %%%%%%%%%%#######%%%%%%%%                     @@@@
       %@@@@@@@@@@%%%%%%%%%%%%%##%%%%%%%%@@@@        %%%%%%%%%%%%%%%%%%%%%                         @@@
          @@@@@@@@@@%%%%@%%%%%%%%%%%%%%@%@@%            %%%%%%%%%%%%%%%                             @@
             @@@@@@@@@@%%%%%%%%%%%%@@@@@@@                  %%%%%%%%                                %%
                @@@@@@@%%%%%%%%%@@@@@@@@                                                              `
				}));
			}
		}
	}, [wsConnected]);

	return (
		<Box
			sx={{
				width: '100vw',
				height: '100vh',
			}}
		>
			<PointcloudViewer frameData={frameData} objectList={objectList} />
		</Box>
	);
}
