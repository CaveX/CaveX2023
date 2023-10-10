import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export interface Point {
	x: number;
	y: number;
	z: number;
};

export interface PointCloud {
	frame: number;
	points: Point[];
};

export interface Object {
	id: number;
	centroid: Point;
	vertices: Point[];
};

export interface ObjectList {
	objects: Object[] | null;
};

export interface Frame { 
	frame: number | null;
	pc: PointCloud | null;
	obstacles: ObjectList | null;
};

export function convertRawPointToPoint(rawPoint: number[]): Point | null {
	if(!rawPoint) return null;
	let p: Point = { x: rawPoint[0] ?? 0, y: rawPoint[1] ?? 0, z: rawPoint[2] ?? 0 };
	return p;
};

export function convertRawPointDataToPointCloud(rawPointData: number[][]): PointCloud | null {
	if(!rawPointData) return null;
	let pc: PointCloud = { frame: 1, points: [] }
	for(let i = 0; i < rawPointData.length; i++) {
		pc.points.push(convertRawPointToPoint(rawPointData[i]) ?? { x: 0, y: 0, z: 0 });
	}
	return pc;
};

// Example JSON structure of obstacle data
// export let obstacleJSONExample = {
// 	"frame_id": 1,
// 	"obstacles": [
// 		["id",
// 			["centroid"],
// 			[
// 				["vertex1x,vertex1y,vertex1z"],
// 				["vertex2x,vertex2y,vertex2z"],
// 				["vertex3x,vertex3y,vertex3z"],
// 				["vertex4x,vertex4y,vertex4z"],
// 				["vertex5x,vertex5y,vertex5z"],
// 				["vertex6x,vertex6y,vertex6z"],
// 				["vertex7x,vertex7y,vertex7z"],
// 				["vertex8x,vertex8y,vertex8z"]
// 			]
// 		],
// 		[1,    // obstacles[1][0]
// 			[1, 2, 3],       // obstacles[1][0] -> x = obstacles[1][0][0], y = obstacles[1][0][1], z = obstacles[1][0][2]
// 			[
// 				[1, 2, 3],		// obstacles[1][1][0] -> x = obstacles[1][1][0][0], y = obstacles[1][1][3][0], z = obstacles[1][1][6][0]
// 				[4, 5, 6],		// obstacles[1][1][1] -> x = obstacles[1][1][1][1], y = obstacles[1][1][4][1], z = obstacles[1][1][7][1]
// 				[7, 8, 9],		// obstacles[1][1][2] -> x = obstacles[1][1][2][2], y = obstacles[1][1][5][2], z = obstacles[1][1][8][2]
// 				[10, 11, 12],
// 				[13, 14, 15],
// 				[16, 17, 18],
// 				[19, 20, 21],
// 				[22, 23, 24]
// 			]
// 		]
// 	],
// 	"obstTwo": [    [1, [10,20,30], [ [1,2,3],[4,5,6],[7,8,9] ] ],      [1, [10,20,30], [ [1,2,3],[4,5,6],[7,8,9] ] ]    ],
// 	// obstTwo[0] = [1, [10,20,30], [ [1,2,3],[4,5,6],[7,8,9] ] ]
// 	// obstTwo[0][0] = 1
// 	// obstTwo[0][1] = [10,20,30]
// 	// obstTwo[0][1][0] = 10
// 	// obstTwo[0][2] = [ [1,2,3],[4,5,6],[7,8,9] ]
// 	// obstTwo[0][2][0] = [1,2,3]
// 	// obstTwo[0][2][0][0] = 1 // 0th obstacle -> vertices -> 0th vertex -> x coordinate of 0th vertex
// };

export function convertRawObstacleDataToObjectList(rawObstacleData: any): ObjectList | null {
	if(!rawObstacleData) return null;
	let obs: ObjectList = { objects: [] };
	for(let i = 0; i < rawObstacleData.length; i++) {
		obs.objects?.push({
			id: rawObstacleData[i][0],
			centroid: { x: rawObstacleData[i][1][0], y: rawObstacleData[i][1][1], z: rawObstacleData[i][1][2] },
			vertices: [
				{ x: rawObstacleData[i][2][0][0], y: rawObstacleData[i][2][0][1], z: rawObstacleData[i][2][0][2] }, // now using the x, y, and z values as the dimensions (width, height, depth) rather than vertices
				// { x: rawObstacleData[i][2][1][0], y: rawObstacleData[i][2][1][1], z: rawObstacleData[i][2][1][2] },
				// { x: rawObstacleData[i][2][2][0], y: rawObstacleData[i][2][2][1], z: rawObstacleData[i][2][2][2] },
				// { x: rawObstacleData[i][2][3][0], y: rawObstacleData[i][2][3][1], z: rawObstacleData[i][2][3][2] },
				// { x: rawObstacleData[i][2][4][0], y: rawObstacleData[i][2][4][1], z: rawObstacleData[i][2][4][2] },
				// { x: rawObstacleData[i][2][5][0], y: rawObstacleData[i][2][5][1], z: rawObstacleData[i][2][5][2] },
				// { x: rawObstacleData[i][2][6][0], y: rawObstacleData[i][2][6][1], z: rawObstacleData[i][2][6][2] },
				// { x: rawObstacleData[i][2][7][0], y: rawObstacleData[i][2][7][1], z: rawObstacleData[i][2][7][2] },
			]
		});
	};

	return obs;
};

export default function PointcloudViewer({ frameData, objectList }: { frameData: Frame | null, objectList?: ObjectList | null }): JSX.Element {
	const canvasRef = useRef<HTMLCanvasElement>(null);
	const [scene, setScene] = useState<THREE.Scene | null>(null);
	const [camera, setCamera] = useState<THREE.Camera | null>(null);
	const [renderer, setRenderer] = useState<THREE.Renderer | null>(null);
	const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(null);
	const [material, setMaterial] = useState<THREE.Material | null>(null);
	const [points, setPoints] = useState<THREE.Points | null>(null);
	const [objects, setObjects] = useState<THREE.Mesh[] | null>(null);
	const [controls, setControls] = useState<OrbitControls | null>(null);

	useEffect(() => {
		if (!canvasRef.current) return;

		const scene = new THREE.Scene();
		const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
		const renderer = new THREE.WebGLRenderer({
			canvas: canvasRef.current,
			antialias: true,
		});

		const controls = new OrbitControls(camera, renderer.domElement);
		controls.enablePan = true;
		controls.enableZoom = true;
		controls.enableRotate = true;
		controls.enableDamping = true;
		setControls(controls);

		renderer.setSize(window.innerWidth, window.innerHeight);
		renderer.setClearColor(0x000000, 1);

		const geometry = new THREE.BufferGeometry();
		const material = new THREE.PointsMaterial({
			size: 0.01,
			color: 0x00ff00,
		});
		const points = new THREE.Points(geometry, material);

		scene.add(points);

		camera.position.z = 5;

		setScene(scene);
		setCamera(camera);
		setRenderer(renderer);
		setGeometry(geometry);
		setMaterial(material);
		setPoints(points);
	}, [canvasRef]);

	useEffect(() => {
		if (!geometry || !frameData) return;

		if(frameData.pc && frameData.pc.points) {
			const positions = new Float32Array(frameData.pc.points.length * 3);

			for (let i = 0; i < frameData.pc.points.length; i++) {
				positions[i * 3 + 0] = frameData.pc.points[i].x;
				positions[i * 3 + 1] = frameData.pc.points[i].y;
				positions[i * 3 + 2] = frameData.pc.points[i].z;
			}

			geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
		}

		if(frameData.obstacles && frameData.obstacles.objects) {
			frameData.obstacles.objects.map((o: Object) => {
				// const bufGeometry = new THREE.BufferGeometry();

				const objGeometry = new THREE.BoxGeometry(o.vertices[0].x, o.vertices[0].y, o.vertices[0].z);
				// const vertices = new Float32Array([
				// 	o.vertices[0].x, o.vertices[0].y, o.vertices[0].z,
				// 	o.vertices[1].x, o.vertices[1].y, o.vertices[1].z,
				// 	o.vertices[2].x, o.vertices[2].y, o.vertices[2].z,
				// 	o.vertices[3].x, o.vertices[3].y, o.vertices[3].z,
				// 	o.vertices[4].x, o.vertices[4].y, o.vertices[4].z,
				// 	o.vertices[5].x, o.vertices[5].y, o.vertices[5].z,
				// 	o.vertices[6].x, o.vertices[6].y, o.vertices[6].z,
				// 	o.vertices[7].x, o.vertices[7].y, o.vertices[7].z,
				// ]);

				// bufGeometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3)); // itemSize = 3 because there are 3 values per vertex (x, y, z)

				const objMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000, opacity: 0.3, transparent: true });
				const obj = new THREE.Mesh(objGeometry, objMaterial);

				obj.position.x = o.centroid.x;
				obj.position.y = o.centroid.y;
				obj.position.z = o.centroid.z;



				scene?.add(obj);
			});
		}

	}, [geometry, frameData]);

	// useEffect(() => {
	// 	if(!objectList || !scene) return;
	// 	
	// 	if(objects) {
	// 		for(let i = 0; i < objects?.length; i++) {
	// 			if(objects[i]) scene.remove(objects[i]);
	// 		}
	// 	}
	// 	
	// 	let newObjects: THREE.Mesh[] = [];
	//
	// 	objectList.objects.map((o: Object) => {
	// 		const objGeometry = new THREE.BoxGeometry(1, 1, 1);
	// 		const objMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000, opacity: 0.3, transparent: true });
	// 		const obj = new THREE.Mesh(objGeometry, objMaterial);
	//
	// 		obj.position.x = o.centroid.x;
	// 		obj.position.y = o.centroid.y;
	// 		obj.position.z = o.centroid.z;
	//
	// 		newObjects.push(obj);
	//
	// 		scene.add(obj);
	// 	});
	//
	// 	setObjects(newObjects);
	// }, [objectList]);

	useEffect(() => {
		if (!renderer || !scene || !camera) return;

		const animate = () => {
			requestAnimationFrame(animate);

			renderer.render(scene, camera);
		};

		animate();
	}, [renderer, scene, camera]);


	// Render dummy points and objects for testing
	useEffect(() => {
		if (!scene || !camera || !renderer) return;
		console.log('rendering');

		// const resize = () => {
		// 	camera.aspect = window.innerWidth / window.innerHeight;
		// 	camera.updateProjectionMatrix();
		//
		// 	renderer.setSize(window.innerWidth, window.innerHeight);
		// };
		//
		// window.addEventListener('resize', resize);
		//
		// return () => window.removeEventListener('resize', resize);
		
		// for(let i = 0; i < 5000; i++) {
		// 	const x: number = Math.random() * 3;
		// 	const y: number = Math.random() * 3;
		// 	const z: number = Math.random() * 3;
		//
		// 	const point = generatePoint(x, y, z);
		// 	
		// 	const material = new THREE.PointsMaterial({
		// 		size: 0.01,
		// 		color: 0x00ff00,
		// 	});
		//
		// 	const points = new THREE.Points(point, material);
		//
		// 	scene.add(points);
		// };

		
		// for(let i = 0; i < 15; i++) {
		// 	const geometry = new THREE.BoxGeometry(1, 1, 1);
		// 	const material = new THREE.MeshBasicMaterial({ color: 0xff0000, opacity: 0.3, transparent: true });
		// 	const cube = new THREE.Mesh(geometry, material);
		//
		// 	cube.position.x = Math.random() * 2;
		// 	cube.position.y = Math.random() * 2;
		// 	cube.position.z = Math.random() * 2;
		//
		// 	scene.add(cube);
		//
		// 	// console.log('added cube: ', cube);
		// };

		animate();
	}, [scene, renderer, camera]);

	function animate() {
		if(!renderer || !scene || !camera) return;
		requestAnimationFrame(animate);

		renderer.render(scene, camera);
	};

	function generatePoint(x: number, y: number, z: number): THREE.BufferGeometry {
		const g	= new THREE.BufferGeometry();
		const vertices = new Float32Array( [
			x, y, z,
		] );

		g.setAttribute( 'position', new THREE.BufferAttribute(vertices, 3));

		return g;
	};

	return (
		<canvas ref={canvasRef}>

		</canvas>
	)
};
