import { forwardRef, useImperativeHandle, useRef, useEffect, useState } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import { Sphere, Box, Plane, OrbitControls, useTexture, Grid } from "@react-three/drei";
import * as THREE from "three";
import { TEXTURES, type PhysicsWorldInstance, type SimulationObject, type TextureType, type ShapeType, type InputMode, type PhysicsModule } from "../types";
import { GamepadHandler } from "./GamepadHandler";
import { KeyboardHandler } from "./KeyboardHandler";
import { MouseHandler } from "./MouseHandler";

interface PhysicsSceneProps {
    objects: SimulationObject[];
    setObjects: React.Dispatch<React.SetStateAction<SimulationObject[]>>;
    gravity: number;
    restitution: number;
    friction: number;
    debugMode: boolean;
    inputMode: InputMode;
    selectedShape: ShapeType;
    selectedTexture: TextureType;
    onToggleShape: () => void;
    physicsModule: PhysicsModule | null;
}

export interface PhysicsSceneRef {
    spawn: (type?: ShapeType) => void;
}

export const PhysicsScene = forwardRef<PhysicsSceneRef, PhysicsSceneProps>(({
    objects,
    setObjects,
    gravity,
    restitution,
    friction,
    debugMode,
    inputMode,
    selectedShape,
    selectedTexture,
    onToggleShape,
    physicsModule
}, ref) => {
    const { camera } = useThree();
    const worldRef = useRef<PhysicsWorldInstance | null>(null);
    const meshRefs = useRef<(THREE.Mesh | null)[]>([]);
    const maps = useTexture(TEXTURES);
    const [orbitEnabled, setOrbitEnabled] = useState(true);

    const performSpawn = (type?: ShapeType) => {
        const shapeType = type || selectedShape;
        
        // Spawn 5 units in front of camera
        const spawnDist = 5;
        const spawnPos = new THREE.Vector3();
        camera.getWorldDirection(spawnPos);
        spawnPos.multiplyScalar(spawnDist);
        spawnPos.add(camera.position);

        const newObj: SimulationObject = shapeType === 'sphere' 
            ? { id: Date.now(), type: 'sphere', size: [0.6], textureId: selectedTexture }
            : { id: Date.now(), type: 'box', size: [1, 1, 1], textureId: selectedTexture };
        
        setObjects(prev => [...prev, { ...newObj, initialPos: [spawnPos.x, spawnPos.y, spawnPos.z] }]);
    };

    useImperativeHandle(ref, () => ({
        spawn: performSpawn
    }));

    useEffect(() => {
        if (physicsModule && !worldRef.current) {
            worldRef.current = new physicsModule.PhysicsWorld();
        }
        return () => { worldRef.current?.delete(); worldRef.current = null; };
    }, [physicsModule]);

    useEffect(() => {
        if (worldRef.current) {
            worldRef.current.setGravity(gravity);
            worldRef.current.setRestitution(restitution);
            if(worldRef.current.setFriction) worldRef.current.setFriction(friction);
        }
    }, [gravity, restitution, friction]);

    useEffect(() => {
        if (!worldRef.current) return;
        const world = worldRef.current;
        if (objects.length === 0 && world.getBodyCount() > 0) { world.reset(); return; }

        const currentCount = world.getBodyCount();
        for (let i = currentCount; i < objects.length; i++) {
            const obj = objects[i];
            const initPos = obj.initialPos;
            let x, y, z;
            if (initPos) {
                [x, y, z] = initPos;
            } else {
                x = (Math.random() - 0.5) * 5;
                z = (Math.random() - 0.5) * 5;
                y = 8 + (i * 2);
            }
            if (obj.type === 'sphere') world.addSphere(x, y, z, obj.size[0], 1.0);
            else {
                // box size array is [w, h, d]
                const w = obj.size[0];
                const h = obj.size[1] ?? 1;
                const d = obj.size[2] ?? 1;
                world.addBox(x, y, z, w, h, d, 1.0);
            }
        }
    }, [objects]);

    useFrame((_, delta) => {
        if (!worldRef.current) return;
        worldRef.current.step(Math.min(delta, 0.1));

        for (let i = 0; i < objects.length; i++) {
            const bodyData = worldRef.current.getBodyPosition(i);
            const mesh = meshRefs.current[i];
            if (bodyData && mesh) {
                mesh.position.set(bodyData.pos.x, bodyData.pos.y, bodyData.pos.z);
                if (bodyData.rot) mesh.quaternion.set(bodyData.rot.x, bodyData.rot.y, bodyData.rot.z, bodyData.rot.w);
            }
        }
    });

    return (
        <>
            {inputMode === 'controller' && (
                <GamepadHandler 
                    setOrbitEnabled={setOrbitEnabled} 
                    onSpawn={() => performSpawn()} 
                    onToggleShape={onToggleShape} 
                />
            )}
            {inputMode === 'keyboard' && (
                <KeyboardHandler 
                    setOrbitEnabled={setOrbitEnabled} 
                    onSpawn={() => performSpawn()} 
                    onToggleShape={onToggleShape} 
                />
            )}
            
            <MouseHandler worldRef={worldRef} />
            
            <OrbitControls 
                enabled={orbitEnabled}
                makeDefault 
                onChange={(e) => {
                   // eslint-disable-next-line @typescript-eslint/no-explicit-any
                   if(e?.target?.object?.parent) (e.target.object.parent as any).userData.controls = e.target;
                }}
            />

            {objects.map((obj, i) => {
                const textureMap = maps[obj.textureId];
                return obj.type === 'sphere' ? (
                    <Sphere key={obj.id} ref={el => {meshRefs.current[i]=el}} args={[obj.size[0], 32, 32]} castShadow userData={{ physicsId: i }}>
                        <meshStandardMaterial map={textureMap} wireframe={debugMode} />
                    </Sphere>
                ) : (
                    <Box key={obj.id} ref={el => {meshRefs.current[i]=el}} args={obj.size.length === 3 ? [obj.size[0], obj.size[1]!, obj.size[2]!] : [1, 1, 1]} castShadow userData={{ physicsId: i }}>
                        <meshStandardMaterial map={textureMap} wireframe={debugMode} />
                    </Box>
                );
            })}

            {/* Shadow-receiving infinite floor */}
            <Plane args={[50000, 50000]} rotation={[-Math.PI / 2, 0, 0]} receiveShadow position={[0, -0.01, 0]}>
                <meshStandardMaterial color="#1a1a1a" roughness={0.8} />
            </Plane>
            
            {/* Infinite Grid using drei component */}
            <Grid 
                position={[0, 0.01, 0]} 
                args={[100, 100]} 
                cellSize={1} 
                sectionSize={5} 
                infiniteGrid 
                fadeDistance={100} 
                sectionColor="#555555" 
                cellColor="#333333" 
            />
        </>
    );
});
