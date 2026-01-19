import { Suspense, useEffect, useRef, useState, useMemo } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Sphere, Box, Plane, PointerLockControls, useTexture, Environment, Html } from "@react-three/drei";
import * as THREE from "three";
import "./App.css";

// --- Types ---
interface Vector3 { x: number; y: number; z: number; }
interface BodyData { pos: Vector3; rot: { w: number, x: number, y: number, z: number } }

interface PhysicsWorldInstance {
  addSphere(x: number, y: number, z: number, radius: number, mass: number): void;
  addBox(x: number, y: number, z: number, w: number, h: number, d: number, mass: number): void;
  step(dt: number): void;
  getBodyPosition(index: number): BodyData | null;
  getBodyCount(): number;
  setGravity(g: number): void;
  setRestitution(r: number): void;
  setFriction(f: number): void;
  setVelocity(index: number, x: number, y: number, z: number): void;
  reset(): void;
  delete(): void;
}

interface PhysicsModule {
  PhysicsWorld: new () => PhysicsWorldInstance;
}

declare global {
  interface Window { createPhysicsModule: () => Promise<PhysicsModule>; }
}

type TextureType = 'wood' | 'metal' | 'bricks' | 'grid';

interface BaseSimulationObject {
  id: number;
  textureId: TextureType;
  // We add initial velocity support for shooting
  initialPos?: [number, number, number];
  initialVel?: [number, number, number];
}

interface SphereObject extends BaseSimulationObject {
  type: 'sphere';
  size: [number];
}

interface BoxObject extends BaseSimulationObject {
  type: 'box';
  size: [number, number, number];
}

type SimulationObject = SphereObject | BoxObject;

let physicsModule: PhysicsModule | null = null;

const TEXTURES = {
    wood: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/crate.gif',
    metal: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/planets/earth_atmos_2048.jpg',
    bricks: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/brick_diffuse.jpg',
    grid: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/uv_grid_opengl.jpg'
};

// --- FPS Controller Component ---
const FirstPersonPlayer = ({ onShoot, onUnlock }: { onShoot: (pos: THREE.Vector3, vel: THREE.Vector3) => void, onUnlock: () => void }) => {
    const { camera } = useThree();
    const moveForward = useRef(false);
    const moveBackward = useRef(false);
    const moveLeft = useRef(false);
    const moveRight = useRef(false);
    const moveUp = useRef(false);
    const moveDown = useRef(false);

    useEffect(() => {
        const onKeyDown = (event: KeyboardEvent) => {
            switch (event.code) {
                case 'KeyW': moveForward.current = true; break;
                case 'KeyS': moveBackward.current = true; break;
                case 'KeyA': moveLeft.current = true; break;
                case 'KeyD': moveRight.current = true; break;
                case 'Space': moveUp.current = true; break;
                case 'ShiftLeft': moveDown.current = true; break;
            }
        };
        const onKeyUp = (event: KeyboardEvent) => {
            switch (event.code) {
                case 'KeyW': moveForward.current = false; break;
                case 'KeyS': moveBackward.current = false; break;
                case 'KeyA': moveLeft.current = false; break;
                case 'KeyD': moveRight.current = false; break;
                case 'Space': moveUp.current = false; break;
                case 'ShiftLeft': moveDown.current = false; break;
            }
        };
        const onMouseDown = () => {
            if (document.pointerLockElement) {
                const direction = new THREE.Vector3();
                camera.getWorldDirection(direction);
                const position = camera.position.clone();
                // Add offset so we don't spawn inside ourselves
                position.add(direction.clone().multiplyScalar(1.5));
                
                onShoot(position, direction.multiplyScalar(20)); // 20 is shoot speed
            }
        };
        document.addEventListener('keydown', onKeyDown);
        document.addEventListener('keyup', onKeyUp);
        document.addEventListener('mousedown', onMouseDown);
        return () => {
            document.removeEventListener('keydown', onKeyDown);
            document.removeEventListener('keyup', onKeyUp);
            document.removeEventListener('mousedown', onMouseDown);
        };
    }, [camera, onShoot]);

    useFrame((_, delta) => {
        // WASD Movement Logic
        const speed = 15.0 * delta; // Movement speed
        
        if (moveForward.current) camera.translateZ(-speed);
        if (moveBackward.current) camera.translateZ(speed);
        if (moveLeft.current) camera.translateX(-speed);
        if (moveRight.current) camera.translateX(speed);
        if (moveUp.current) camera.position.setY(camera.position.y + speed);
        if (moveDown.current) camera.position.setY(camera.position.y - speed);

        // Floor constraint (Optional: Keep player above ground)
        if (camera.position.y < 1.5) camera.position.setY(1.5);
    });

    return <PointerLockControls onUnlock={onUnlock} />;
};

// --- Scene Component ---
const PhysicsScene = ({ 
  objects, 
  gravity, 
  restitution, 
  friction,
  simulationRunning 
}: { 
  objects: SimulationObject[], 
  gravity: number, 
  restitution: number,
  friction: number,
  simulationRunning: boolean
}) => {
  const worldRef = useRef<PhysicsWorldInstance | null>(null);
  const meshRefs = useRef<(THREE.Mesh | null)[]>([]);
  const maps = useTexture(TEXTURES);

  // Configure Floor Texture for "Infinite" look
  const rawFloorTexture = useTexture('https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/hardwood2_diffuse.jpg');
  const floorTexture = useMemo(() => {
      const t = rawFloorTexture.clone();
      t.wrapS = t.wrapT = THREE.RepeatWrapping;
      t.repeat.set(100, 100);
      return t;
  }, [rawFloorTexture]);

  useEffect(() => {
    if (physicsModule && !worldRef.current) {
      worldRef.current = new physicsModule.PhysicsWorld();
    }
    return () => {
      worldRef.current?.delete();
      worldRef.current = null;
    };
  }, []);

  useEffect(() => {
    if (worldRef.current) {
      worldRef.current.setGravity(gravity);
      worldRef.current.setRestitution(restitution);
      worldRef.current.setFriction(friction);
    }
  }, [gravity, restitution, friction]);

  useEffect(() => {
    if (!worldRef.current) return;
    const world = worldRef.current;
    
    if (objects.length === 0 && world.getBodyCount() > 0) {
        world.reset();
        return;
    }

    const currentCount = world.getBodyCount();
    for (let i = currentCount; i < objects.length; i++) {
      const obj = objects[i];
      // Use custom initial pos if provided (for shooting), else random
      const x = obj.initialPos ? obj.initialPos[0] : (Math.random() - 0.5) * 5;
      const y = obj.initialPos ? obj.initialPos[1] : 5 + (i * 2);
      const z = obj.initialPos ? obj.initialPos[2] : (Math.random() - 0.5) * 5;

      if (obj.type === 'sphere') {
        world.addSphere(x, y, z, obj.size[0], 1.0);
      } else {
        world.addBox(x, y, z, obj.size[0], obj.size[1], obj.size[2], 1.0);
      }
      
      if (obj.initialVel) {
          const iv = obj.initialVel!;
          world.setVelocity(i, iv[0]!, iv[1]!, iv[2]!);
      }
    }
  }, [objects]);

  useFrame((_, delta) => {
    if (!worldRef.current || !simulationRunning) return;
    worldRef.current.step(Math.min(delta, 0.1));

    for (let i = 0; i < objects.length; i++) {
      const bodyData = worldRef.current.getBodyPosition(i);
      const mesh = meshRefs.current[i];
      if (bodyData && mesh) {
        mesh.position.set(bodyData.pos.x, bodyData.pos.y, bodyData.pos.z);
        if (bodyData.rot) {
            mesh.quaternion.set(bodyData.rot.x, bodyData.rot.y, bodyData.rot.z, bodyData.rot.w);
        }
      }
    }
  });

  return (
    <>
      {objects.map((obj, i) => {
          const textureMap = maps[obj.textureId];
          return obj.type === 'sphere' ? (
            <Sphere key={obj.id} ref={(el) => { meshRefs.current[i] = el; }} args={[obj.size[0], 32, 32]} castShadow>
                <meshStandardMaterial map={textureMap} />
            </Sphere>
          ) : (
            <Box key={obj.id} ref={(el) => { meshRefs.current[i] = el; }} args={obj.size as [number, number, number]} castShadow>
                <meshStandardMaterial map={textureMap} />
            </Box>
          );
      })}

      {/* "Infinite" Floor: Huge Plane + Fog handles the illusion */}
      <Plane args={[1000, 1000]} rotation={[-Math.PI / 2, 0, 0]} receiveShadow position={[0, -0.01, 0]}>
        <meshStandardMaterial map={floorTexture} roughness={0.8} />
      </Plane>
    </>
  );
};

// --- Main App ---
function App() {
  const [ready, setReady] = useState(false);
  const [objects, setObjects] = useState<SimulationObject[]>([]);
  const [gravity, setGravity] = useState(-9.81);
  const [restitution, setRestitution] = useState(0.6);
  const [friction, setFriction] = useState(0.5);
  const [selectedTexture, setSelectedTexture] = useState<TextureType>('wood');
  const [isPlaying, setIsPlaying] = useState(false); // Track if user clicked "Play"

  useEffect(() => {
    const script = document.createElement("script");
    script.src = "/wasm/physics.js";
    script.async = true;
    script.onload = () => {
      window.createPhysicsModule().then((module) => {
        physicsModule = module;
        setReady(true);
      });
    };
    document.body.appendChild(script);
  }, []);

  // Standard spawn
  const spawn = (type: 'sphere' | 'box') => {
      const newObj: SimulationObject = type === 'sphere' 
        ? {
            id: Date.now(),
            type: 'sphere',
            size: [0.6],
            textureId: selectedTexture
        }
        : {
            id: Date.now(),
            type: 'box',
            size: [1, 1, 1],
            textureId: selectedTexture
        };
      setObjects(prev => [...prev, newObj]);
  };

  // Shoot function
  const handleShoot = (pos: THREE.Vector3, vel: THREE.Vector3) => {
      const newObj: SimulationObject = {
          id: Date.now(),
          type: 'sphere',
          size: [0.6],
          textureId: selectedTexture,
          initialPos: [pos.x, pos.y, pos.z],
          initialVel: [vel.x, vel.y, vel.z]
      };
      setObjects(prev => [...prev, newObj]);
  };

  if (!ready) return <div className="loader">Loading Engine...</div>;

  return (
    <div className="app-layout">
      <div className="viewport" onClick={() => setIsPlaying(true)}>
          <Canvas shadows camera={{ position: [0, 5, 10], fov: 60 }}>
            <color attach="background" args={['#87CEEB']} /> {/* Sky Blue */}
            <fog attach="fog" args={['#87CEEB', 20, 100]} /> {/* Distance Fog */}
            
            <ambientLight intensity={0.5} />
            <directionalLight position={[50, 50, 25]} castShadow shadow-mapSize={[2048, 2048]} />
            <Environment preset="park" />

            {/* Controls Switch */}
            {isPlaying ? (
                <FirstPersonPlayer 
                    onShoot={handleShoot} 
                    onUnlock={() => setIsPlaying(false)} 
                />
            ) : null}

            <Suspense fallback={<Html center>Loading...</Html>}>
                <PhysicsScene 
                    objects={objects}
                    gravity={gravity}
                    restitution={restitution}
                    friction={friction}
                    simulationRunning={true}
                />
            </Suspense>
          </Canvas>

          {/* Crosshair */}
          {isPlaying && <div className="crosshair">+</div>}

          {/* Hint Overlay */}
          {!isPlaying && (
              <div className="start-overlay">
                  <h1>Click to Start Simulation</h1>
                  <p>WASD to Move • Space/Shift to Fly • Mouzse to Look • ESC to Release</p>
              </div>
          )}
      </div>

      <div className="sidebar">
        <div className="sidebar-header">
            <h2>🕹️ Physics Sandbox</h2>
        </div>

        <div className="control-group">
            <label>Gravity</label>
            <input type="range" min="-20" max="0" step="0.1" value={gravity} onChange={e => setGravity(Number(e.target.value))} />
        </div>
        
        <div className="control-group">
            <label>Bounciness</label>
            <input type="range" min="0" max="1.5" step="0.1" value={restitution} onChange={e => setRestitution(Number(e.target.value))} />
        </div>

        <div className="control-group">
            <label>Friction</label>
            <input type="range" min="0" max="1" step="0.1" value={friction} onChange={e => setFriction(Number(e.target.value))} />
        </div>

        <hr />
        
        <div className="texture-grid">
            {(Object.keys(TEXTURES) as TextureType[]).map(tex => (
                <button 
                    key={tex}
                    className={`texture-btn ${selectedTexture === tex ? 'active' : ''}`}
                    onClick={() => setSelectedTexture(tex)}
                    style={{ backgroundImage: `url(${TEXTURES[tex]})` }}
                />
            ))}
        </div>

        <div className="action-buttons">
            <button className="spawn-btn" onClick={() => spawn('box')}>📦 Spawn Crate</button>
            <button className="spawn-btn" onClick={() => spawn('sphere')}>⚪ Spawn Ball</button>
            <button className="clear-btn" onClick={() => setObjects([])}>Clear All</button>
        </div>
      </div>
    </div>
  );
}

export default App;
