import { Suspense, useEffect, useRef, useState } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { Sphere, Box, Plane, OrbitControls, useTexture, Environment, Html } from "@react-three/drei";
import * as THREE from "three";
import "./App.css";

// --- Types ---
interface Vector3 { x: number; y: number; z: number; }
interface BodyData { pos: Vector3; }

interface PhysicsWorldInstance {
  addSphere(x: number, y: number, z: number, radius: number, mass: number): void;
  addBox(x: number, y: number, z: number, w: number, h: number, d: number, mass: number): void;
  step(dt: number): void;
  getBodyPosition(index: number): BodyData | null;
  getBodyCount(): number;
  setGravity(g: number): void;
  setRestitution(r: number): void;
  setFriction(f: number): void; // Ensure C++ has this exposed!
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

type SimulationObject = 
  | { id: number; type: 'sphere'; size: [number]; textureId: TextureType }
  | { id: number; type: 'box'; size: [number, number, number]; textureId: TextureType };

let physicsModule: PhysicsModule | null = null;

// --- Texture Definitions ---
const TEXTURES = {
    wood: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/crate.gif',
    metal: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/planets/earth_atmos_2048.jpg', // Using earth as metal placeholder for cool effect
    bricks: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/brick_diffuse.jpg',
    grid: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/uv_grid_opengl.jpg'
};

// --- Scene Component (Handles Rendering) ---
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

  // Load all textures upfront
  const maps = useTexture(TEXTURES);

  // Initialize Physics
  useEffect(() => {
    if (physicsModule && !worldRef.current) {
      worldRef.current = new physicsModule.PhysicsWorld();
    }
    return () => {
      worldRef.current?.delete();
      worldRef.current = null;
    };
  }, []);

  // Sync Physics Parameters
  useEffect(() => {
    if (worldRef.current) {
      worldRef.current.setGravity(gravity);
      worldRef.current.setRestitution(restitution);
      worldRef.current.setFriction(friction);
    }
  }, [gravity, restitution, friction]);

  // Sync Object Spawning
  useEffect(() => {
    if (!worldRef.current) return;
    const world = worldRef.current;
    
    // Check for Reset
    if (objects.length === 0 && world.getBodyCount() > 0) {
        world.reset();
        return;
    }

    // Add new objects
    const currentCount = world.getBodyCount();
    for (let i = currentCount; i < objects.length; i++) {
      const obj = objects[i];
      // Randomize spawn position slightly to avoid perfect stacking
      const x = (Math.random() - 0.5) * 2;
      const z = (Math.random() - 0.5) * 2;
      const y = 5 + (i * 2);

      if (obj.type === 'sphere') {
        world.addSphere(x, y, z, obj.size[0], 1.0);
      } else {
        world.addBox(x, y, z, obj.size[0], obj.size[1], obj.size[2], 1.0);
      }
    }
  }, [objects]);

  // Animation Loop
  useFrame((_, delta) => {
    if (!worldRef.current || !simulationRunning) return;

    // Run Physics Step
    worldRef.current.step(Math.min(delta, 0.1));

    // Sync Meshes
    for (let i = 0; i < objects.length; i++) {
      const bodyData = worldRef.current.getBodyPosition(i);
      const mesh = meshRefs.current[i];
      if (bodyData && mesh) {
        mesh.position.set(bodyData.pos.x, bodyData.pos.y, bodyData.pos.z);

        mesh.quaternion.set(
          bodyData.rot.x,
          bodyData.rot.y,
          bodyData.rot.z,
          bodyData.rot.w,
        );
      }
    }
  });

  return (
    <>
      {objects.map((obj, i) => {
          const textureMap = maps[obj.textureId];
          return obj.type === 'sphere' ? (
            <Sphere 
                key={obj.id} 
                ref={(el) => { meshRefs.current[i] = el; }} 
                args={[obj.size[0], 32, 32]} 
                castShadow
            >
                <meshStandardMaterial map={textureMap} roughness={0.2} metalness={0.1} />
            </Sphere>
          ) : (
            <Box 
                key={obj.id} 
                ref={(el) => { meshRefs.current[i] = el; }} 
                args={obj.size} 
                castShadow
            >
                <meshStandardMaterial map={textureMap} />
            </Box>
          );
      })}

      {/* Floor */}
      <Plane args={[100, 100]} rotation={[-Math.PI / 2, 0, 0]} receiveShadow position={[0, -0.01, 0]}>
        <meshStandardMaterial color="#1a1a1a" roughness={0.8} />
        <gridHelper args={[100, 50, 0x444444, 0x222222]} rotation={[-Math.PI/2, 0, 0]} />
      </Plane>
    </>
  );
};

// --- Main Application ---
function App() {
  const [ready, setReady] = useState(false);
  const [objects, setObjects] = useState<SimulationObject[]>([]);
  
  // Physics State
  const [gravity, setGravity] = useState(-9.81);
  const [restitution, setRestitution] = useState(0.6);
  const [friction, setFriction] = useState(0.5);
  const [isRunning, setIsRunning] = useState(true);

  // UI State
  const [selectedTexture, setSelectedTexture] = useState<TextureType>('wood');

  // Load Wasm
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

  const spawn = (type: 'sphere' | 'box') => {
      const newObj: SimulationObject = type === 'sphere' 
        ? { id: Date.now(), type: 'sphere', size: [0.6], textureId: selectedTexture }
        : { id: Date.now(), type: 'box', size: [1, 1, 1], textureId: selectedTexture };

      setObjects(prev => [...prev, newObj]);
  };

  if (!ready) return <div className="loader">Loading Physics Engine...</div>;

  return (
    <div className="app-layout">
      {/* 3D Viewport */}
      <div className="viewport">
          <Canvas shadows camera={{ position: [8, 6, 8], fov: 45 }}>
            <color attach="background" args={['#111']} />
            <fog attach="fog" args={['#111', 10, 40]} />
            
            <ambientLight intensity={0.4} />
            <spotLight 
                position={[10, 20, 10]} 
                angle={0.25} 
                penumbra={1} 
                intensity={1.5} 
                castShadow 
                shadow-mapSize={[2048, 2048]} 
            />
            <Environment preset="city" />
            <OrbitControls />

            <Suspense fallback={<Html center>Loading Textures...</Html>}>
                <PhysicsScene 
                    objects={objects}
                    gravity={gravity}
                    restitution={restitution}
                    friction={friction}
                    simulationRunning={isRunning}
                />
            </Suspense>
          </Canvas>

          {/* Overlay Stats */}
          <div className="stats-overlay">
              <span>{objects.length} Objects</span>
              <span>{isRunning ? 'Running' : 'Paused'}</span>
          </div>
      </div>

      {/* Control Sidebar */}
      <div className="sidebar">
        <div className="sidebar-header">
            <h2>⚛️ PhysX Engine</h2>
            <p>Interactive WASM Playground</p>
        </div>

        <div className="control-group">
            <label>Gravity ({gravity.toFixed(1)})</label>
            <input 
                type="range" min="-20" max="0" step="0.1" 
                value={gravity} 
                onChange={e => setGravity(Number(e.target.value))} 
            />
        </div>

        <div className="control-group">
            <label>Bounciness ({restitution.toFixed(1)})</label>
            <input 
                type="range" min="0" max="1.5" step="0.1" 
                value={restitution} 
                onChange={e => setRestitution(Number(e.target.value))} 
            />
        </div>

        <div className="control-group">
            <label>Friction ({friction.toFixed(1)})</label>
            <input 
                type="range" min="0" max="1" step="0.1" 
                value={friction} 
                onChange={e => setFriction(Number(e.target.value))} 
            />
        </div>

        <hr />

        <div className="control-group">
            <label>Texture</label>
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
        </div>

        <div className="action-buttons">
            <button className="spawn-btn" onClick={() => spawn('sphere')}>
                <span>⚪</span> Spawn Sphere
            </button>
            <button className="spawn-btn" onClick={() => spawn('box')}>
                <span>📦</span> Spawn Box
            </button>
            
            <div className="row-btns">
                <button 
                    className={`toggle-btn ${isRunning ? 'active' : ''}`} 
                    onClick={() => setIsRunning(!isRunning)}
                >
                    {isRunning ? "Pause" : "Resume"}
                </button>
                <button className="clear-btn" onClick={() => setObjects([])}>
                    Clear
                </button>
            </div>
        </div>
      </div>
    </div>
  );
}

export default App;
