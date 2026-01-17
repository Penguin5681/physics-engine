import { Suspense, useEffect, useRef, useState } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { Sphere, Box, Plane, OrbitControls, useTexture, Environment } from "@react-three/drei";
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
  reset(): void;
  delete(): void;
}

interface PhysicsModule {
  PhysicsWorld: new () => PhysicsWorldInstance;
}

declare global {
  interface Window { createPhysicsModule: () => Promise<PhysicsModule>; }
}

// Visual Object Definition
type SimulationObject = 
  | { id: number; type: 'sphere'; radius: number }
  | { id: number; type: 'box'; size: [number, number, number] };

let physicsModule: PhysicsModule | null = null;

// --- Texture Components ---

const TextureManager = () => {
    // Preload textures to avoid pop-in
    useTexture.preload('https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/crate.gif');
    useTexture.preload('https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/uv_grid_opengl.jpg');
    useTexture.preload('https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/hardwood2_diffuse.jpg');
    return null;
}

// --- The Simulation Component ---
const PhysicsScene = ({ 
  objects, 
  gravity, 
  restitution, 
  simulationRunning 
}: { 
  objects: SimulationObject[], 
  gravity: number, 
  restitution: number,
  simulationRunning: boolean
}) => {
  const worldRef = useRef<PhysicsWorldInstance | null>(null);
  const meshRefs = useRef<(THREE.Mesh | null)[]>([]);

  // Load Textures
  const crateTexture = useTexture('https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/crate.gif');
  const sphereTexture = useTexture('https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/uv_grid_opengl.jpg');
  const floorTexture = useTexture('https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/hardwood2_diffuse.jpg');

  // Configure textures
  floorTexture.wrapS = floorTexture.wrapT = THREE.RepeatWrapping;
  floorTexture.repeat.set(10, 10);

  // 1. Initialize Physics World
  useEffect(() => {
    if (physicsModule && !worldRef.current) {
      worldRef.current = new physicsModule.PhysicsWorld();
    }
    return () => {
      worldRef.current?.delete();
      worldRef.current = null;
    };
  }, []);

  // 2. Sync Props
  useEffect(() => {
    if (worldRef.current) {
      worldRef.current.setGravity(gravity);
      worldRef.current.setRestitution(restitution);
    }
  }, [gravity, restitution]);

  // 3. Sync Objects
  useEffect(() => {
    if (!worldRef.current) return;
    
    const world = worldRef.current;
    const currentCount = world.getBodyCount();
    
    if (objects.length === 0 && currentCount > 0) {
        world.reset();
        return;
    }

    for (let i = currentCount; i < objects.length; i++) {
      const obj = objects[i];
      const startX = (Math.random() - 0.5) * 5;
      const startZ = (Math.random() - 0.5) * 5;
      const startY = 8 + (i * 1.5); 

      if (obj.type === 'sphere') {
        world.addSphere(startX, startY, startZ, obj.radius, 1.0);
      } else if (obj.type === 'box') {
        world.addBox(startX, startY, startZ, obj.size[0], obj.size[1], obj.size[2], 1.0);
      }
    }
  }, [objects]);

  // 4. Loop
  useFrame((_, delta) => {
    if (!worldRef.current || !simulationRunning) return;

    worldRef.current.step(Math.min(delta, 0.1));

    for (let i = 0; i < objects.length; i++) {
      const bodyData = worldRef.current.getBodyPosition(i);
      const mesh = meshRefs.current[i];
      
      if (bodyData && mesh) {
        mesh.position.set(bodyData.pos.x, bodyData.pos.y, bodyData.pos.z);
      }
    }
  });

  return (
    <>
      <TextureManager />
      {objects.map((obj, i) => (
        obj.type === 'sphere' ? (
          <Sphere 
            key={obj.id} 
            ref={(el) => (meshRefs.current[i] = el)} 
            args={[obj.radius, 32, 32]} 
            castShadow
          >
            <meshStandardMaterial map={sphereTexture} roughness={0.1} metalness={0.2} />
          </Sphere>
        ) : (
          <Box 
            key={obj.id} 
            ref={(el) => (meshRefs.current[i] = el)} 
            args={obj.size} 
            castShadow
          >
            <meshStandardMaterial map={crateTexture} />
          </Box>
        )
      ))}
      
      {/* Floor with Wood Texture */}
      <Plane args={[50, 50]} rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
        <meshStandardMaterial map={floorTexture} roughness={0.8} />
      </Plane>
    </>
  );
};

// --- Main App Component ---
function App() {
  const [ready, setReady] = useState(false);
  const [objects, setObjects] = useState<SimulationObject[]>([]);
  
  const [gravity, setGravity] = useState(-9.81);
  const [restitution, setRestitution] = useState(0.7);
  const [isRunning, setIsRunning] = useState(true);

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

  const spawnSphere = () => {
    setObjects(prev => [...prev, { id: Date.now(), type: 'sphere', radius: 0.5 + Math.random()*0.3 }]);
  };

  const spawnBox = () => {
    setObjects(prev => [...prev, { id: Date.now(), type: 'box', size: [1, 1, 1] }]);
  };

  const clearAll = () => setObjects([]);

  if (!ready) return <div className="loading">Initializing Engine...</div>;

  return (
    <div className="app-container">
      <div className="sidebar">
        <h3>Physics Engine</h3>
        
        <div className="section">
          <label>Gravity ({gravity})</label>
          <input 
            type="range" min="-20" max="0" step="0.1" 
            value={gravity} 
            onChange={(e) => setGravity(parseFloat(e.target.value))} 
          />
        </div>

        <div className="section">
          <label>Bounciness ({restitution})</label>
          <input 
            type="range" min="0" max="1.5" step="0.1" 
            value={restitution} 
            onChange={(e) => setRestitution(parseFloat(e.target.value))} 
          />
        </div>

        <div className="section actions">
            <button onClick={spawnSphere}>Spawn Sphere ⚪</button>
            <button onClick={spawnBox}>Spawn Crate 📦</button>
            <button className="danger" onClick={clearAll}>Clear All 🗑️</button>
            <button onClick={() => setIsRunning(!isRunning)}>
                {isRunning ? "Pause ⏸️" : "Resume ▶️"}
            </button>
        </div>
        
        <div className="stats">
            Entities: {objects.length}
        </div>
      </div>

      <div className="canvas-container">
        <Canvas shadows camera={{ position: [8, 8, 12], fov: 45 }}>
            {/* Realistic Lighting Setup */}
            <ambientLight intensity={0.5} />
            <spotLight 
                position={[15, 20, 10]} 
                angle={0.3} 
                penumbra={1} 
                intensity={2} 
                castShadow 
                shadow-mapSize={[2048, 2048]}
            />
            <Environment preset="sunset" />
            <OrbitControls />
          
            <Suspense fallback={null}>
                <PhysicsScene 
                    objects={objects} 
                    gravity={gravity} 
                    restitution={restitution}
                    simulationRunning={isRunning}
                />
            </Suspense>
        </Canvas>
      </div>
    </div>
  );
}

export default App;