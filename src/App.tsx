import { Suspense, useEffect, useRef, useState } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { Box, Plane, useTexture, OrbitControls } from "@react-three/drei";
import { Mesh } from "three";
import "./App.css";

interface Vector3 {
  x: number;
  y: number;
  z: number;
}

interface PhysicsWorldInstance {
  addParticle(x: number, y: number, z: number): void;
  step(dt: number): void;
  getParticlePosition(index: number): Vector3 | null;
  getParticleCount(): number;
  delete(): void;
  setGravity(g: number): void;
  setRestitution(r: number): void;
  reset(): void;
}

interface PhysicsModule {
  PhysicsWorld: new () => PhysicsWorldInstance;
}

interface RoadmapItem {
  id: number;
  todo: string;
  description: string;
  is_completed: boolean;
  sequence_number: number;
  created_at: string;
}

declare global {
  interface Window {
    createPhysicsModule: () => Promise<PhysicsModule>;
  }
}

let physicsModule: PhysicsModule | null = null;

interface SimulationProps {
  gravity: number;
  restitution: number;
  resetTrigger: number;
}

const API_BASE_URL = "https://applicable-v1-backend-671108073568.europe-west1.run.app";

const Simulation = ({ gravity, restitution, resetTrigger }: SimulationProps) => {
  const worldRef = useRef<PhysicsWorldInstance | null>(null);
  const boxRef = useRef<Mesh>(null);

  useEffect(() => {
    if (physicsModule && !worldRef.current) {
      const world = new physicsModule.PhysicsWorld();
      world.addParticle(0, 10, 0);
      worldRef.current = world;
    }
  }, []);

  useEffect(() => {
    if (worldRef.current) {
      worldRef.current.setGravity(gravity);
      worldRef.current.setRestitution(restitution);
    }
  }, [gravity, restitution]);

  useEffect(() => {
    if (worldRef.current && resetTrigger > 0) {
      worldRef.current.reset();
      worldRef.current.addParticle(0, 10, 0);
    }
  }, [resetTrigger]);

  useFrame((_, delta) => {
    if (worldRef.current && boxRef.current) {
      worldRef.current.step(Math.min(delta, 0.1));

      const pos = worldRef.current.getParticlePosition(0);
      if (pos) {
        boxRef.current.position.set(pos.x, pos.y, pos.z);
      }
    }
  });

  const texture = useTexture('https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/crate.gif');

  return (
    <>
      <Box ref={boxRef} args={[1, 1, 1]} position={[0, 10, 0]} castShadow>
        <meshStandardMaterial map={texture} />
      </Box>
      <Plane args={[20, 20]} rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
        <meshStandardMaterial color="gray" />
      </Plane>
    </>
  );
};

function App() {
  const [ready, setReady] = useState(false);
  
  const [gravity, setGravity] = useState(-9.81);
  const [restitution, setRestitution] = useState(0.5);
  const [resetTrigger, setResetTrigger] = useState(0);
  
  const [theme, setTheme] = useState<'light' | 'dark'>('dark');

  const [ambientIntensity, setAmbientIntensity] = useState(0.3);
  const [spotIntensity, setSpotIntensity] = useState(2);
  const [spotAngle, setSpotAngle] = useState(0.5);
  const [spotPenumbra, setSpotPenumbra] = useState(0.5);
  const [spotPos, setSpotPos] = useState<[number, number, number]>([10, 10, 10]);
  const [dirIntensity, setDirIntensity] = useState(0.5);
  const [dirPos, setDirPos] = useState<[number, number, number]>([-5, 5, 5]);
  const [castShadows, setCastShadows] = useState(true);

  const [isRoadmapOpen, setIsRoadmapOpen] = useState(false);
  const [roadmapItems, setRoadmapItems] = useState<RoadmapItem[]>([]);

  useEffect(() => {
    if (isRoadmapOpen) {
      fetch(`${API_BASE_URL}/api/roadmap`)
        .then((res) => res.json())
        .then((data) => setRoadmapItems(data.sort((a: RoadmapItem, b: RoadmapItem) => a.sequence_number - b.sequence_number)))
        .catch((err) => console.error("Failed to fetch roadmap", err));
    }
  }, [isRoadmapOpen]);

  const toggleRoadmapItem = (id: number, currentStatus: boolean) => {
    const newStatus = !currentStatus;
    setRoadmapItems((items) =>
      items.map((item) => (item.id === id ? { ...item, is_completed: newStatus } : item))
    );

    fetch(`${API_BASE_URL}/api/roadmap/${id}/status`, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ is_completed: newStatus }),
    }).catch((err) => {
      console.error("Failed to update status", err);
      setRoadmapItems((items) =>
        items.map((item) => (item.id === id ? { ...item, is_completed: currentStatus } : item))
      );
    });
  };

  const resetControls = () => {
    setGravity(-9.81);
    setRestitution(0.5);
    setAmbientIntensity(0.3);
    setSpotIntensity(2);
    setSpotAngle(0.5);
    setSpotPenumbra(0.5);
    setSpotPos([10, 10, 10]);
    setDirIntensity(0.5);
    setDirPos([-5, 5, 5]);
    setCastShadows(true);
  };

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

  if (!ready) return <div style={{ padding: "20px" }}>Loading Physics Engine...</div>;

  return (
    <div className={`app-container ${theme}`}>
      <div className="sidebar">
        <h3>Controls</h3>
        
        <div className="control-group">
          <label>Theme</label>
          <button onClick={() => setTheme((t) => (t === "light" ? "dark" : "light"))}>
            Switch to {theme === "light" ? "Dark" : "Light"} Mode
          </button>
        </div>

        <button onClick={resetControls} style={{ padding: "10px", cursor: "pointer", marginTop: "10px", width: "100%", marginBottom: "10px" }}>
          Reset Controls
        </button>

        <h4>Physics</h4>
        <div className="control-group">
          <label>Gravity: {gravity}</label>
          <input type="range" min="-20" max="0" step="0.1" value={gravity} onChange={(e) => setGravity(parseFloat(e.target.value))} />
        </div>
        <div className="control-group">
          <label>Bounciness: {restitution}</label>
          <input type="range" min="0" max="2" step="0.1" value={restitution} onChange={(e) => setRestitution(parseFloat(e.target.value))} />
        </div>
        <button onClick={() => setResetTrigger((n) => n + 1)} style={{ padding: "10px", cursor: "pointer", marginTop: "10px", width: "100%" }}>
          Reset Box
        </button>
        
        <button onClick={() => setIsRoadmapOpen(true)} style={{ padding: "10px", cursor: "pointer", marginTop: "10px", width: "100%" }}>
          View Roadmap
        </button>

        <h4>Lighting</h4>
        <div className="control-group">
          <label>
            <input type="checkbox" checked={castShadows} onChange={(e) => setCastShadows(e.target.checked)} /> Cast Shadows
          </label>
        </div>
        <div className="control-group">
          <label>Ambient Intensity: {ambientIntensity}</label>
          <input type="range" min="0" max="2" step="0.1" value={ambientIntensity} onChange={(e) => setAmbientIntensity(parseFloat(e.target.value))} />
        </div>

        <h4>Spot Light</h4>
        <div className="control-group">
          <label>Intensity: {spotIntensity}</label>
          <input type="range" min="0" max="10" step="0.1" value={spotIntensity} onChange={(e) => setSpotIntensity(parseFloat(e.target.value))} />
        </div>
        <div className="control-group">
          <label>Angle: {spotAngle}</label>
          <input type="range" min="0" max="1.5" step="0.1" value={spotAngle} onChange={(e) => setSpotAngle(parseFloat(e.target.value))} />
        </div>
        <div className="control-group">
          <label>Penumbra: {spotPenumbra}</label>
          <input type="range" min="0" max="1" step="0.1" value={spotPenumbra} onChange={(e) => setSpotPenumbra(parseFloat(e.target.value))} />
        </div>
        <div className="control-group">
          <label>Pos X: {spotPos[0]}</label>
          <input type="range" min="-20" max="20" step="1" value={spotPos[0]} onChange={(e) => setSpotPos([parseFloat(e.target.value), spotPos[1], spotPos[2]])} />
        </div>
        <div className="control-group">
          <label>Pos Y: {spotPos[1]}</label>
          <input type="range" min="0" max="30" step="1" value={spotPos[1]} onChange={(e) => setSpotPos([spotPos[0], parseFloat(e.target.value), spotPos[2]])} />
        </div>
        <div className="control-group">
          <label>Pos Z: {spotPos[2]}</label>
          <input type="range" min="-20" max="20" step="1" value={spotPos[2]} onChange={(e) => setSpotPos([spotPos[0], spotPos[1], parseFloat(e.target.value)])} />
        </div>

        <h4>Directional Light</h4>
        <div className="control-group">
          <label>Intensity: {dirIntensity}</label>
          <input type="range" min="0" max="5" step="0.1" value={dirIntensity} onChange={(e) => setDirIntensity(parseFloat(e.target.value))} />
        </div>
        <div className="control-group">
          <label>Pos X: {dirPos[0]}</label>
          <input type="range" min="-20" max="20" step="1" value={dirPos[0]} onChange={(e) => setDirPos([parseFloat(e.target.value), dirPos[1], dirPos[2]])} />
        </div>
        <div className="control-group">
          <label>Pos Y: {dirPos[1]}</label>
          <input type="range" min="0" max="30" step="1" value={dirPos[1]} onChange={(e) => setDirPos([dirPos[0], parseFloat(e.target.value), dirPos[2]])} />
        </div>
        <div className="control-group">
          <label>Pos Z: {dirPos[2]}</label>
          <input type="range" min="-20" max="20" step="1" value={dirPos[2]} onChange={(e) => setDirPos([dirPos[0], dirPos[1], parseFloat(e.target.value)])} />
        </div>
      </div>
      
      <div className="canvas-container">
        <Canvas shadows={castShadows} camera={{ position: [5, 5, 10], fov: 50 }}>
          <ambientLight intensity={ambientIntensity} />
          <spotLight
            position={spotPos}
            angle={spotAngle}
            penumbra={spotPenumbra}
            intensity={spotIntensity}
            castShadow={castShadows}
            shadow-mapSize={[1024, 1024]}
          />
          <directionalLight position={dirPos} intensity={dirIntensity} castShadow={castShadows} />
          <Suspense fallback={null}>
            <Simulation gravity={gravity} restitution={restitution} resetTrigger={resetTrigger} />
          </Suspense>
          <OrbitControls />
        </Canvas>
      </div>

      {isRoadmapOpen && (
        <div className="modal-overlay" onClick={() => setIsRoadmapOpen(false)}>
          <div className={`modal-content ${theme}`} onClick={(e) => e.stopPropagation()}>
            <button className="close-btn" onClick={() => setIsRoadmapOpen(false)}>
              &times;
            </button>
            <h2>Project Roadmap</h2>
            <div className="roadmap-list">
              {roadmapItems.map((item) => (
                <div key={item.id} className="roadmap-item">
                  <input
                    type="checkbox"
                    checked={item.is_completed}
                    onChange={() => toggleRoadmapItem(item.id, item.is_completed)}
                  />
                  <div>
                    <strong>{item.todo}</strong>
                    <p>{item.description}</p>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

export default App;
