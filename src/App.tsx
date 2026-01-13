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
  const [theme, setTheme] = useState<'light' | 'dark'>('light');

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

        <div className="control-group">
          <label>Gravity: {gravity}</label>
          <input
            type="range"
            min="-20"
            max="0"
            step="0.1"
            value={gravity}
            onChange={(e) => setGravity(parseFloat(e.target.value))}
          />
        </div>

        <div className="control-group">
          <label>Bounciness: {restitution}</label>
          <input
            type="range"
            min="0"
            max="2"
            step="0.1"
            value={restitution}
            onChange={(e) => setRestitution(parseFloat(e.target.value))}
          />
        </div>

        <button onClick={() => setResetTrigger((n) => n + 1)} style={{ padding: "10px", cursor: "pointer", marginTop: "10px" }}>
          Reset Box
        </button>
      </div>
      
      <div className="canvas-container">
        <Canvas shadows camera={{ position: [5, 5, 10], fov: 50 }}>
          <ambientLight intensity={0.3} />
          <spotLight
            position={[10, 10, 10]}
            angle={0.5}
            penumbra={0.5}
            intensity={2}
            castShadow
            shadow-mapSize={[1024, 1024]}
          />
          <directionalLight position={[-5, 5, 5]} intensity={0.5} castShadow />
          <Suspense fallback={null}>
            <Simulation gravity={gravity} restitution={restitution} resetTrigger={resetTrigger} />
          </Suspense>
          <OrbitControls />
        </Canvas>
      </div>
    </div>
  );
}

export default App;
