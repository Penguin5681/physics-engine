import { Suspense, useEffect, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { Environment, Html } from "@react-three/drei";
import "./App.css";
import type { SimulationObject, TextureType, ShapeType, InputMode, PhysicsModule } from "./types";
import { Sidebar } from "./components/Sidebar";
import { PhysicsScene, type PhysicsSceneRef } from "./components/PhysicsScene";

declare global {
  interface Window { createPhysicsModule: () => Promise<PhysicsModule>; }
}

let physicsModule: PhysicsModule | null = null;

function App() {
  const [ready, setReady] = useState(false);
  const [objects, setObjects] = useState<SimulationObject[]>([]);
  const [gravity, setGravity] = useState(-9.81);
  const [restitution, setRestitution] = useState(0.6);
  const [friction, setFriction] = useState(0.5);
  const [selectedTexture, setSelectedTexture] = useState<TextureType>('wood');
  const [debugMode, setDebugMode] = useState(false);
  const [selectedShape, setSelectedShape] = useState<ShapeType>('box');
  const [inputMode, setInputMode] = useState<InputMode>('keyboard');

  const physicsSceneRef = useRef<PhysicsSceneRef>(null);

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

  const handleSpawn = (type: ShapeType) => {
      if (physicsSceneRef.current) {
          physicsSceneRef.current.spawn(type);
      }
  };
  
  const toggleShape = () => {
      setSelectedShape(prev => prev === 'box' ? 'sphere' : prev === 'sphere' ? 'cylinder' : 'box');
  };

  if (!ready) return <div className="loader">Loading Engine...</div>;

  return (
    <div className="app-layout">
      <div className="viewport">
          <Canvas shadows camera={{ position: [8, 5, 8], fov: 50 }}>
            <color attach="background" args={['#534d4d']} /> 
            
            <ambientLight intensity={0.5} />
            <spotLight position={[10, 20, 10]} angle={0.3} penumbra={1} castShadow shadow-mapSize={[2048, 2048]} />
            <Environment preset="city" />

            <Suspense fallback={<Html center>Loading Textures...</Html>}>
                <PhysicsScene 
                    ref={physicsSceneRef}
                    objects={objects}
                    setObjects={setObjects}
                    gravity={gravity}
                    restitution={restitution}
                    friction={friction}
                    debugMode={debugMode}
                    inputMode={inputMode}
                    selectedShape={selectedShape}
                    selectedTexture={selectedTexture}
                    onToggleShape={toggleShape}
                    physicsModule={physicsModule}
                />
            </Suspense>
          </Canvas>

          <div className="overlay-info">
             {inputMode === 'keyboard' && (
                 <>
                    ⌨️ <strong>Keyboard:</strong> WASD (Move) | Arrows (Look) | Space/Shift (Up/Down) <br/>
                    Enter/'B' (Spawn) | 'X' (Switch Shape)
                 </>
             )}
             {inputMode === 'controller' && (
                 <>
                    🎮 <strong>Gamepad:</strong> Left Stick (Move) | Right Stick (Look) <br/>
                    Btn 'X' (Switch Shape) | Btn 'B' (Spawn)
                 </>
             )}
             <br/>
             Selected: {selectedShape}
          </div>
      </div>

      <Sidebar 
          gravity={gravity} setGravity={setGravity}
          restitution={restitution} setRestitution={setRestitution}
          friction={friction} setFriction={setFriction}
          debugMode={debugMode} setDebugMode={setDebugMode}
          selectedTexture={selectedTexture} setSelectedTexture={setSelectedTexture}
          selectedShape={selectedShape} setSelectedShape={setSelectedShape}
          inputMode={inputMode} setInputMode={setInputMode}
          onSpawn={handleSpawn}
          onClear={() => setObjects([])}
      />
    </div>
  );
}

export default App;
