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
  getParticlePosition(index: number): Vector3;
  getParticleCount(): number;
  delete(): void;
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

const Simulation = () => {
  const worldRef = useRef<PhysicsWorldInstance | null>(null);
  const boxRef = useRef<Mesh>(null);

  useEffect(() => {
    if (physicsModule && !worldRef.current) {
      const world = new physicsModule.PhysicsWorld();
      world.addParticle(0, 10, 0);
      worldRef.current = world;
    }
  }, []);

  useFrame((_, delta) => {
    if (worldRef.current && boxRef.current) {
      worldRef.current.step(Math.min(delta, 0.1));

      const pos = worldRef.current.getParticlePosition(0);

      boxRef.current.position.set(pos.x, pos.y, pos.z);
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

  if (!ready) return <div>loading engine</div>;

  return (
    <div style={{ width: "80vw", height: "80vh" }}>
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
          <Simulation />
        </Suspense>
        <OrbitControls />
      </Canvas>
    </div>
  );
}

export default App;
