export interface Vector3 { x: number; y: number; z: number; }
export interface BodyData { pos: Vector3; rot: { w: number, x: number, y: number, z: number } }

export interface PhysicsWorldInstance {
  addSphere(x: number, y: number, z: number, radius: number, mass: number): void;
  addBox(x: number, y: number, z: number, w: number, h: number, d: number, mass: number): void;
  step(dt: number): void;
  getBodyPosition(index: number): BodyData | null;
  getBodyCount(): number;
  setGravity(g: number): void;
  setRestitution(r: number): void;
  setFriction(f: number): void;
  setVelocity(index: number, x: number, y: number, z: number): void;
  applyForce(index: number, x: number, y: number, z: number): void; 
  reset(): void;
  delete(): void;
}

export interface PhysicsModule {
  PhysicsWorld: new () => PhysicsWorldInstance;
}

export type TextureType = 'wood' | 'metal' | 'bricks' | 'grid';

export interface SimulationObject {
  id: number;
  type: 'sphere' | 'box';
  size: [number, number, number] | [number]; 
  textureId: TextureType;
  initialPos?: [number, number, number];
}

export type ShapeType = 'sphere' | 'box';
export type InputMode = 'keyboard' | 'controller';

export const TEXTURES: Record<TextureType, string> = {
    wood: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/crate.gif',
    metal: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/planets/earth_atmos_2048.jpg',
    bricks: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/brick_diffuse.jpg',
    grid: 'https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/uv_grid_opengl.jpg'
};
