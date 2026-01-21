import { TEXTURES, type TextureType, type ShapeType, type InputMode } from "../types";

interface SidebarProps {
    gravity: number;
    setGravity: (v: number) => void;
    restitution: number;
    setRestitution: (v: number) => void;
    friction: number;
    setFriction: (v: number) => void;
    debugMode: boolean;
    setDebugMode: (v: boolean) => void;
    selectedTexture: TextureType;
    setSelectedTexture: (v: TextureType) => void;
    selectedShape: ShapeType;
    setSelectedShape: (v: ShapeType) => void;
    inputMode: InputMode;
    setInputMode: (v: InputMode) => void;
    onSpawn: (type: ShapeType) => void;
    onClear: () => void;
}

export const Sidebar = ({
    gravity, setGravity,
    restitution, setRestitution,
    friction, setFriction,
    debugMode, setDebugMode,
    selectedTexture, setSelectedTexture,
    selectedShape, setSelectedShape,
    inputMode, setInputMode,
    onSpawn, onClear
}: SidebarProps) => {
    return (
      <div className="sidebar">
        <div className="sidebar-header"><h2>🛠️ Physics Workbench</h2></div>

        <div className="control-group">
            <label>Control Mode</label>
            <div className="row-btns">
                <button className={`toggle-btn ${inputMode === 'keyboard' ? 'active' : ''}`} onClick={() => setInputMode('keyboard')}>Keyboard</button>
                <button className={`toggle-btn ${inputMode === 'controller' ? 'active' : ''}`} onClick={() => setInputMode('controller')}>Controller</button>
            </div>
        </div>
        
        <hr />

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
        
        <div className="control-group">
            <label>Display</label>
            <button className={`toggle-btn ${debugMode ? 'active' : ''}`} onClick={() => setDebugMode(!debugMode)}>
                {debugMode ? 'Wireframe ON' : 'Wireframe OFF'}
            </button>
        </div>

        <hr />

        <div className="texture-grid">
            {(Object.keys(TEXTURES) as TextureType[]).map(tex => (
                <button key={tex} className={`texture-btn ${selectedTexture === tex ? 'active' : ''}`} onClick={() => setSelectedTexture(tex)} style={{ backgroundImage: `url(${TEXTURES[tex]})` }} />
            ))}
        </div>

        <div className="action-buttons">
            <button className={`spawn-btn ${selectedShape === 'box' ? 'active' : ''}`} onClick={() => { setSelectedShape('box'); onSpawn('box'); }}>+ Box</button>
            <button className={`spawn-btn ${selectedShape === 'sphere' ? 'active' : ''}`} onClick={() => { setSelectedShape('sphere'); onSpawn('sphere'); }}>+ Sphere</button>
            <button className="clear-btn" onClick={onClear}>Clear All</button>
        </div>
      </div>
    );
};
