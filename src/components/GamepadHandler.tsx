import { useRef } from "react";
import { useFrame, useThree } from "@react-three/fiber";

interface GamepadHandlerProps {
    setOrbitEnabled: (e: boolean) => void;
    onSpawn: () => void;
    onToggleShape: () => void;
}

export const GamepadHandler = ({ 
    setOrbitEnabled,
    onSpawn,
    onToggleShape
}: GamepadHandlerProps) => {
    const { camera } = useThree();
    
    const isControllerActive = useRef(false);
    const prevButtons = useRef<boolean[]>([]);
    
    const callbacks = useRef({ onSpawn, onToggleShape });
    callbacks.current = { onSpawn, onToggleShape };

    useFrame((_state, delta) => {
        const gamepads = navigator.getGamepads();
        const gp = gamepads[0];

        if (!gp) {
            if (isControllerActive.current) {
                isControllerActive.current = false;
                setOrbitEnabled(true);
            }
            return;
        }

        const deadzone = 0.1;
        const lx = Math.abs(gp.axes[0]) > deadzone ? gp.axes[0] : 0;
        const ly = Math.abs(gp.axes[1]) > deadzone ? gp.axes[1] : 0;
        const rx = Math.abs(gp.axes[2]) > deadzone ? gp.axes[2] : 0;
        const ry = Math.abs(gp.axes[3]) > deadzone ? gp.axes[3] : 0;

        if (Math.abs(lx) + Math.abs(ly) + Math.abs(rx) + Math.abs(ry) > 0.1 || gp.buttons.some(b => b.pressed)) {
            if (!isControllerActive.current) {
                isControllerActive.current = true;
                setOrbitEnabled(false);
            }
        }

        if (!isControllerActive.current) return;

        // Button 1 (B / Circle) -> Spawn
        if (gp.buttons[1].pressed && !prevButtons.current[1]) {
            callbacks.current.onSpawn();
        }
        // Button 2 (X / Square) -> Switch Shape
        if (gp.buttons[2].pressed && !prevButtons.current[2]) {
            callbacks.current.onToggleShape();
        }

        prevButtons.current = gp.buttons.map(b => b.pressed);

        const moveSpeed = 10.0 * delta;
        const lookSpeed = 2.0 * delta;

        camera.translateX(lx * moveSpeed);
        camera.translateZ(ly * moveSpeed);

        // eslint-disable-next-line
        camera.rotation.y -= rx * lookSpeed;
        camera.rotation.x -= ry * lookSpeed;
        camera.rotation.x = Math.max(-1.5, Math.min(1.5, camera.rotation.x));
        
        if (gp.buttons[4].pressed || gp.buttons[6].pressed) camera.position.y -= moveSpeed;
        if (gp.buttons[5].pressed || gp.buttons[7].pressed) camera.position.y += moveSpeed;
    });

    return null;
}
