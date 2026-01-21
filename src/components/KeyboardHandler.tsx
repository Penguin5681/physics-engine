import { useEffect, useRef } from "react";
import { useFrame, useThree } from "@react-three/fiber";

interface KeyboardHandlerProps {
    setOrbitEnabled: (e: boolean) => void;
    onSpawn: () => void;
    onToggleShape: () => void;
}

export const KeyboardHandler = ({ 
    setOrbitEnabled,
    onSpawn,
    onToggleShape
}: KeyboardHandlerProps) => {
    const { camera } = useThree();
    const keys = useRef<Set<string>>(new Set());
    
    // Callbacks ref pattern
    const callbacks = useRef({ onSpawn, onToggleShape });
    callbacks.current = { onSpawn, onToggleShape };

    useEffect(() => {
        const handleDown = (e: KeyboardEvent) => {
            keys.current.add(e.code);
            // Edge detection for actions
            if (e.code === 'KeyB') callbacks.current.onSpawn();
            if (e.code === 'KeyX') callbacks.current.onToggleShape();
        };
        const handleUp = (e: KeyboardEvent) => {
            keys.current.delete(e.code);
        };
        window.addEventListener('keydown', handleDown);
        window.addEventListener('keyup', handleUp);
        
        // Disable orbit controls when mounted (Keyboard Mode implies Fly)
        setOrbitEnabled(false);
        
        return () => {
            window.removeEventListener('keydown', handleDown);
            window.removeEventListener('keyup', handleUp);
            setOrbitEnabled(true);
        };
    }, [setOrbitEnabled]);

    useFrame((_, delta) => {
        const moveSpeed = 10.0 * delta;
        const lookSpeed = 2.0 * delta;

        // Move
        if (keys.current.has('KeyW')) camera.translateZ(-moveSpeed);
        if (keys.current.has('KeyS')) camera.translateZ(moveSpeed);
        if (keys.current.has('KeyA')) camera.translateX(-moveSpeed);
        if (keys.current.has('KeyD')) camera.translateX(moveSpeed);
        
        // Up/Down
        // eslint-disable-next-line
        if (keys.current.has('Space')) camera.position.y += moveSpeed;
        if (keys.current.has('ShiftLeft') || keys.current.has('ShiftRight')) camera.position.y -= moveSpeed;

        // Look (Arrow Keys)
        if (keys.current.has('ArrowLeft')) {
             camera.rotation.y += lookSpeed;
        }
        if (keys.current.has('ArrowRight')) {
             camera.rotation.y -= lookSpeed;
        }
        if (keys.current.has('ArrowUp')) {
             camera.rotation.x += lookSpeed;
        }
        if (keys.current.has('ArrowDown')) {
             camera.rotation.x -= lookSpeed;
        }
        
        camera.rotation.x = Math.max(-1.5, Math.min(1.5, camera.rotation.x));
    });

    return null;
}
